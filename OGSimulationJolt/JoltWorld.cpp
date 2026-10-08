// SPDX-License-Identifier: MPL-2.0
// docs/JoltWorld-rationale.md · docs/JoltWorld-guards.md

#include "OGSimulationJolt/JoltWorld.h"

#include <algorithm>
#include <bit>
#include <chrono>
#include <cstdio>
#include <variant>

#include <Jolt/Core/Reference.h>
#include <Jolt/Physics/Body/Body.h>
#include <Jolt/Physics/Body/BodyLockInterface.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/CapsuleShape.h>
#include <Jolt/Physics/Collision/Shape/RotatedTranslatedShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>

#include "OGSimulation/OGAssert.h"
#include "OGSimulationJolt/JoltFpEnvironment.h"
#include "OGSimulationJolt/JoltUnits.h"

namespace
{
	double steadyClockSeconds()
	{
		return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
	}

	JoltLayerKey templateLayerKeyOf(const PhysicalObjectDescriptor& descriptor)
	{
		const ShapeDescriptor& shape = descriptor.shapes.front();
		return JoltLayerKey{ JoltBroadPhaseClass::Moving, shape.categories.bits, shape.blockingCategories.bits };
	}

	std::vector<JoltLayerKey> layerKeysOf(const JoltWorldConfig& config)
	{
		std::vector<JoltLayerKey> keys = config.staticLayers;
		for (const SlotBodyTemplate& slotTemplate : config.slotTemplate)
		{
			OG_CHECK(slotTemplate.descriptor.shapes.size() == 1,
				"JoltWorld: a slot template body must have exactly one shape (multi-shape slot bodies are not supported in M1)");
			if (!slotTemplate.descriptor.shapes.empty())
			{
				keys.push_back(templateLayerKeyOf(slotTemplate.descriptor));
			}
		}
		for (const JoltLayerKey& key : config.staticLayers)
		{
			OG_CHECK(key.broadPhaseClass == JoltBroadPhaseClass::Static, "JoltWorld: JoltWorldConfig::staticLayers holds a non-static key");
		}
		return keys;
	}

	JPH::RefConst<JPH::Shape> shapeOf(const QueryGeometry& geometry)
	{
		if (const SphereGeometry* sphere = std::get_if<SphereGeometry>(&geometry))
		{
			return new JPH::SphereShape(joltUnits::centimetresToMetres(sphere->radius));
		}
		if (const BoxGeometry* box = std::get_if<BoxGeometry>(&geometry))
		{
			const JPH::Vec3 halfExtents = joltUnits::centimetresToMetres(box->halfExtents);
			const float convexRadius = std::min(JPH::cDefaultConvexRadius, halfExtents.ReduceMin());
			return new JPH::BoxShape(halfExtents, convexRadius);
		}
		const CapsuleGeometry& capsule = std::get<CapsuleGeometry>(geometry);
		const float radius = joltUnits::centimetresToMetres(capsule.radius);
		const float cylinderHalfHeight = joltUnits::centimetresToMetres(capsule.halfHeight - capsule.radius);
		OG_CHECK(cylinderHalfHeight >= 0.f, "JoltWorld: CapsuleGeometry::halfHeight is the TOTAL half height and must be >= radius");
		if (cylinderHalfHeight <= 0.f)
		{
			return new JPH::SphereShape(radius);
		}
		const JPH::Quat yAxisToZAxis = JPH::Quat::sRotation(JPH::Vec3::sAxisX(), 0.5f * JPH::JPH_PI);
		return new JPH::RotatedTranslatedShape(JPH::Vec3::sZero(), yAxisToZAxis, new JPH::CapsuleShape(cylinderHalfHeight, radius));
	}

	constexpr const char* overflowName(JoltContactOverflow kind)
	{
		switch (kind)
		{
		case JoltContactOverflow::ManifoldCacheFull: return "ManifoldCacheFull";
		case JoltContactOverflow::BodyPairCacheFull: return "BodyPairCacheFull";
		case JoltContactOverflow::ContactConstraintsFull: return "ContactConstraintsFull";
		case JoltContactOverflow::Count: break;
		}
		return "?";
	}

	constexpr const char* overflowRemedy(JoltContactOverflow kind)
	{
		switch (kind)
		{
		case JoltContactOverflow::ManifoldCacheFull: return "raise JoltWorldConfig::maxContactConstraints";
		case JoltContactOverflow::BodyPairCacheFull: return "raise JoltWorldConfig::maxBodyPairs";
		case JoltContactOverflow::ContactConstraintsFull: return "raise JoltWorldConfig::maxContactConstraints";
		case JoltContactOverflow::Count: break;
		}
		return "?";
	}

	constexpr JPH::EPhysicsUpdateError overflowFlag(JoltContactOverflow kind)
	{
		switch (kind)
		{
		case JoltContactOverflow::ManifoldCacheFull: return JPH::EPhysicsUpdateError::ManifoldCacheFull;
		case JoltContactOverflow::BodyPairCacheFull: return JPH::EPhysicsUpdateError::BodyPairCacheFull;
		case JoltContactOverflow::ContactConstraintsFull: return JPH::EPhysicsUpdateError::ContactConstraintsFull;
		case JoltContactOverflow::Count: break;
		}
		return JPH::EPhysicsUpdateError::None;
	}

	class SaveFilter final : public JPH::StateRecorderFilter
	{
	public:
		SaveFilter(const JPH::BodyLockInterface& bodies, std::vector<JPH::BodyID>& savedBodies)
			: m_bodies(bodies)
			, m_savedBodies(savedBodies)
		{
		}

		bool ShouldSaveBody(const JPH::Body& body) const override
		{
			if (body.IsStatic())
			{
				return false;
			}
			m_savedBodies.push_back(body.GetID());
			return true;
		}

		bool ShouldSaveContact(const JPH::BodyID& first, const JPH::BodyID& second) const override
		{
			return isNonStatic(first) || isNonStatic(second);
		}

	private:
		bool isNonStatic(const JPH::BodyID& id) const
		{
			const JPH::Body* body = m_bodies.TryGetBody(id);
			return body != nullptr && !body->IsStatic();
		}

		const JPH::BodyLockInterface& m_bodies;
		std::vector<JPH::BodyID>& m_savedBodies;
	};

	class StateHash
	{
	public:
		void bytes(const void* data, size_t count)
		{
			const auto* bytes = static_cast<const uint8_t*>(data);
			for (size_t i = 0; i < count; ++i)
			{
				m_value ^= bytes[i];
				m_value *= 1099511628211ull;
			}
		}

		void u32(uint32_t value)
		{
			uint8_t littleEndian[4];
			for (int i = 0; i < 4; ++i)
			{
				littleEndian[i] = static_cast<uint8_t>(value >> (8 * i));
			}
			bytes(littleEndian, sizeof(littleEndian));
		}

		void f32(float value) { u32(std::bit_cast<uint32_t>(value)); }

		[[nodiscard]] uint64_t value() const { return m_value; }

	private:
		uint64_t m_value = 14695981039346656037ull;
	};
}

JoltWorld::JoltWorld(JoltRuntime& runtime, const JoltWorldConfig& config, JoltWorldLoggerFn logger)
	: m_logger(std::move(logger))
	, m_clock(config.clock != nullptr ? config.clock : &steadyClockSeconds)
	, m_simulatableSlots(config.simulatableSlots)
	, m_layers(layerKeysOf(config))
	, m_tempAllocator(config.tempAllocatorBytes)
	, m_jobSystem(JPH::cMaxPhysicsJobs)
	, m_ring(config.ringDepthTicks, config.ringSlotBytes,
		config.simulatableSlots * static_cast<uint32_t>(config.slotTemplate.size()))
{
	(void)runtime;
	OG_CHECK(config.simulatableSlots <= kMaxSimulatableSlots, "JoltWorld: simulatableSlots exceeds kMaxSimulatableSlots (BodySlotOccupancy has one bit per slot)");
	m_simulatableSlots = std::min(config.simulatableSlots, kMaxSimulatableSlots);

	m_physics.Init(config.maxBodies, 0, config.maxBodyPairs, config.maxContactConstraints,
		m_layers.broadPhaseLayers(), m_layers.objectVsBroadPhaseFilter(), m_layers.objectLayerPairFilter());
	// ⛔G-01  docs/JoltWorld-guards.md
	JPH::PhysicsSettings settings = m_physics.GetPhysicsSettings();
	settings.mMinVelocityForRestitution = joltUnits::centimetresToMetres(config.minVelocityForRestitutionCmPerS);
	m_physics.SetPhysicsSettings(settings);
	m_physics.SetGravity(joltUnits::centimetresToMetres(config.gravityCmPerS2));

	m_liveBodies.reserve(config.maxBodies);
	buildSlotBodies(config);
}

JoltWorld::~JoltWorld()
{
	JPH::BodyInterface& bodyInterface = bodies();
	m_physics.GetBodies(m_liveBodies);
	for (const JPH::BodyID& id : m_liveBodies)
	{
		if (bodyInterface.IsAdded(id))
		{
			bodyInterface.RemoveBody(id);
		}
		bodyInterface.DestroyBody(id);
	}
}

void JoltWorld::buildSlotBodies(const JoltWorldConfig& config)
{
	JPH::BodyInterface& bodyInterface = bodies();
	const JPH::RVec3 parkingPosition = joltUnits::centimetresToMetres(config.parkingPositionCm);

	std::vector<JPH::RefConst<JPH::Shape>> shapes;
	for (const SlotBodyTemplate& slotTemplate : config.slotTemplate)
	{
		const PhysicalObjectDescriptor& descriptor = slotTemplate.descriptor;
		const bool hasShape = !descriptor.shapes.empty();
		m_template.push_back(TemplateBody{
			hasShape ? m_layers.layerOf(templateLayerKeyOf(descriptor)) : m_layers.parkedLayer(),
			descriptor.body.enableGravity ? 1.f : 0.f });
		shapes.push_back(hasShape ? shapeOf(descriptor.shapes.front().geometry) : JPH::RefConst<JPH::Shape>(new JPH::SphereShape(0.01f)));
	}

	for (uint32_t slot = 0; slot < m_simulatableSlots; ++slot)
	{
		for (uint32_t templateIndex = 0; templateIndex < m_template.size(); ++templateIndex)
		{
			const BodyDescriptor& body = config.slotTemplate[templateIndex].descriptor.body;
			JPH::BodyCreationSettings settings(shapes[templateIndex], parkingPosition, JPH::Quat::sIdentity(),
				body.simulatePhysics ? JPH::EMotionType::Dynamic : JPH::EMotionType::Kinematic, m_layers.parkedLayer());
			settings.mAllowSleeping = false;
			settings.mGravityFactor = 0.f;
			if (body.lockRotation)
			{
				settings.mAllowedDOFs = JPH::EAllowedDOFs::TranslationX | JPH::EAllowedDOFs::TranslationY | JPH::EAllowedDOFs::TranslationZ;
			}

			const JPH::BodyID id = slotBodyId(slot, templateIndex);
			JPH::Body* created = bodyInterface.CreateBodyWithID(id, settings);
			OG_CHECK(created != nullptr, "JoltWorld: CreateBodyWithID failed for a slot body (maxBodies too small, or the id is taken)");
			if (created != nullptr)
			{
				bodyInterface.AddBody(id, JPH::EActivation::Activate);
			}
		}
	}
	m_nextStaticBodyIndex = 1u + m_simulatableSlots * static_cast<uint32_t>(m_template.size());

	applyOccupancyToSlots(BodySlotOccupancy{}, true);
}

JPH::BodyID JoltWorld::slotBodyId(uint32_t slot, uint32_t templateIndex) const
{
	OG_CHECK(slot < m_simulatableSlots && templateIndex < m_template.size(), "JoltWorld::slotBodyId - slot or template index out of range");
	return JPH::BodyID(1u + slot * static_cast<uint32_t>(m_template.size()) + templateIndex);
}

JPH::ObjectLayer JoltWorld::templateLayer(uint32_t templateIndex) const
{
	OG_CHECK(templateIndex < m_template.size(), "JoltWorld::templateLayer - template index out of range");
	return m_template[templateIndex].layer;
}

JPH::BodyID JoltWorld::createStaticBody(const JPH::BodyCreationSettings& settings)
{
	OG_CHECK(settings.mMotionType == JPH::EMotionType::Static, "JoltWorld::createStaticBody - the settings are not static");
	OG_CHECK(m_layers.keyOf(settings.mObjectLayer).broadPhaseClass == JoltBroadPhaseClass::Static,
		"JoltWorld::createStaticBody - the object layer is not a STATIC-class layer of this world's table");
	const JPH::BodyID id(m_nextStaticBodyIndex);
	JPH::Body* created = bodies().CreateBodyWithID(id, settings);
	OG_CHECK(created != nullptr, "JoltWorld::createStaticBody - CreateBodyWithID failed (maxBodies too small?)");
	if (created == nullptr)
	{
		return JPH::BodyID();
	}
	++m_nextStaticBodyIndex;
	return id;
}

void JoltWorld::step(float dt)
{
	JoltStepFpScope fpScope;
	OG_CHECK(isExpectedJoltStepFpMode(readJoltFpMode()),
		"JoltWorld::step - the FP environment is not FTZ + DAZ + round-to-nearest inside the step scope; every thread must step under the same FP mode");

	const JPH::EPhysicsUpdateError errors = m_physics.Update(dt, 1, &m_tempAllocator, &m_jobSystem);
	++m_physicsStepCount;
	if (errors != JPH::EPhysicsUpdateError::None)
	{
		reportUpdateErrors(errors);
	}
}

void JoltWorld::reportUpdateErrors(JPH::EPhysicsUpdateError errors)
{
	const double now = m_clock();
	for (size_t index = 0; index < m_overflow.size(); ++index)
	{
		const JoltContactOverflow kind = static_cast<JoltContactOverflow>(index);
		if ((errors & overflowFlag(kind)) == JPH::EPhysicsUpdateError::None)
		{
			continue;
		}
		OverflowLog& entry = m_overflow[index];
		++entry.occurrences;
		if (entry.lastLineSeconds.has_value() && now - *entry.lastLineSeconds < 1.0)
		{
			continue;
		}
		char lastTick[16] = "none";
		if (m_lastSavedTick.has_value())
		{
			std::snprintf(lastTick, sizeof(lastTick), "%u", *m_lastSavedTick);
		}
		char line[320];
		std::snprintf(line, sizeof(line),
			"[Warning] JoltWorld: Jolt dropped contacts: %s at physics step %llu (last saved sim tick %s); %llu step(s) with this flag since the last line; %s",
			overflowName(kind), static_cast<unsigned long long>(m_physicsStepCount), lastTick,
			static_cast<unsigned long long>(entry.occurrences - entry.occurrencesAtLastLine), overflowRemedy(kind));
		log(line);
		entry.lastLineSeconds = now;
		entry.occurrencesAtLastLine = entry.occurrences;
	}
}

void JoltWorld::saveTick(SimTick tick)
{
	JoltStateSlot* slot = m_ring.slotForSave(tick);
	if (slot == nullptr)
	{
		return;
	}
	saveInto(*slot, tick);
	m_lastSavedTick = tick;
}

void JoltWorld::saveScratch()
{
	saveInto(m_ring.scratch(), 0u);
}

void JoltWorld::commitScratch(SimTick tick)
{
	m_ring.commit(m_ring.scratch(), tick);
	m_lastSavedTick = tick;
}

void JoltWorld::saveInto(JoltStateSlot& slot, SimTick tick)
{
	slot.valid = false;
	slot.bodyIds.clear();
	SaveFilter filter(m_physics.GetBodyLockInterfaceNoLock(), slot.bodyIds);
	m_recorder.beginWrite(slot.bytes.data(), slot.bytes.size());
	m_physics.SaveState(m_recorder, JPH::EStateRecorderState::All, &filter);
	if (m_recorder.IsFailed())
	{
		char line[192];
		std::snprintf(line, sizeof(line),
			"[Warning] JoltWorld: snapshot of tick %u does not fit in JoltWorldConfig::ringSlotBytes (%u); the tick is not held",
			tick, static_cast<unsigned>(slot.bytes.size()));
		log(line);
		OG_CHECK(false, "JoltWorld: a snapshot overflowed its ring slot; raise JoltWorldConfig::ringSlotBytes");
		return;
	}
	slot.byteCount = static_cast<uint32_t>(m_recorder.bytesWritten());
	slot.sidecar = ConfigSidecar{ m_appliedOccupancy, m_shapeEnables.bits() };
	slot.stateHash = liveStateHash();
	slot.tick = tick;
	slot.valid = true;
}

bool JoltWorld::restoreTick(SimTick tick)
{
	const JoltStateSlot* slot = m_ring.find(tick);
	if (slot == nullptr)
	{
		return false;
	}
	return restoreSlot(*slot, RestoreUndo::Take, "restoreTick");
}

bool JoltWorld::restoreFromSnapshot(const JoltStateSlot& slot)
{
	if (!slot.valid)
	{
		log("[Warning] JoltWorld: restoreFromSnapshot refused: the slot holds no snapshot; the world is unchanged");
		return false;
	}
	return restoreSlot(slot, RestoreUndo::Skip, "restoreFromSnapshot");
}

bool JoltWorld::restoreSlot(const JoltStateSlot& slot, RestoreUndo undo, const char* caller)
{
	if (!liveBodySetMatches(slot))
	{
		char line[224];
		std::snprintf(line, sizeof(line),
			"[Warning] JoltWorld: %s(%u) refused: the snapshot's body set differs from the live body set; the world is unchanged",
			caller, slot.tick);
		log(line);
		return false;
	}

	JoltStateSlot* undoSlot = nullptr;
	if (undo == RestoreUndo::Take)
	{
		undoSlot = &m_ring.restoreUndo();
		saveInto(*undoSlot, slot.tick);
	}

	if (!restoreBytes(slot))
	{
		const bool undone = undoSlot != nullptr && undoSlot->valid && restoreBytes(*undoSlot);
		char line[224];
		std::snprintf(line, sizeof(line),
			"[Warning] JoltWorld: %s: RestoreState failed for tick %u after pre-validation; the pre-restore world was %s",
			caller, slot.tick, undone ? "restored" : (undoSlot != nullptr ? "NOT restored" : "not saved (no undo)"));
		log(line);
		OG_CHECK(false, "JoltWorld::restoreTick / restoreFromSnapshot - Jolt RestoreState returned false on a pre-validated snapshot");
		return false;
	}

	applyOccupancyToSlots(slot.sidecar.occupancy, true);
	m_shapeEnables.restore(slot.sidecar.shapeEnables);
	return true;
}

bool JoltWorld::restoreBytes(const JoltStateSlot& slot)
{
	m_recorder.beginRead(slot.bytes.data(), slot.byteCount);
	const bool restored = m_physics.RestoreState(m_recorder);
	return restored && !m_recorder.IsFailed() && m_recorder.fullyRead();
}

bool JoltWorld::liveBodySetMatches(const JoltStateSlot& slot)
{
	const JPH::BodyLockInterface& lockInterface = m_physics.GetBodyLockInterfaceNoLock();
	for (const JPH::BodyID& id : slot.bodyIds)
	{
		const JPH::Body* body = lockInterface.TryGetBody(id);
		if (body == nullptr || !body->IsInBroadPhase() || body->IsStatic())
		{
			return false;
		}
	}

	m_physics.GetBodies(m_liveBodies);
	size_t liveNonStatic = 0;
	for (const JPH::BodyID& id : m_liveBodies)
	{
		const JPH::Body* body = lockInterface.TryGetBody(id);
		if (body != nullptr && body->IsInBroadPhase() && !body->IsStatic())
		{
			++liveNonStatic;
		}
	}
	return liveNonStatic == slot.bodyIds.size();
}

bool JoltWorld::hasTick(SimTick tick) const
{
	return m_ring.find(tick) != nullptr;
}

std::optional<SimTick> JoltWorld::oldestHeldTick() const
{
	return m_ring.oldestHeldTick();
}

void JoltWorld::invalidateAllTicks()
{
	m_ring.invalidateAll();
}

std::optional<uint64_t> JoltWorld::stateHash(SimTick tick) const
{
	const JoltStateSlot* slot = m_ring.find(tick);
	return slot != nullptr ? std::optional<uint64_t>(slot->stateHash) : std::nullopt;
}

void JoltWorld::applyOccupancy(const BodySlotOccupancy& occupancy)
{
	applyOccupancyToSlots(occupancy, false);
}

void JoltWorld::applyOccupancyToSlots(const BodySlotOccupancy& occupancy, bool forceEverySlot)
{
	JPH::BodyInterface& bodyInterface = bodies();
	for (uint32_t slot = 0; slot < m_simulatableSlots; ++slot)
	{
		const bool occupied = occupancy.occupied.test(slot);
		if (!forceEverySlot && occupied == m_appliedOccupancy.occupied.test(slot))
		{
			continue;
		}
		for (uint32_t templateIndex = 0; templateIndex < m_template.size(); ++templateIndex)
		{
			const JPH::BodyID id = slotBodyId(slot, templateIndex);
			const JPH::ObjectLayer layer = occupied ? m_template[templateIndex].layer : m_layers.parkedLayer();
			if (bodyInterface.GetObjectLayer(id) != layer)
			{
				bodyInterface.SetObjectLayer(id, layer);
			}
			bodyInterface.SetGravityFactor(id, occupied ? m_template[templateIndex].gravityFactor : 0.f);
			if (!occupied)
			{
				bodyInterface.SetLinearAndAngularVelocity(id, JPH::Vec3::sZero(), JPH::Vec3::sZero());
			}
		}
	}
	m_appliedOccupancy = occupancy;
}

uint64_t JoltWorld::liveStateHash() const
{
	const JPH::BodyInterface& bodyInterface = bodies();
	StateHash hash;
	for (uint32_t slot = 0; slot < m_simulatableSlots; ++slot)
	{
		if (!m_appliedOccupancy.occupied.test(slot))
		{
			continue;
		}
		for (uint32_t templateIndex = 0; templateIndex < m_template.size(); ++templateIndex)
		{
			const JPH::BodyID id = slotBodyId(slot, templateIndex);
			const JPH::RVec3 position = bodyInterface.GetCenterOfMassPosition(id);
			const JPH::Quat rotation = bodyInterface.GetRotation(id);
			const JPH::Vec3 linear = bodyInterface.GetLinearVelocity(id);
			const JPH::Vec3 angular = bodyInterface.GetAngularVelocity(id);
			hash.u32(id.GetIndexAndSequenceNumber());
			hash.f32(position.GetX());
			hash.f32(position.GetY());
			hash.f32(position.GetZ());
			hash.f32(rotation.GetX());
			hash.f32(rotation.GetY());
			hash.f32(rotation.GetZ());
			hash.f32(rotation.GetW());
			hash.f32(linear.GetX());
			hash.f32(linear.GetY());
			hash.f32(linear.GetZ());
			hash.f32(angular.GetX());
			hash.f32(angular.GetY());
			hash.f32(angular.GetZ());
		}
	}
	hash.u32(static_cast<uint32_t>(m_appliedOccupancy.occupied.to_ulong()));
	return hash.value();
}

void JoltWorld::log(const char* message) const
{
	if (m_logger)
	{
		m_logger(message);
	}
}
