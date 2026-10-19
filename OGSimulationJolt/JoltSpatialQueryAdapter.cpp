// SPDX-License-Identifier: MPL-2.0
// docs/JoltSpatialQueryAdapter-rationale.md · docs/JoltSpatialQueryAdapter-guards.md

#include "OGSimulationJolt/JoltSpatialQueryAdapter.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <numeric>
#include <tuple>
#include <variant>

#include <Jolt/Physics/Body/Body.h>
#include <Jolt/Physics/Body/BodyFilter.h>
#include <Jolt/Physics/Body/BodyLock.h>
#include <Jolt/Physics/Body/BodyLockInterface.h>
#include <Jolt/Physics/Collision/CollideShape.h>
#include <Jolt/Physics/Collision/CollisionCollectorImpl.h>
#include <Jolt/Physics/Collision/NarrowPhaseQuery.h>
#include <Jolt/Physics/Collision/ShapeCast.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/CapsuleShape.h>
#include <Jolt/Physics/Collision/Shape/CompoundShape.h>
#include <Jolt/Physics/Collision/Shape/RotatedTranslatedShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>

#include "OGSimulation/OGAssert.h"
#include "OGSimulationJolt/JoltPhysicsBodyAdapter.h"
#include "OGSimulationJolt/JoltPhysicsFactory.h"
#include "OGSimulationJolt/JoltUnits.h"

namespace
{
	thread_local uint32_t t_worldAccessScopeDepth = 0;

	constexpr float kCentimetresPerMetre = 100.f;
	constexpr float kNearlyZeroSweepLengthCm = 1.e-8f;

	glm::vec3 translationOf(const glm::mat4& transform)
	{
		return glm::vec3(transform[3]);
	}

	glm::vec3 toGlm(JPH::Vec3Arg value)
	{
		return glm::vec3(value.GetX(), value.GetY(), value.GetZ());
	}

	glm::vec3 toCentimetres(JPH::Vec3Arg metres)
	{
		return toGlm(metres) * kCentimetresPerMetre;
	}

	JPH::RefConst<JPH::Shape> queryShapeOf(const QueryGeometry& geometry)
	{
		if (const SphereGeometry* sphere = std::get_if<SphereGeometry>(&geometry))
		{
			OG_CHECK(sphere->radius > 0.f, "JoltSpatialQueryAdapter::registerVolume - a SphereGeometry volume needs a positive radius");
			return new JPH::SphereShape(joltUnits::centimetresToMetres(std::max(sphere->radius, 0.001f)));
		}
		if (const BoxGeometry* box = std::get_if<BoxGeometry>(&geometry))
		{
			return new JPH::BoxShape(joltUnits::centimetresToMetres(box->halfExtents), 0.f);
		}
		const CapsuleGeometry& capsule = std::get<CapsuleGeometry>(geometry);
		OG_CHECK(capsule.radius > 0.f, "JoltSpatialQueryAdapter::registerVolume - a CapsuleGeometry volume needs a positive radius");
		const float radius = joltUnits::centimetresToMetres(std::max(capsule.radius, 0.001f));
		const float cylinderHalfHeight = joltUnits::centimetresToMetres(capsule.halfHeight - capsule.radius);
		if (cylinderHalfHeight <= 0.f)
		{
			return new JPH::SphereShape(radius);
		}
		const JPH::Quat yAxisToZAxis = JPH::Quat::sRotation(JPH::Vec3::sAxisX(), 0.5f * JPH::JPH_PI);
		return new JPH::RotatedTranslatedShape(JPH::Vec3::sZero(), yAxisToZAxis, new JPH::CapsuleShape(cylinderHalfHeight, radius));
	}

	JPH::RMat44 queryPoseOf(const JPH::Shape& shape, const glm::vec3& positionCm)
	{
		// ⛔G-02  docs/JoltSpatialQueryAdapter-guards.md
		return JPH::RMat44::sTranslation(joltUnits::centimetresToMetres(positionCm) + shape.GetCenterOfMass());
	}

	class JoltTraceQueryLayerFilter final : public JPH::ObjectLayerFilter
	{
	public:
		JoltTraceQueryLayerFilter(const JoltLayerTable& table, uint32_t traceCategory, bool requireBlock)
			: m_table(table)
			, m_traceCategory(traceCategory)
			, m_requireBlock(requireBlock)
		{
		}

		bool ShouldCollide(JPH::ObjectLayer layer) const override
		{
			const JoltLayerKey& key = m_table.keyOf(layer);
			return m_requireBlock ? joltQueryRule::blocksTraceCategory(key, m_traceCategory)
				: joltQueryRule::respondsToTraceCategory(key, m_traceCategory);
		}

	private:
		const JoltLayerTable& m_table;
		uint32_t m_traceCategory = 0;
		bool m_requireBlock = false;
	};

	class QueryLayerFilters
	{
	public:
		QueryLayerFilters(const JoltLayerTable& table, uint32_t objectMask, uint32_t traceCategory, bool requireBlock)
			: m_object(table, objectMask)
			, m_trace(table, traceCategory, requireBlock)
			, m_isObjectQuery(objectMask != 0u)
		{
		}

		const JPH::ObjectLayerFilter& selected() const
		{
			// ⛔G-05  docs/JoltSpatialQueryAdapter-guards.md
			return m_isObjectQuery ? static_cast<const JPH::ObjectLayerFilter&>(m_object) : static_cast<const JPH::ObjectLayerFilter&>(m_trace);
		}

	private:
		JoltQueryLayerFilter m_object;
		JoltTraceQueryLayerFilter m_trace;
		bool m_isObjectQuery = false;
	};

	auto peerStableOrderOf(const auto& candidate)
	{
		// ⛔G-06  docs/JoltSpatialQueryAdapter-guards.md
		return std::make_tuple(candidate.key, -candidate.penetrationDepth, candidate.subShapeId);
	}

	bool candidateOrder(const auto& first, const auto& second)
	{
		return peerStableOrderOf(first) < peerStableOrderOf(second);
	}

	bool nearestCandidateOrder(const auto& first, const auto& second)
	{
		return std::make_tuple(first.fraction, peerStableOrderOf(first)) < std::make_tuple(second.fraction, peerStableOrderOf(second));
	}
}

class JoltQueryBodyFilter final : public JPH::BodyFilter
{
public:
	JoltQueryBodyFilter(const JoltSpatialQueryAdapter& adapter, BodyId ignoredRootBodyId)
		: m_adapter(adapter)
		, m_ignoredRootBodyId(ignoredRootBodyId)
	{
	}

	bool ShouldCollideLocked(const JPH::Body& body) const override
	{
		const JPH::BodyID id = body.GetID();
		if (body.IsStatic())
		{
			return !std::binary_search(m_adapter.m_queryExcludedStatics.begin(), m_adapter.m_queryExcludedStatics.end(), id.GetIndex());
		}
		// ⛔G-04  docs/JoltSpatialQueryAdapter-guards.md
		if (!joltBodyUserData::decode(body.GetUserData()).has_value() || body.GetObjectLayer() == m_adapter.m_world.layers().parkedLayer())
		{
			return false;
		}
		const BodyId bodyId = JoltPhysicsBodyAdapter::bodyIdOf(id);
		const uint32_t index = id.GetIndex();
		if (index < m_adapter.m_shapeRegistered.size() && m_adapter.m_shapeRegistered[index] != 0
			&& !m_adapter.m_world.shapeEnables().isEnabled(JoltPhysicsFactory::defaultShapeIdOf(bodyId, 0)))
		{
			return false;
		}
		return m_ignoredRootBodyId.value == 0 || m_adapter.rootOf(bodyId) != m_ignoredRootBodyId;
	}

private:
	const JoltSpatialQueryAdapter& m_adapter;
	BodyId m_ignoredRootBodyId;
};

JoltWorldAccessScope::JoltWorldAccessScope()
{
	++t_worldAccessScopeDepth;
}

JoltWorldAccessScope::~JoltWorldAccessScope()
{
	--t_worldAccessScopeDepth;
}

bool JoltWorldAccessScope::isOpenOnThisThread()
{
	return t_worldAccessScopeDepth > 0;
}

JoltSpatialQueryAdapter::JoltSpatialQueryAdapter(JoltWorld& world, JoltWorldLoggerFn logger, JoltSpatialQueryConfig config)
	: m_world(world)
	, m_logger(std::move(logger))
	, m_mappedCategories(config.mappedCategories)
	, m_accessCheck(config.accessCheck != nullptr ? config.accessCheck : &JoltWorldAccessScope::isOpenOnThisThread)
{
	OG_CHECK(m_mappedCategories != 0u,
		"JoltSpatialQueryAdapter: JoltSpatialQueryConfig::mappedCategories is empty; pass the host's category table (the categories the Chaos adapter maps)");
	const uint32_t bodyIndexCount = 1u + world.simulatableSlots() * world.bodiesPerSlot();
	m_rootIndexOf.resize(bodyIndexCount);
	std::iota(m_rootIndexOf.begin(), m_rootIndexOf.end(), 0u);
	m_shapeRegistered.assign(bodyIndexCount, 0u);
}

bool JoltSpatialQueryAdapter::callerMayAccessWorld() const
{
	return m_accessCheck();
}

QueryVolumeId JoltSpatialQueryAdapter::registerVolume(const QueryVolumeDescriptor& descriptor, BodyId ignoredRootBodyId)
{
	OG_CHECK(callerMayAccessWorld(), "JoltSpatialQueryAdapter::registerVolume - not on the step thread and not under the world mutex");
	const QueryVolumeId volumeId{ static_cast<uint32_t>(m_volumes.size()) };
	diagnoseVolumeCategories(descriptor, volumeId);
	m_volumes.push_back(VolumeEntry{
		queryShapeOf(descriptor.geometry),
		joltQueryRule::effectiveObjectMask(descriptor.searchCategories.bits, m_mappedCategories),
		descriptor.traceCategory,
		ignoredRootBodyId,
		glm::mat4(1.f),
		descriptor.offsetTransform });
	return volumeId;
}

ShapeId JoltSpatialQueryAdapter::registerShape(BodyId body, uint32_t shapeIndex, std::optional<BodyId> rootBodyId)
{
	OG_CHECK(callerMayAccessWorld(), "JoltSpatialQueryAdapter::registerShape - not on the step thread and not under the world mutex");
	const uint32_t index = JPH::BodyID(body.value).GetIndex();
	OG_CHECK(index >= 1u && index < m_rootIndexOf.size(), "JoltSpatialQueryAdapter::registerShape - the body is not a slot body of this world");
	if (index < 1u || index >= m_rootIndexOf.size())
	{
		return ShapeId{};
	}

	uint32_t rootIndex = rootBodyId.has_value() ? JPH::BodyID(rootBodyId->value).GetIndex() : index;
	OG_CHECK(rootIndex >= 1u && rootIndex < m_rootIndexOf.size(), "JoltSpatialQueryAdapter::registerShape - the root body is not a slot body of this world");
	for (uint32_t hops = 0; rootIndex < m_rootIndexOf.size() && m_rootIndexOf[rootIndex] != rootIndex && hops < m_rootIndexOf.size(); ++hops)
	{
		rootIndex = m_rootIndexOf[rootIndex];
	}
	m_rootIndexOf[index] = rootIndex < m_rootIndexOf.size() ? rootIndex : index;
	m_shapeRegistered[index] = 1u;

	const ShapeId shapeId = JoltPhysicsFactory::defaultShapeIdOf(body, shapeIndex);
	// ⛔G-03  docs/JoltSpatialQueryAdapter-guards.md
	m_world.shapeEnables().enable(shapeId);
	return shapeId;
}

void JoltSpatialQueryAdapter::excludeStaticBodiesFromQueries(const std::vector<JPH::BodyID>& staticBodies)
{
	OG_CHECK(callerMayAccessWorld(), "JoltSpatialQueryAdapter::excludeStaticBodiesFromQueries - not on the step thread and not under the world mutex");
	const JPH::BodyLockInterface& locks = m_world.physics().GetBodyLockInterfaceNoLock();
	for (const JPH::BodyID& id : staticBodies)
	{
		const JPH::Body* body = locks.TryGetBody(id);
		OG_CHECK(body != nullptr && body->IsStatic(),
			"JoltSpatialQueryAdapter::excludeStaticBodiesFromQueries - only static bodies (PhysicsOnly statics) can be excluded from queries");
		if (body != nullptr && body->IsStatic())
		{
			m_queryExcludedStatics.push_back(id.GetIndex());
		}
	}
	std::sort(m_queryExcludedStatics.begin(), m_queryExcludedStatics.end());
	m_queryExcludedStatics.erase(std::unique(m_queryExcludedStatics.begin(), m_queryExcludedStatics.end()), m_queryExcludedStatics.end());
}

SpatialQueryReport JoltSpatialQueryAdapter::overlap(const std::vector<QueryVolumeId>& volumeIds)
{
	OG_CHECK(callerMayAccessWorld(), "JoltSpatialQueryAdapter::overlap - not on the step thread and not under the world mutex");
	SpatialQueryReport report;
	for (const QueryVolumeId& volumeId : volumeIds)
	{
		OG_CHECK(volumeOf(volumeId) != nullptr, "JoltSpatialQueryAdapter::overlap - an unregistered QueryVolumeId");
	}
	if (volumeIds.empty())
	{
		return report;
	}

	// ⛔G-01  docs/JoltSpatialQueryAdapter-guards.md
	const VolumeEntry* volume = volumeOf(volumeIds.back());
	if (volume == nullptr)
	{
		return report;
	}
	const std::vector<Candidate> candidates =
		collideAt(*volume, translationOf(volume->parentTransform * volume->offsetTransform), CandidatePurpose::Overlap);
	report.hits.reserve(candidates.size());
	for (const Candidate& candidate : candidates)
	{
		report.hits.push_back(hitOf(candidate));
	}
	return report;
}

SweepHit JoltSpatialQueryAdapter::sweep(QueryVolumeId volumeId, const glm::mat4& transform, const glm::vec3& delta)
{
	OG_CHECK(callerMayAccessWorld(), "JoltSpatialQueryAdapter::sweep - not on the step thread and not under the world mutex");
	const VolumeEntry* volume = volumeOf(volumeId);
	OG_CHECK(volume != nullptr, "JoltSpatialQueryAdapter::sweep - an unregistered QueryVolumeId");
	if (volume == nullptr)
	{
		return SweepHit{};
	}

	const glm::vec3 start = translationOf(transform * volume->offsetTransform);
	const float length = std::sqrt(delta.x * delta.x + delta.y * delta.y + delta.z * delta.z);
	const std::vector<Candidate> candidates = length <= kNearlyZeroSweepLengthCm
		? collideAt(*volume, start, CandidatePurpose::Sweep)
		: castFrom(*volume, start, delta);
	if (candidates.empty())
	{
		return SweepHit{};
	}

	const Candidate& nearest = *std::min_element(candidates.begin(), candidates.end(),
		[](const Candidate& first, const Candidate& second) { return nearestCandidateOrder(first, second); });

	SweepHit hit;
	hit.blocked = true;
	hit.fraction = std::clamp(nearest.fraction, 0.f, 1.f);
	hit.startPenetrating = nearest.fraction <= 0.f;
	hit.penetrationDepth = hit.startPenetrating ? std::max(nearest.penetrationDepth, 0.f) * kCentimetresPerMetre : 0.f;
	const float axisLengthSquared = nearest.penetrationAxis.LengthSq();
	hit.normal = axisLengthSquared > 1.e-12f ? toGlm(-nearest.penetrationAxis / std::sqrt(axisLengthSquared)) : glm::vec3(0.f, 0.f, 1.f);
	hit.impactPoint = toCentimetres(nearest.contactPointOnBody);
	hit.bodyId = JoltPhysicsBodyAdapter::bodyIdOf(nearest.body);
	hit.rootBodyId = rootOf(hit.bodyId);
	hit.objectCategories = categoriesOf(nearest.body);
	return hit;
}

void JoltSpatialQueryAdapter::setVolumeParentTransform(QueryVolumeId volumeId, const glm::mat4& transform)
{
	OG_CHECK(callerMayAccessWorld(), "JoltSpatialQueryAdapter::setVolumeParentTransform - not on the step thread and not under the world mutex");
	OG_CHECK(volumeOf(volumeId) != nullptr, "JoltSpatialQueryAdapter::setVolumeParentTransform - an unregistered QueryVolumeId");
	if (volumeId.value < m_volumes.size())
	{
		m_volumes[volumeId.value].parentTransform = transform;
	}
}

void JoltSpatialQueryAdapter::enableShape(ShapeId shapeId)
{
	OG_CHECK(callerMayAccessWorld(), "JoltSpatialQueryAdapter::enableShape - not on the step thread and not under the world mutex");
	m_world.shapeEnables().enable(shapeId);
}

void JoltSpatialQueryAdapter::disableShape(ShapeId shapeId)
{
	OG_CHECK(callerMayAccessWorld(), "JoltSpatialQueryAdapter::disableShape - not on the step thread and not under the world mutex");
	m_world.shapeEnables().disable(shapeId);
}

std::optional<JoltQueryHitKey> JoltSpatialQueryAdapter::hitKeyOf(BodyId bodyId, uint32_t staticElementIndex) const
{
	OG_CHECK(callerMayAccessWorld(), "JoltSpatialQueryAdapter::hitKeyOf - not on the step thread and not under the world mutex");
	std::optional<JoltQueryHitKey> key = keyOf(JPH::BodyID(bodyId.value), 0u);
	if (key.has_value() && key->isStatic != 0)
	{
		key->staticElementIndex = staticElementIndex;
	}
	return key;
}

const JoltSpatialQueryAdapter::VolumeEntry* JoltSpatialQueryAdapter::volumeOf(QueryVolumeId volumeId) const
{
	return volumeId.value < m_volumes.size() ? &m_volumes[volumeId.value] : nullptr;
}

std::vector<JoltSpatialQueryAdapter::Candidate> JoltSpatialQueryAdapter::collideAt(const VolumeEntry& volume, const glm::vec3& positionCm,
	CandidatePurpose purpose) const
{
	JPH::CollideShapeSettings settings;
	settings.mBackFaceMode = JPH::EBackFaceMode::CollideWithBackFaces;
	JPH::AllHitCollisionCollector<JPH::CollideShapeCollector> collector;
	const JoltQueryBodyFilter bodyFilter(*this, volume.ignoredRootBodyId);
	const JPH::RMat44 pose = queryPoseOf(*volume.shape, positionCm);

	const QueryLayerFilters layerFilters(m_world.layers(), volume.objectMask, volume.traceCategory, purpose == CandidatePurpose::Sweep);
	m_world.query().CollideShape(volume.shape, JPH::Vec3::sOne(), pose, settings, JPH::RVec3::sZero(), collector, {}, layerFilters.selected(), bodyFilter);

	std::vector<Candidate> candidates;
	candidates.reserve(collector.mHits.size());
	for (const JPH::CollideShapeResult& result : collector.mHits)
	{
		const std::optional<JoltQueryHitKey> key = keyOf(result.mBodyID2, result.mSubShapeID2.GetValue());
		if (!key.has_value())
		{
			continue;
		}
		candidates.push_back(Candidate{ *key, result.mBodyID2, 0.f, result.mPenetrationDepth, result.mPenetrationAxis,
			result.mContactPointOn2, result.mSubShapeID2.GetValue() });
	}

	std::sort(candidates.begin(), candidates.end(), [](const Candidate& first, const Candidate& second) { return candidateOrder(first, second); });
	candidates.erase(std::unique(candidates.begin(), candidates.end(),
		[](const Candidate& first, const Candidate& second) { return first.key == second.key; }), candidates.end());
	return candidates;
}

std::vector<JoltSpatialQueryAdapter::Candidate> JoltSpatialQueryAdapter::castFrom(const VolumeEntry& volume, const glm::vec3& startCm,
	const glm::vec3& deltaCm) const
{
	const JPH::RShapeCast cast(volume.shape, JPH::Vec3::sOne(), queryPoseOf(*volume.shape, startCm), joltUnits::centimetresToMetres(deltaCm));
	JPH::ShapeCastSettings settings;
	// ⛔G-07  docs/JoltSpatialQueryAdapter-guards.md
	settings.SetBackFaceMode(JPH::EBackFaceMode::CollideWithBackFaces);
	settings.mReturnDeepestPoint = true;
	JPH::AllHitCollisionCollector<JPH::CastShapeCollector> collector;
	const JoltQueryBodyFilter bodyFilter(*this, volume.ignoredRootBodyId);

	const QueryLayerFilters layerFilters(m_world.layers(), volume.objectMask, volume.traceCategory, true);
	m_world.query().CastShape(cast, settings, JPH::RVec3::sZero(), collector, {}, layerFilters.selected(), bodyFilter);

	std::vector<Candidate> candidates;
	candidates.reserve(collector.mHits.size());
	for (const JPH::ShapeCastResult& result : collector.mHits)
	{
		const std::optional<JoltQueryHitKey> key = keyOf(result.mBodyID2, result.mSubShapeID2.GetValue());
		if (!key.has_value())
		{
			continue;
		}
		candidates.push_back(Candidate{ *key, result.mBodyID2, result.mFraction, result.mPenetrationDepth, result.mPenetrationAxis,
			result.mContactPointOn2, result.mSubShapeID2.GetValue() });
	}
	return candidates;
}

std::optional<JoltQueryHitKey> JoltSpatialQueryAdapter::keyOf(const JPH::BodyID& id, uint32_t subShapeId) const
{
	JPH::BodyLockRead lock(m_world.physics().GetBodyLockInterfaceNoLock(), id);
	if (!lock.Succeeded())
	{
		return std::nullopt;
	}
	const JPH::Body& body = lock.GetBody();
	if (body.IsStatic())
	{
		JoltQueryHitKey key;
		key.isStatic = 1u;
		key.staticBodyIndex = id.GetIndex();
		const JPH::Shape* shape = body.GetShape();
		if (shape->GetType() == JPH::EShapeType::Compound)
		{
			JPH::SubShapeID subShape;
			subShape.SetValue(subShapeId);
			JPH::SubShapeID remainder;
			key.staticElementIndex = static_cast<const JPH::CompoundShape*>(shape)->GetSubShapeIndexFromID(subShape, remainder);
		}
		return key;
	}
	const std::optional<JoltBodyBinding> binding = joltBodyUserData::decode(body.GetUserData());
	if (!binding.has_value())
	{
		return std::nullopt;
	}
	JoltQueryHitKey key;
	key.simulatableId = binding->simulatableId;
	key.declarationIndex = binding->declarationIndex;
	return key;
}

BodyId JoltSpatialQueryAdapter::rootOf(BodyId body) const
{
	const uint32_t index = JPH::BodyID(body.value).GetIndex();
	if (index >= m_rootIndexOf.size())
	{
		return body;
	}
	return JoltPhysicsBodyAdapter::bodyIdOf(JPH::BodyID(m_rootIndexOf[index]));
}

CollisionCategories JoltSpatialQueryAdapter::categoriesOf(const JPH::BodyID& id) const
{
	return CollisionCategories{ m_world.layers().keyOf(m_world.bodies().GetObjectLayer(id)).categories };
}

SpatialQueryHit JoltSpatialQueryAdapter::hitOf(const Candidate& candidate) const
{
	SpatialQueryHit hit;
	hit.bodyId = JoltPhysicsBodyAdapter::bodyIdOf(candidate.body);
	hit.rootBodyId = rootOf(hit.bodyId);
	hit.objectCategories = categoriesOf(candidate.body);
	if (candidate.key.isStatic == 0)
	{
		// ⛔G-08  docs/JoltSpatialQueryAdapter-guards.md
		hit.objectPosition = toCentimetres(m_world.bodies().GetPosition(JoltPhysicsBodyAdapter::joltBodyIdOf(hit.rootBodyId)));
		return hit;
	}

	JPH::BodyLockRead lock(m_world.physics().GetBodyLockInterfaceNoLock(), candidate.body);
	if (!lock.Succeeded())
	{
		return hit;
	}
	const JPH::Body& body = lock.GetBody();
	const JPH::Shape* shape = body.GetShape();
	if (shape->GetType() == JPH::EShapeType::Compound)
	{
		const JPH::CompoundShape* compound = static_cast<const JPH::CompoundShape*>(shape);
		const JPH::CompoundShape::SubShape& element = compound->GetSubShape(candidate.key.staticElementIndex);
		hit.objectPosition = toCentimetres(JPH::Vec3(body.GetCenterOfMassTransform() * element.GetPositionCOM()));
	}
	else
	{
		hit.objectPosition = toCentimetres(JPH::Vec3(body.GetCenterOfMassPosition()));
	}
	return hit;
}

void JoltSpatialQueryAdapter::diagnoseVolumeCategories(const QueryVolumeDescriptor& descriptor, QueryVolumeId volumeId) const
{
	const uint32_t requested = descriptor.searchCategories.bits;
	const uint32_t unmapped = joltQueryRule::unmappedSearchMask(requested, m_mappedCategories);
	char line[512];
	if (joltQueryRule::isSilentlyEmptyObjectQuery(requested, m_mappedCategories))
	{
		std::snprintf(line, sizeof(line),
			"[Error] JoltSpatialQueryAdapter: [SpatialQuery.EmptyObjectQuery] volume=%u searchCategories=0x%08X mapped=0x%08X -- NOT ONE "
			"requested category is mapped, so this volume runs as a TRACE-CHANNEL query on traceCategory=%u (what the Chaos adapter's "
			"empty object-type set does too). Map the categories in the adapter's configuration or stop searching them.",
			volumeId.value, requested, m_mappedCategories, descriptor.traceCategory);
		log(line);
	}
	else if (unmapped != 0u)
	{
		std::snprintf(line, sizeof(line),
			"[Error] JoltSpatialQueryAdapter: [SpatialQuery.PartialObjectQuery] volume=%u searchCategories=0x%08X unmapped=0x%08X "
			"mapped=0x%08X -- the unmapped bits are SILENTLY DROPPED from this volume's object query. It will find the rest and miss "
			"those, and a partial result reads as a healthy one.",
			volumeId.value, requested, unmapped, m_mappedCategories);
		log(line);
	}

	if ((joltQueryRule::categoryBit(descriptor.traceCategory) & m_mappedCategories) == 0u)
	{
		std::snprintf(line, sizeof(line),
			"[Error] JoltSpatialQueryAdapter: [SpatialQuery.UnmappedTraceCategory] volume=%u traceCategory=%u mapped=0x%08X -- the "
			"trace category is not mapped. A trace-channel query on this volume answers on that category's own bit; the Chaos adapter "
			"traced on ECollisionChannel(0) (WorldStatic) instead.",
			volumeId.value, descriptor.traceCategory, m_mappedCategories);
		log(line);
	}
}

void JoltSpatialQueryAdapter::log(const char* message) const
{
	if (m_logger)
	{
		m_logger(message);
	}
}
