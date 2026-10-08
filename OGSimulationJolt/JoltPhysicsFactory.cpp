// SPDX-License-Identifier: MPL-2.0
// docs/JoltPhysicsFactory-rationale.md · docs/JoltPhysicsFactory-guards.md

#include "OGSimulationJolt/JoltPhysicsFactory.h"

#include <utility>
#include <variant>

#include <Jolt/Physics/Body/Body.h>
#include <Jolt/Physics/Body/BodyLock.h>
#include <Jolt/Physics/Body/BodyLockInterface.h>
#include <Jolt/Physics/Body/MassProperties.h>
#include <Jolt/Physics/Body/MotionProperties.h>

#include "OGSimulation/OGAssert.h"

namespace
{
	JPH::EAllowedDOFs allowedDofsOf(const BodyDescriptor& body)
	{
		return body.lockRotation ? JPH::EAllowedDOFs::TranslationX | JPH::EAllowedDOFs::TranslationY | JPH::EAllowedDOFs::TranslationZ
			: JPH::EAllowedDOFs::All;
	}

	bool sameGeometry(const QueryGeometry& first, const QueryGeometry& second)
	{
		if (first.index() != second.index())
		{
			return false;
		}
		if (const SphereGeometry* sphere = std::get_if<SphereGeometry>(&first))
		{
			return sphere->radius == std::get<SphereGeometry>(second).radius;
		}
		if (const CapsuleGeometry* capsule = std::get_if<CapsuleGeometry>(&first))
		{
			const CapsuleGeometry& other = std::get<CapsuleGeometry>(second);
			return capsule->radius == other.radius && capsule->halfHeight == other.halfHeight;
		}
		return std::get<BoxGeometry>(first).halfExtents == std::get<BoxGeometry>(second).halfExtents;
	}
}

JoltPhysicsFactory::JoltPhysicsFactory(JoltPhysicsBodyAdapter& bodyAdapter,
	std::vector<SlotBodyTemplate> slotTemplate,
	uint32_t slot,
	uint32_t simulatableId,
	JoltPhysicsFactoryOptions options,
	JoltShapeRegistrarFn shapeRegistrar)
	: m_bodyAdapter(bodyAdapter)
	, m_template(std::move(slotTemplate))
	, m_slot(slot)
	, m_simulatableId(simulatableId)
	, m_options(options)
	, m_shapeRegistrar(std::move(shapeRegistrar))
{
	JoltWorld& world = m_bodyAdapter.world();
	OG_CHECK(m_template.size() == world.bodiesPerSlot(),
		"JoltPhysicsFactory: the slot template is not the one the world was built from (body count differs)");
	OG_CHECK(m_slot < world.simulatableSlots(), "JoltPhysicsFactory: slot out of range");

	uint32_t rootCount = 0;
	for (uint32_t templateIndex = 0; templateIndex < m_template.size() && m_slot < world.simulatableSlots(); ++templateIndex)
	{
		if (m_template[templateIndex].descriptor.body.isRoot)
		{
			++rootCount;
			m_parentBodyId = JoltPhysicsBodyAdapter::bodyIdOf(world.slotBodyId(m_slot, templateIndex));
		}
	}
	OG_CHECK(rootCount == 1, "JoltPhysicsFactory: the slot template must hold exactly one isRoot body (the simulatable's root, its parentBodyId)");

	world.physics().SetCombineFriction(&joltBodyDefaults::combineFrictionAverage);
	world.physics().SetCombineRestitution(&joltBodyDefaults::combineRestitutionAverage);
}

JoltPhysicsFactory::PhysicalObjectResult JoltPhysicsFactory::createPhysicalObject(const PhysicalObjectDescriptor& descriptor, const char* /*name*/)
{
	OG_CHECK(m_nextTemplateIndex < m_template.size(),
		"JoltPhysicsFactory::createPhysicalObject - more declarations than the slot template has bodies");
	if (m_nextTemplateIndex >= m_template.size() || m_slot >= m_bodyAdapter.world().simulatableSlots())
	{
		return PhysicalObjectResult{};
	}

	const uint32_t templateIndex = m_nextTemplateIndex++;
	const SlotBodyTemplate& slotTemplate = m_template[templateIndex];
	OG_CHECK(sameDescriptor(descriptor, slotTemplate.descriptor),
		"JoltPhysicsFactory::createPhysicalObject - the declaration differs from the next slot-template body; the factory binds in template order, which must be the physics composite's declaration order");
	OG_CHECK(descriptor.shapes.size() == 1, "JoltPhysicsFactory::createPhysicalObject - M1 slot bodies have exactly one shape");

	const JPH::BodyID joltId = m_bodyAdapter.world().slotBodyId(m_slot, templateIndex);
	const BodyId bodyId = JoltPhysicsBodyAdapter::bodyIdOf(joltId);
	const joltBodyDefaults::ChaosParityMassProperties massProperties =
		joltBodyDefaults::chaosParityMassPropertiesOf(slotTemplate.descriptor.shapes.front().geometry);

	applyBodyDefaults(joltId, slotTemplate, massProperties);
	m_bodyAdapter.bindTable().bind(bodyId, JoltBodyBinding{ m_simulatableId, slotTemplate.declarationIndex },
		massProperties.inertiaKilogramSquareCentimetres);

	const bool isRoot = slotTemplate.descriptor.body.isRoot;
	PhysicalObjectResult result{ bodyId, {} };
	for (uint32_t shapeIndex = 0; shapeIndex < descriptor.shapes.size(); ++shapeIndex)
	{
		const std::optional<BodyId> rootBodyId = isRoot ? std::nullopt : std::optional<BodyId>(m_parentBodyId);
		result.shapeIds.push_back(m_shapeRegistrar ? m_shapeRegistrar(bodyId, shapeIndex, rootBodyId) : defaultShapeIdOf(bodyId, shapeIndex));
	}
	return result;
}

void JoltPhysicsFactory::applyBodyDefaults(const JPH::BodyID& id, const SlotBodyTemplate& slotTemplate,
	const joltBodyDefaults::ChaosParityMassProperties& massProperties)
{
	JoltWorld& world = m_bodyAdapter.world();
	const BodyDescriptor& descriptor = slotTemplate.descriptor.body;
	const JPH::EAllowedDOFs allowedDofs = allowedDofsOf(descriptor);
	{
		JPH::BodyLockWrite lock(world.physics().GetBodyLockInterfaceNoLock(), id);
		OG_CHECK(lock.Succeeded(), "JoltPhysicsFactory: a slot body does not exist");
		if (!lock.Succeeded())
		{
			return;
		}
		JPH::Body& body = lock.GetBody();
		OG_CHECK(!body.IsStatic(), "JoltPhysicsFactory: a slot body is static");
		if (body.IsStatic())
		{
			return;
		}

		// ⛔G-01  docs/JoltPhysicsFactory-guards.md
		const joltBodyDefaults::JoltBodyMaterial material =
			descriptor.isRoot ? joltBodyDefaults::kAdoptedRootMaterial : joltBodyDefaults::kCreatedBodyMaterial;
		body.SetFriction(material.friction);
		body.SetRestitution(material.restitution);
		body.SetUserData(joltBodyUserData::encode(JoltBodyBinding{ m_simulatableId, slotTemplate.declarationIndex }));

		// ⛔G-02  docs/JoltPhysicsFactory-guards.md
		JPH::MotionProperties& motion = *body.GetMotionProperties();
		OG_CHECK(motion.GetAllowedDOFs() == allowedDofs,
			"JoltPhysicsFactory: the slot body's allowed DOFs differ from its descriptor's lockRotation (the world and the factory disagree)");
		OG_CHECK(!body.GetAllowSleeping(), "JoltPhysicsFactory: a slot body may sleep; every slot body must have mAllowSleeping = false");
		motion.SetLinearDamping(joltBodyDefaults::kLinearDamping);
		motion.SetAngularDamping(joltBodyDefaults::kAngularDamping);
		motion.SetMaxAngularVelocity(joltBodyDefaults::kMaxAngularVelocityRadiansPerSecond);
		motion.SetMaxLinearVelocity(joltBodyDefaults::kMaxLinearVelocityMetresPerSecond);

		JPH::MassProperties mass;
		mass.mMass = massProperties.massKilograms;
		mass.mInertia = JPH::Mat44::sScale(joltSeamUnits::inertiaToJolt(massProperties.inertiaKilogramSquareCentimetres));
		motion.SetMassProperties(allowedDofs, mass);
	}

	const bool linearCast = descriptor.isRoot && m_options.linearCastCharacters;
	world.bodies().SetMotionQuality(id, linearCast ? JPH::EMotionQuality::LinearCast : JPH::EMotionQuality::Discrete);
}

ShapeId JoltPhysicsFactory::defaultShapeIdOf(BodyId body, uint32_t shapeIndex)
{
	OG_CHECK(shapeIndex == 0, "JoltPhysicsFactory::defaultShapeIdOf - M1 slot bodies have exactly one shape");
	OG_CHECK(body.value < kShapeEnableCapacity, "JoltPhysicsFactory::defaultShapeIdOf - the body index exceeds the shape-enable table");
	return ShapeId{ body.value };
}

bool JoltPhysicsFactory::sameDescriptor(const PhysicalObjectDescriptor& first, const PhysicalObjectDescriptor& second)
{
	const BodyDescriptor& a = first.body;
	const BodyDescriptor& b = second.body;
	if (a.simulatePhysics != b.simulatePhysics || a.enableGravity != b.enableGravity || a.isRoot != b.isRoot
		|| a.lockRotation != b.lockRotation || a.resimPolicy != b.resimPolicy || first.shapes.size() != second.shapes.size())
	{
		return false;
	}
	for (size_t index = 0; index < first.shapes.size(); ++index)
	{
		const ShapeDescriptor& x = first.shapes[index];
		const ShapeDescriptor& y = second.shapes[index];
		if (!sameGeometry(x.geometry, y.geometry) || x.categories.bits != y.categories.bits || x.blockingCategories.bits != y.blockingCategories.bits)
		{
			return false;
		}
	}
	return true;
}
