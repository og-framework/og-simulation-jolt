// SPDX-License-Identifier: MPL-2.0
// docs/JoltPhysicsBodyAdapter-rationale.md · docs/JoltPhysicsBodyAdapter-guards.md

#include "OGSimulationJolt/JoltPhysicsBodyAdapter.h"

#include <Jolt/Physics/Body/Body.h>
#include <Jolt/Physics/Body/BodyLock.h>
#include <Jolt/Physics/Body/BodyLockInterface.h>
#include <Jolt/Physics/Body/MotionProperties.h>

#include "glm/ext/matrix_transform.hpp"
#include "glm/gtc/quaternion.hpp"
#include "OGSimulation/OGAssert.h"

namespace
{
	constexpr uint32_t kRotationDofMask = uint32_t(JPH::EAllowedDOFs::RotationX) | uint32_t(JPH::EAllowedDOFs::RotationY) | uint32_t(JPH::EAllowedDOFs::RotationZ);

	glm::mat4 composeTransform(const glm::vec3& positionCm, const glm::quat& rotation)
	{
		return glm::translate(glm::mat4(1.f), positionCm) * glm::mat4_cast(rotation);
	}
}

JoltBodyBindTable::JoltBodyBindTable(uint32_t simulatableSlots, uint32_t bodiesPerSlot)
	: m_simulatableSlots(simulatableSlots)
	, m_bodiesPerSlot(bodiesPerSlot)
	, m_bindings(size_t(simulatableSlots) * bodiesPerSlot)
	, m_lockedRotationInertia(size_t(simulatableSlots) * bodiesPerSlot, glm::vec3(0.f))
{
}

std::optional<uint32_t> JoltBodyBindTable::indexOf(BodyId bodyId) const
{
	if (bodyId.value == 0u || bodyId.value > m_bindings.size())
	{
		return std::nullopt;
	}
	return bodyId.value - 1u;
}

void JoltBodyBindTable::bind(BodyId bodyId, const JoltBodyBinding& binding, const glm::vec3& lockedRotationInertiaKilogramSquareCentimetres)
{
	const std::optional<uint32_t> index = indexOf(bodyId);
	OG_CHECK(index.has_value(), "JoltBodyBindTable::bind - the BodyId is not a slot body of this world");
	if (!index.has_value())
	{
		return;
	}
	m_bindings[*index] = binding;
	m_lockedRotationInertia[*index] = lockedRotationInertiaKilogramSquareCentimetres;
}

void JoltBodyBindTable::releaseSlot(uint32_t slot)
{
	OG_CHECK(slot < m_simulatableSlots, "JoltBodyBindTable::releaseSlot - slot out of range");
	if (slot >= m_simulatableSlots)
	{
		return;
	}
	// ⛔G-02  docs/JoltPhysicsBodyAdapter-guards.md
	for (uint32_t templateIndex = 0; templateIndex < m_bodiesPerSlot; ++templateIndex)
	{
		m_bindings[size_t(slot) * m_bodiesPerSlot + templateIndex].reset();
	}
}

bool JoltBodyBindTable::isBound(BodyId bodyId) const
{
	const std::optional<uint32_t> index = indexOf(bodyId);
	return index.has_value() && m_bindings[*index].has_value();
}

std::optional<JoltBodyBinding> JoltBodyBindTable::bindingOf(BodyId bodyId) const
{
	const std::optional<uint32_t> index = indexOf(bodyId);
	return index.has_value() ? m_bindings[*index] : std::nullopt;
}

glm::vec3 JoltBodyBindTable::lockedRotationInertiaOf(BodyId bodyId) const
{
	const std::optional<uint32_t> index = indexOf(bodyId);
	return index.has_value() ? m_lockedRotationInertia[*index] : glm::vec3(0.f);
}

glm::mat4 joltBodyTransformOf(const JoltWorld& world, BodyId bodyId)
{
	JPH::RVec3 position;
	JPH::Quat rotation;
	world.bodies().GetPositionAndRotation(JoltPhysicsBodyAdapter::joltBodyIdOf(bodyId), position, rotation);
	return composeTransform(joltSeamUnits::metreVectorToSeam(JPH::Vec3(position)), joltSeamUnits::rotationToSeam(rotation));
}

PhysicsBodyState joltBodyStateOf(const JoltWorld& world, BodyId bodyId)
{
	const JPH::BodyID id = JoltPhysicsBodyAdapter::joltBodyIdOf(bodyId);
	const JPH::BodyInterface& bodies = world.bodies();
	JPH::RVec3 position;
	JPH::Quat rotation;
	bodies.GetPositionAndRotation(id, position, rotation);

	PhysicsBodyState state;
	state.position = joltSeamUnits::metreVectorToSeam(JPH::Vec3(position));
	state.rotation = joltSeamUnits::rotationToSeam(rotation);
	state.linearVelocity = joltSeamUnits::metreVectorToSeam(bodies.GetLinearVelocity(id));
	state.angularVelocity = joltSeamUnits::toGlm(bodies.GetAngularVelocity(id));
	return state;
}

float joltBodyMassOf(const JoltWorld& world, BodyId bodyId)
{
	JPH::BodyLockRead lock(world.physics().GetBodyLockInterfaceNoLock(), JoltPhysicsBodyAdapter::joltBodyIdOf(bodyId));
	if (!lock.Succeeded() || !lock.GetBody().IsDynamic())
	{
		return 0.f;
	}
	const float inverseMass = lock.GetBody().GetMotionProperties()->GetInverseMass();
	return inverseMass > 0.f ? 1.f / inverseMass : 0.f;
}

glm::vec3 joltBodyInertiaOf(const JoltWorld& world, const JoltBodyBindTable& bindTable, BodyId bodyId)
{
	JPH::BodyLockRead lock(world.physics().GetBodyLockInterfaceNoLock(), JoltPhysicsBodyAdapter::joltBodyIdOf(bodyId));
	if (!lock.Succeeded() || !lock.GetBody().IsDynamic())
	{
		return glm::vec3(0.f);
	}
	const JPH::Body& body = lock.GetBody();
	const uint32_t rotationDofs = uint32_t(body.GetMotionProperties()->GetAllowedDOFs()) & kRotationDofMask;
	if (rotationDofs == 0u)
	{
		return bindTable.lockedRotationInertiaOf(bodyId);
	}
	OG_CHECK(rotationDofs == kRotationDofMask,
		"joltBodyInertiaOf - a body with SOME rotation axes locked has a singular inverse inertia; M1 bodies lock all three or none");
	const JPH::Mat44 inertia = body.GetInverseInertia().Inversed3x3();
	return joltSeamUnits::inertiaToSeam(JPH::Vec3(inertia(0, 0), inertia(1, 1), inertia(2, 2)));
}

JoltPhysicsBodyAdapter::JoltPhysicsBodyAdapter(JoltWorld& world)
	: m_world(world)
	, m_bindTable(world.simulatableSlots(), world.bodiesPerSlot())
{
}

bool JoltPhysicsBodyAdapter::isSlotBody(BodyId bodyId) const
{
	const bool slotBody = m_bindTable.indexOf(bodyId).has_value();
	OG_CHECK(slotBody, "JoltPhysicsBodyAdapter - the BodyId is not a slot body of this world");
	return slotBody;
}

glm::mat4 JoltPhysicsBodyAdapter::getBodyTransform(BodyId bodyId) const
{
	if (!isSlotBody(bodyId))
	{
		return glm::mat4(1.f);
	}
	return joltBodyTransformOf(m_world, bodyId);
}

void JoltPhysicsBodyAdapter::setBodyTransform(BodyId bodyId, const glm::mat4& transform)
{
	if (!isSlotBody(bodyId))
	{
		return;
	}
	const glm::vec3 positionCm(transform[3]);
	const glm::quat rotation = glm::normalize(glm::quat_cast(glm::mat3(transform)));
	m_world.bodies().SetPositionAndRotation(joltBodyIdOf(bodyId), JPH::RVec3(joltSeamUnits::centimetreVectorToJolt(positionCm)),
		joltSeamUnits::rotationToJolt(rotation), JPH::EActivation::DontActivate);
}

void JoltPhysicsBodyAdapter::addBodyTorque(BodyId bodyId, const glm::vec3& torque)
{
	if (!isSlotBody(bodyId))
	{
		return;
	}
	m_world.bodies().AddTorque(joltBodyIdOf(bodyId), joltSeamUnits::torqueToJolt(torque));
}

void JoltPhysicsBodyAdapter::setBodyAngularVelocity(BodyId bodyId, const glm::vec3& velocity)
{
	if (!isSlotBody(bodyId))
	{
		return;
	}
	m_world.bodies().SetAngularVelocity(joltBodyIdOf(bodyId), joltSeamUnits::toJolt(velocity));
}

void JoltPhysicsBodyAdapter::setBodyLinearVelocity(BodyId bodyId, const glm::vec3& velocity)
{
	if (!isSlotBody(bodyId))
	{
		return;
	}
	m_world.bodies().SetLinearVelocity(joltBodyIdOf(bodyId), joltSeamUnits::centimetreVectorToJolt(velocity));
}

void JoltPhysicsBodyAdapter::addBodyAcceleration(BodyId bodyId, const glm::vec3& acceleration)
{
	if (!isSlotBody(bodyId))
	{
		return;
	}
	const float mass = joltBodyMassOf(m_world, bodyId);
	if (mass <= 0.f)
	{
		return;
	}
	m_world.bodies().AddForce(joltBodyIdOf(bodyId), joltSeamUnits::centimetreVectorToJolt(acceleration) * mass);
}

void JoltPhysicsBodyAdapter::addBodyVelocityChange(BodyId bodyId, const glm::vec3& velocityChange)
{
	if (!isSlotBody(bodyId))
	{
		return;
	}
	m_world.bodies().AddLinearVelocity(joltBodyIdOf(bodyId), joltSeamUnits::centimetreVectorToJolt(velocityChange));
}

glm::vec3 JoltPhysicsBodyAdapter::getBodyInertiaTensor(BodyId bodyId) const
{
	if (!isSlotBody(bodyId))
	{
		return glm::vec3(0.f);
	}
	return joltBodyInertiaOf(m_world, m_bindTable, bodyId);
}

PhysicsBodyState JoltPhysicsBodyAdapter::captureBodyState(BodyId bodyId) const
{
	if (!isSlotBody(bodyId))
	{
		return PhysicsBodyState{};
	}
	return joltBodyStateOf(m_world, bodyId);
}

float JoltPhysicsBodyAdapter::getBodyMass(BodyId bodyId) const
{
	if (!isSlotBody(bodyId))
	{
		return 0.f;
	}
	return joltBodyMassOf(m_world, bodyId);
}
