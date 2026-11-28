// SPDX-License-Identifier: MPL-2.0
// docs/JoltPhysicsBodyReaderAdapter-rationale.md

#include "OGSimulationJolt/JoltPhysicsBodyReaderAdapter.h"

#include "OGSimulation/OGAssert.h"

JoltPhysicsBodyReaderAdapter::JoltPhysicsBodyReaderAdapter(const JoltWorld& world, const JoltBodyBindTable& bindTable)
	: m_world(world)
	, m_bindTable(bindTable)
{
}

bool JoltPhysicsBodyReaderAdapter::isBodyResolvable(BodyId bodyId) const
{
	return m_bindTable.isBound(bodyId);
}

glm::mat4 JoltPhysicsBodyReaderAdapter::getBodyTransform(BodyId bodyId) const
{
	OG_CHECK(m_world.callerMayAccess(), "JoltPhysicsBodyReaderAdapter::getBodyTransform: called off the world's owning thread, outside a step and the world lock. Was og-simulationscheduler-withjolt task 39's RW2 m4");
	return isBodyResolvable(bodyId) ? joltBodyTransformOf(m_world, bodyId) : glm::mat4(1.f);
}

glm::vec3 JoltPhysicsBodyReaderAdapter::getBodyInertiaTensor(BodyId bodyId) const
{
	OG_CHECK(m_world.callerMayAccess(), "JoltPhysicsBodyReaderAdapter::getBodyInertiaTensor: called off the world's owning thread, outside a step and the world lock. Was og-simulationscheduler-withjolt task 39's RW2 m4");
	return isBodyResolvable(bodyId) ? joltBodyInertiaOf(m_world, m_bindTable, bodyId) : glm::vec3(0.f);
}

PhysicsBodyState JoltPhysicsBodyReaderAdapter::captureBodyState(BodyId bodyId) const
{
	OG_CHECK(m_world.callerMayAccess(), "JoltPhysicsBodyReaderAdapter::captureBodyState: called off the world's owning thread, outside a step and the world lock. Was og-simulationscheduler-withjolt task 39's RW2 m4");
	return isBodyResolvable(bodyId) ? joltBodyStateOf(m_world, bodyId) : PhysicsBodyState{};
}

float JoltPhysicsBodyReaderAdapter::getBodyMass(BodyId bodyId) const
{
	OG_CHECK(m_world.callerMayAccess(), "JoltPhysicsBodyReaderAdapter::getBodyMass: called off the world's owning thread, outside a step and the world lock. Was og-simulationscheduler-withjolt task 39's RW2 m4");
	return isBodyResolvable(bodyId) ? joltBodyMassOf(m_world, bodyId) : 0.f;
}
