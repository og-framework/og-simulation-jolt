#pragma once
// SPDX-License-Identifier: MPL-2.0
// docs/JoltPhysicsBodyReaderAdapter-rationale.md

#include "OGSimulationJolt/JoltPhysicsBodyAdapter.h"
#include "OGSimulationJolt/JoltWorld.h"
#include "OGSimulationJolt/OGJoltExport.h"

#include "glm/mat4x4.hpp"
#include "glm/vec3.hpp"
#include "OGSimulation/BodyId.h"
#include "OGSimulation/PhysicsBodyReaderAdapter.h"
#include "OGSimulation/PhysicsBodyState.h"

class JoltPhysicsBodyReaderAdapter
{
public:
	OGSIMULATIONJOLT_API JoltPhysicsBodyReaderAdapter(const JoltWorld& world, const JoltBodyBindTable& bindTable);

	[[nodiscard]] OGSIMULATIONJOLT_API glm::mat4 getBodyTransform(BodyId bodyId) const;
	[[nodiscard]] OGSIMULATIONJOLT_API glm::vec3 getBodyInertiaTensor(BodyId bodyId) const;
	[[nodiscard]] OGSIMULATIONJOLT_API PhysicsBodyState captureBodyState(BodyId bodyId) const;
	[[nodiscard]] OGSIMULATIONJOLT_API bool isBodyResolvable(BodyId bodyId) const;
	[[nodiscard]] OGSIMULATIONJOLT_API float getBodyMass(BodyId bodyId) const;

private:
	const JoltWorld& m_world;
	const JoltBodyBindTable& m_bindTable;
};

static_assert(PhysicsBodyReaderAdapter<JoltPhysicsBodyReaderAdapter>, "JoltPhysicsBodyReaderAdapter must satisfy og-simulation's PhysicsBodyReaderAdapter concept");
