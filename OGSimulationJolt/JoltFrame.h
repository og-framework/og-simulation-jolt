#pragma once
// SPDX-License-Identifier: MPL-2.0
// docs/JoltFrame-rationale.md

#include "OGSimulationJolt/JoltDefineChecks.h"

#include <Jolt/Core/Reference.h>
#include <Jolt/Physics/Collision/Shape/CapsuleShape.h>

#include "glm/vec3.hpp"
#include "OGSimulation/SimulationFrame.h"

namespace joltFrame
{
	static_assert(simFrame::kUp == glm::vec3(0.f, 1.f, 0.f),
		"JoltFrame: a Jolt capsule lies along local +Y, the og frame's up, so every capsule is a plain CapsuleShape. Was "
		"capsuleShapeFor's Ue (+Z up) branch, which rotated the capsule from Y to Z and was deleted at Stage C");

	inline JPH::RefConst<JPH::Shape> capsuleShape(float cylinderHalfHeight, float radius)
	{
		return new JPH::CapsuleShape(cylinderHalfHeight, radius);
	}

	inline JPH::Shape::ShapeResult capsuleShapeFromSettings(float cylinderHalfHeight, float radius)
	{
		return JPH::CapsuleShapeSettings(cylinderHalfHeight, radius).Create();
	}
} // namespace joltFrame
