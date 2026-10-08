#pragma once
// SPDX-License-Identifier: MPL-2.0
// docs/JoltUnits-rationale.md

#include "OGSimulationJolt/JoltDefineChecks.h"

#include <Jolt/Math/Vec3.h>

#include "glm/vec3.hpp"

namespace joltUnits
{
	inline constexpr float kMetresPerCentimetre = 0.01f;

	constexpr float centimetresToMetres(float centimetres)
	{
		return centimetres * kMetresPerCentimetre;
	}

	inline JPH::Vec3 centimetresToMetres(const glm::vec3& centimetres)
	{
		return JPH::Vec3(centimetres.x, centimetres.y, centimetres.z) * kMetresPerCentimetre;
	}
} // namespace joltUnits
