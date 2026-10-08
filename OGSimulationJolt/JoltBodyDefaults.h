#pragma once
// SPDX-License-Identifier: MPL-2.0
// docs/JoltBodyDefaults-rationale.md

#include <algorithm>
#include <cmath>
#include <numbers>
#include <variant>

#include "OGSimulationJolt/JoltDefineChecks.h"

#include <Jolt/Physics/Body/Body.h>

#include "glm/vec3.hpp"
#include "OGSimulation/QueryGeometry.h"

namespace joltBodyDefaults
{
	struct JoltBodyMaterial
	{
		float friction = 0.f;
		float restitution = 0.f;
	};

	inline constexpr JoltBodyMaterial kCreatedBodyMaterial{ 0.7f, 0.3f };
	inline constexpr JoltBodyMaterial kAdoptedRootMaterial{ 0.f, 0.f };

	inline constexpr float kLinearDamping = 0.01f;
	inline constexpr float kAngularDamping = 0.f;

	inline constexpr float kMaxAngularVelocityDegreesPerSecond = 3600.f;
	inline constexpr float kMaxAngularVelocityRadiansPerSecond =
		kMaxAngularVelocityDegreesPerSecond * (std::numbers::pi_v<float> / 180.f);
	inline constexpr float kMaxLinearVelocityMetresPerSecond = 500.f;

	inline constexpr float kDensityGramsPerCubicCentimetre = 1.f;
	inline constexpr float kDensityKilogramsPerCubicCentimetre = kDensityGramsPerCubicCentimetre / 1000.f;
	inline constexpr float kRaiseMassToPower = 0.75f;
	inline constexpr float kMassScale = 1.f;
	inline constexpr float kMinimumMassKilograms = 0.001f;

	inline float combineFrictionAverage(const JPH::Body& first, const JPH::SubShapeID&, const JPH::Body& second, const JPH::SubShapeID&)
	{
		return 0.5f * (first.GetFriction() + second.GetFriction());
	}

	inline float combineRestitutionAverage(const JPH::Body& first, const JPH::SubShapeID&, const JPH::Body& second, const JPH::SubShapeID&)
	{
		return 0.5f * (first.GetRestitution() + second.GetRestitution());
	}

	struct ChaosParityMassProperties
	{
		float massKilograms = 0.f;
		glm::vec3 inertiaKilogramSquareCentimetres{ 0.f };
	};

	inline ChaosParityMassProperties chaosParityMassPropertiesOf(const QueryGeometry& geometry)
	{
		constexpr double pi = std::numbers::pi;
		double volume = 0.0;
		double unitInertiaX = 0.0;
		double unitInertiaY = 0.0;
		double unitInertiaZ = 0.0;
		if (const SphereGeometry* sphere = std::get_if<SphereGeometry>(&geometry))
		{
			const double r = sphere->radius;
			volume = (4.0 / 3.0) * pi * r * r * r;
			unitInertiaX = unitInertiaY = unitInertiaZ = 0.4 * r * r;
		}
		else if (const CapsuleGeometry* capsule = std::get_if<CapsuleGeometry>(&geometry))
		{
			const double r = capsule->radius;
			const double h = 2.0 * std::max(double(capsule->halfHeight) - r, 0.0);
			volume = pi * r * r * (h + (4.0 / 3.0) * r);
			unitInertiaX = unitInertiaY = (5.0 * h * h * h + 20.0 * h * h * r + 45.0 * h * r * r + 32.0 * r * r * r) / (60.0 * h + 80.0 * r);
			unitInertiaZ = (r * r * (15.0 * h + 16.0 * r)) / (30.0 * h + 40.0 * r);
		}
		else
		{
			const BoxGeometry& box = std::get<BoxGeometry>(geometry);
			const double x = 2.0 * box.halfExtents.x;
			const double y = 2.0 * box.halfExtents.y;
			const double z = 2.0 * box.halfExtents.z;
			volume = x * y * z;
			unitInertiaX = (y * y + z * z) / 12.0;
			unitInertiaY = (x * x + z * z) / 12.0;
			unitInertiaZ = (x * x + y * y) / 12.0;
		}

		const double rawMass = double(kDensityKilogramsPerCubicCentimetre) * volume;
		const double mass = std::max(double(kMassScale) * std::pow(rawMass, double(kRaiseMassToPower)), double(kMinimumMassKilograms));
		return ChaosParityMassProperties{
			float(mass),
			glm::vec3(float(mass * unitInertiaX), float(mass * unitInertiaY), float(mass * unitInertiaZ)) };
	}
} // namespace joltBodyDefaults
