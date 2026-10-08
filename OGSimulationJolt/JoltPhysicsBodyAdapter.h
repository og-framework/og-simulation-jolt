#pragma once
// SPDX-License-Identifier: MPL-2.0
// docs/JoltPhysicsBodyAdapter-rationale.md · docs/JoltPhysicsBodyAdapter-guards.md

#include <cstdint>
#include <optional>
#include <vector>

#include "OGSimulationJolt/JoltDefineChecks.h"
#include "OGSimulationJolt/JoltUnits.h"
#include "OGSimulationJolt/JoltWorld.h"
#include "OGSimulationJolt/OGJoltExport.h"

#include <Jolt/Math/Quat.h>
#include <Jolt/Math/Vec3.h>
#include <Jolt/Physics/Body/BodyID.h>

#include "glm/gtc/quaternion.hpp"
#include "glm/mat4x4.hpp"
#include "glm/vec3.hpp"
#include "OGSimulation/BodyId.h"
#include "OGSimulation/PhysicsBodyAdapter.h"
#include "OGSimulation/PhysicsBodyState.h"

namespace joltSeamUnits
{
	inline constexpr float kMetresPerCentimetre = joltUnits::kMetresPerCentimetre;
	inline constexpr float kCentimetresPerMetre = 100.f;
	inline constexpr float kSquareMetresPerSquareCentimetre = kMetresPerCentimetre * kMetresPerCentimetre;
	inline constexpr float kSquareCentimetresPerSquareMetre = kCentimetresPerMetre * kCentimetresPerMetre;

	inline JPH::Vec3 toJolt(const glm::vec3& value) { return JPH::Vec3(value.x, value.y, value.z); }
	inline glm::vec3 toGlm(JPH::Vec3Arg value) { return glm::vec3(value.GetX(), value.GetY(), value.GetZ()); }

	inline JPH::Vec3 centimetreVectorToJolt(const glm::vec3& centimetres) { return joltUnits::centimetresToMetres(centimetres); }
	inline glm::vec3 metreVectorToSeam(JPH::Vec3Arg metres) { return toGlm(metres) * kCentimetresPerMetre; }

	inline JPH::Vec3 torqueToJolt(const glm::vec3& kilogramSquareCentimetresPerSecondSquared)
	{
		return toJolt(kilogramSquareCentimetresPerSecondSquared) * kSquareMetresPerSquareCentimetre;
	}

	inline JPH::Vec3 inertiaToJolt(const glm::vec3& kilogramSquareCentimetres)
	{
		return toJolt(kilogramSquareCentimetres) * kSquareMetresPerSquareCentimetre;
	}

	inline glm::vec3 inertiaToSeam(JPH::Vec3Arg kilogramSquareMetres)
	{
		return toGlm(kilogramSquareMetres) * kSquareCentimetresPerSquareMetre;
	}

	inline JPH::Quat rotationToJolt(const glm::quat& rotation) { return JPH::Quat(rotation.x, rotation.y, rotation.z, rotation.w); }
	inline glm::quat rotationToSeam(JPH::QuatArg rotation) { return glm::quat(rotation.GetW(), rotation.GetX(), rotation.GetY(), rotation.GetZ()); }
} // namespace joltSeamUnits

struct JoltBodyBinding
{
	uint32_t simulatableId = 0;
	uint8_t declarationIndex = 0;

	bool operator==(const JoltBodyBinding&) const = default;
};

namespace joltBodyUserData
{
	inline constexpr uint64_t kBoundBit = uint64_t(1) << 63;

	constexpr uint64_t encode(const JoltBodyBinding& binding)
	{
		return kBoundBit | (uint64_t(binding.simulatableId) << 8) | uint64_t(binding.declarationIndex);
	}

	constexpr std::optional<JoltBodyBinding> decode(uint64_t userData)
	{
		if ((userData & kBoundBit) == 0)
		{
			return std::nullopt;
		}
		return JoltBodyBinding{ uint32_t((userData >> 8) & 0xFFFFFFFFu), uint8_t(userData & 0xFFu) };
	}

	static_assert(decode(encode(JoltBodyBinding{ 0xFFFFFFFFu, 0xFFu })) == JoltBodyBinding{ 0xFFFFFFFFu, 0xFFu });
	static_assert(decode(encode(JoltBodyBinding{ 0u, 0u })).has_value());
	static_assert(!decode(0u).has_value());
} // namespace joltBodyUserData

class JoltBodyBindTable
{
public:
	OGSIMULATIONJOLT_API JoltBodyBindTable(uint32_t simulatableSlots, uint32_t bodiesPerSlot);

	OGSIMULATIONJOLT_API void bind(BodyId bodyId, const JoltBodyBinding& binding, const glm::vec3& lockedRotationInertiaKilogramSquareCentimetres);
	OGSIMULATIONJOLT_API void releaseSlot(uint32_t slot);

	[[nodiscard]] OGSIMULATIONJOLT_API bool isBound(BodyId bodyId) const;
	[[nodiscard]] OGSIMULATIONJOLT_API std::optional<JoltBodyBinding> bindingOf(BodyId bodyId) const;
	[[nodiscard]] OGSIMULATIONJOLT_API glm::vec3 lockedRotationInertiaOf(BodyId bodyId) const;
	[[nodiscard]] OGSIMULATIONJOLT_API std::optional<uint32_t> indexOf(BodyId bodyId) const;

	[[nodiscard]] uint32_t simulatableSlots() const { return m_simulatableSlots; }
	[[nodiscard]] uint32_t bodiesPerSlot() const { return m_bodiesPerSlot; }

private:
	uint32_t m_simulatableSlots = 0;
	uint32_t m_bodiesPerSlot = 0;
	std::vector<std::optional<JoltBodyBinding>> m_bindings;
	std::vector<glm::vec3> m_lockedRotationInertia;
};

class JoltPhysicsBodyAdapter
{
public:
	OGSIMULATIONJOLT_API explicit JoltPhysicsBodyAdapter(JoltWorld& world);

	JoltPhysicsBodyAdapter(const JoltPhysicsBodyAdapter&) = delete;
	JoltPhysicsBodyAdapter& operator=(const JoltPhysicsBodyAdapter&) = delete;

	[[nodiscard]] OGSIMULATIONJOLT_API glm::mat4 getBodyTransform(BodyId bodyId) const;
	OGSIMULATIONJOLT_API void setBodyTransform(BodyId bodyId, const glm::mat4& transform);
	OGSIMULATIONJOLT_API void addBodyTorque(BodyId bodyId, const glm::vec3& torque);
	OGSIMULATIONJOLT_API void setBodyAngularVelocity(BodyId bodyId, const glm::vec3& velocity);
	OGSIMULATIONJOLT_API void setBodyLinearVelocity(BodyId bodyId, const glm::vec3& velocity);
	OGSIMULATIONJOLT_API void addBodyAcceleration(BodyId bodyId, const glm::vec3& acceleration);
	OGSIMULATIONJOLT_API void addBodyVelocityChange(BodyId bodyId, const glm::vec3& velocityChange);
	[[nodiscard]] OGSIMULATIONJOLT_API glm::vec3 getBodyInertiaTensor(BodyId bodyId) const;
	[[nodiscard]] OGSIMULATIONJOLT_API PhysicsBodyState captureBodyState(BodyId bodyId) const;

	// ⛔G-01  docs/JoltPhysicsBodyAdapter-guards.md
	[[nodiscard]] bool isBodyResolvable(BodyId bodyId) const { return m_bindTable.isBound(bodyId); }
	[[nodiscard]] OGSIMULATIONJOLT_API float getBodyMass(BodyId bodyId) const;

	[[nodiscard]] static BodyId bodyIdOf(const JPH::BodyID& joltBodyId) { return BodyId{ joltBodyId.GetIndexAndSequenceNumber() }; }
	[[nodiscard]] static JPH::BodyID joltBodyIdOf(BodyId bodyId) { return JPH::BodyID(bodyId.value); }

	[[nodiscard]] JoltWorld& world() { return m_world; }
	[[nodiscard]] const JoltWorld& world() const { return m_world; }
	[[nodiscard]] JoltBodyBindTable& bindTable() { return m_bindTable; }
	[[nodiscard]] const JoltBodyBindTable& bindTable() const { return m_bindTable; }

private:
	[[nodiscard]] bool isSlotBody(BodyId bodyId) const;

	JoltWorld& m_world;
	JoltBodyBindTable m_bindTable;
};

[[nodiscard]] OGSIMULATIONJOLT_API glm::mat4 joltBodyTransformOf(const JoltWorld& world, BodyId bodyId);
[[nodiscard]] OGSIMULATIONJOLT_API PhysicsBodyState joltBodyStateOf(const JoltWorld& world, BodyId bodyId);
[[nodiscard]] OGSIMULATIONJOLT_API float joltBodyMassOf(const JoltWorld& world, BodyId bodyId);
[[nodiscard]] OGSIMULATIONJOLT_API glm::vec3 joltBodyInertiaOf(const JoltWorld& world, const JoltBodyBindTable& bindTable, BodyId bodyId);

static_assert(PhysicsBodyAdapter<JoltPhysicsBodyAdapter>, "JoltPhysicsBodyAdapter must satisfy og-simulation's PhysicsBodyAdapter concept");
