#pragma once
// SPDX-License-Identifier: MPL-2.0
// docs/JoltPhysicsBodyAdapter-rationale.md · docs/JoltPhysicsBodyAdapter-guards.md

#include <cstdint>
#include <optional>
#include <vector>

#include "OGSimulationJolt/JoltBodyUserData.h"
#include "OGSimulationJolt/JoltDefineChecks.h"
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

namespace joltSeam
{
	inline JPH::Vec3 toJolt(const glm::vec3& value) { return JPH::Vec3(value.x, value.y, value.z); }
	inline glm::vec3 toGlm(JPH::Vec3Arg value) { return glm::vec3(value.GetX(), value.GetY(), value.GetZ()); }

	inline JPH::Quat rotationToJolt(const glm::quat& rotation) { return JPH::Quat(rotation.x, rotation.y, rotation.z, rotation.w); }
	inline glm::quat rotationToSeam(JPH::QuatArg rotation) { return glm::quat(rotation.GetW(), rotation.GetX(), rotation.GetY(), rotation.GetZ()); }
} // namespace joltSeam

class JoltBodyBindTable
{
public:
	OGSIMULATIONJOLT_API JoltBodyBindTable(uint32_t simulatableSlots, uint32_t bodiesPerSlot);

	OGSIMULATIONJOLT_API void bind(BodyId bodyId, const JoltBodyBinding& binding, const glm::vec3& lockedRotationInertiaKilogramSquareMetres);
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
	OGSIMULATIONJOLT_API void setBodyPose(BodyId bodyId, const glm::vec3& position, const glm::quat& rotation);
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
static_assert(PhysicsBodyPoseWriter<JoltPhysicsBodyAdapter>, "JoltPhysicsBodyAdapter must model PhysicsBodyPoseWriter, or the restore push silently falls back to the lossy matrix path");
