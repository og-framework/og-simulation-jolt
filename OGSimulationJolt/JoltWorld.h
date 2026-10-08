#pragma once
// SPDX-License-Identifier: MPL-2.0
// docs/JoltWorld-rationale.md · docs/JoltWorld-guards.md

#include <array>
#include <cstdint>
#include <functional>
#include <optional>
#include <vector>

#include "OGSimulationJolt/JoltConfigSidecar.h"
#include "OGSimulationJolt/JoltDefineChecks.h"
#include "OGSimulationJolt/JoltLayerTable.h"
#include "OGSimulationJolt/JoltRuntime.h"
#include "OGSimulationJolt/JoltStateRing.h"
#include "OGSimulationJolt/OGJoltExport.h"

#include <Jolt/Core/JobSystemSingleThreaded.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/EPhysicsUpdateError.h>
#include <Jolt/Physics/PhysicsSystem.h>

#include "glm/vec3.hpp"
#include "OGSimulation/BodySlotOccupancy.h"
#include "OGSimulation/PhysicsWorldAdapter.h"
#include "OGSimulation/QueryGeometry.h"

using JoltWorldLoggerFn = std::function<void(const char*)>;
using JoltWorldClockFn = double (*)();

inline constexpr float kChaosRestitutionThresholdCmPerS2 = 1000.f;
inline constexpr float kChaosParityStepSeconds = 1.f / 60.f;
inline constexpr float kChaosParityMinVelocityForRestitutionCmPerS = kChaosRestitutionThresholdCmPerS2 * kChaosParityStepSeconds;

struct SlotBodyTemplate
{
	PhysicalObjectDescriptor descriptor;
	uint8_t declarationIndex = 0;
};

struct JoltWorldConfig
{
	uint32_t simulatableSlots = kMaxSimulatableSlots;
	std::vector<SlotBodyTemplate> slotTemplate;
	std::vector<JoltLayerKey> staticLayers;
	uint32_t ringDepthTicks = 0;
	uint32_t ringSlotBytes = 16u * 1024u;
	glm::vec3 gravityCmPerS2{ 0.f, 0.f, -980.f };
	glm::vec3 parkingPositionCm{ 0.f };
	float minVelocityForRestitutionCmPerS = kChaosParityMinVelocityForRestitutionCmPerS;
	uint32_t tempAllocatorBytes = 16u << 20;
	uint32_t maxBodies = 1024;
	uint32_t maxBodyPairs = 4096;
	uint32_t maxContactConstraints = 2048;
	JoltWorldClockFn clock = nullptr;
};

enum class JoltContactOverflow : uint8_t
{
	ManifoldCacheFull = 0,
	BodyPairCacheFull = 1,
	ContactConstraintsFull = 2,
	Count = 3
};

class JoltWorld
{
public:
	static constexpr SnapshotCoverage coverage{};

	OGSIMULATIONJOLT_API JoltWorld(JoltRuntime& runtime, const JoltWorldConfig& config, JoltWorldLoggerFn logger);
	OGSIMULATIONJOLT_API ~JoltWorld();

	JoltWorld(const JoltWorld&) = delete;
	JoltWorld& operator=(const JoltWorld&) = delete;

	OGSIMULATIONJOLT_API void step(float dt);
	OGSIMULATIONJOLT_API void saveTick(SimTick tick);
	OGSIMULATIONJOLT_API bool restoreTick(SimTick tick);
	OGSIMULATIONJOLT_API bool restoreFromSnapshot(const JoltStateSlot& slot);
	[[nodiscard]] OGSIMULATIONJOLT_API bool hasTick(SimTick tick) const;
	[[nodiscard]] OGSIMULATIONJOLT_API std::optional<SimTick> oldestHeldTick() const;
	OGSIMULATIONJOLT_API void invalidateAllTicks();
	OGSIMULATIONJOLT_API void saveScratch();
	OGSIMULATIONJOLT_API void commitScratch(SimTick tick);
	OGSIMULATIONJOLT_API void applyOccupancy(const BodySlotOccupancy& occupancy);
	[[nodiscard]] OGSIMULATIONJOLT_API std::optional<uint64_t> stateHash(SimTick tick) const;

	[[nodiscard]] JPH::PhysicsSystem& physics() { return m_physics; }
	[[nodiscard]] const JPH::PhysicsSystem& physics() const { return m_physics; }
	[[nodiscard]] JPH::BodyInterface& bodies() { return m_physics.GetBodyInterfaceNoLock(); }
	[[nodiscard]] const JPH::BodyInterface& bodies() const { return m_physics.GetBodyInterfaceNoLock(); }
	[[nodiscard]] const JPH::NarrowPhaseQuery& query() const { return m_physics.GetNarrowPhaseQueryNoLock(); }
	[[nodiscard]] const JoltLayerTable& layers() const { return m_layers; }
	[[nodiscard]] ShapeEnableTable& shapeEnables() { return m_shapeEnables; }
	[[nodiscard]] const ShapeEnableTable& shapeEnables() const { return m_shapeEnables; }
	[[nodiscard]] const JoltStateRing& ring() const { return m_ring; }
	[[nodiscard]] const BodySlotOccupancy& appliedOccupancy() const { return m_appliedOccupancy; }

	[[nodiscard]] uint32_t simulatableSlots() const { return m_simulatableSlots; }
	[[nodiscard]] uint32_t bodiesPerSlot() const { return static_cast<uint32_t>(m_template.size()); }
	[[nodiscard]] OGSIMULATIONJOLT_API JPH::BodyID slotBodyId(uint32_t slot, uint32_t templateIndex) const;
	[[nodiscard]] OGSIMULATIONJOLT_API JPH::ObjectLayer templateLayer(uint32_t templateIndex) const;

	[[nodiscard]] OGSIMULATIONJOLT_API JPH::BodyID createStaticBody(const JPH::BodyCreationSettings& settings);

	[[nodiscard]] OGSIMULATIONJOLT_API uint64_t liveStateHash() const;
	[[nodiscard]] uint64_t physicsStepCount() const { return m_physicsStepCount; }
	[[nodiscard]] uint64_t contactOverflowCount(JoltContactOverflow kind) const { return m_overflow[static_cast<size_t>(kind)].occurrences; }

private:
	struct TemplateBody
	{
		JPH::ObjectLayer layer = 0;
		float gravityFactor = 0.f;
	};

	enum class RestoreUndo : uint8_t
	{
		Take,
		Skip
	};

	struct OverflowLog
	{
		uint64_t occurrences = 0;
		uint64_t occurrencesAtLastLine = 0;
		std::optional<double> lastLineSeconds;
	};

	void buildSlotBodies(const JoltWorldConfig& config);
	void applyOccupancyToSlots(const BodySlotOccupancy& occupancy, bool forceEverySlot);
	void saveInto(JoltStateSlot& slot, SimTick tick);
	[[nodiscard]] bool liveBodySetMatches(const JoltStateSlot& slot);
	[[nodiscard]] bool restoreSlot(const JoltStateSlot& slot, RestoreUndo undo, const char* caller);
	[[nodiscard]] bool restoreBytes(const JoltStateSlot& slot);
	void reportUpdateErrors(JPH::EPhysicsUpdateError errors);
	void log(const char* message) const;

	JoltWorldLoggerFn m_logger;
	JoltWorldClockFn m_clock = nullptr;
	uint32_t m_simulatableSlots = 0;
	std::vector<TemplateBody> m_template;
	JoltLayerTable m_layers;
	JPH::TempAllocatorImpl m_tempAllocator;
	JPH::JobSystemSingleThreaded m_jobSystem;
	JPH::PhysicsSystem m_physics;
	JoltStateRing m_ring;
	JoltFixedStateRecorder m_recorder;
	ShapeEnableTable m_shapeEnables;
	BodySlotOccupancy m_appliedOccupancy;
	JPH::BodyIDVector m_liveBodies;
	uint32_t m_nextStaticBodyIndex = 0;
	uint64_t m_physicsStepCount = 0;
	std::optional<SimTick> m_lastSavedTick;
	std::array<OverflowLog, static_cast<size_t>(JoltContactOverflow::Count)> m_overflow{};
};

static_assert(PhysicsWorldAdapter<JoltWorld>, "JoltWorld must satisfy og-simulation's PhysicsWorldAdapter concept");
