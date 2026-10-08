#pragma once
// SPDX-License-Identifier: MPL-2.0
// docs/JoltSpatialQueryAdapter-rationale.md · docs/JoltSpatialQueryAdapter-guards.md

#include <cstdint>
#include <optional>
#include <tuple>
#include <vector>

#include "OGSimulationJolt/JoltDefineChecks.h"
#include "OGSimulationJolt/JoltLayerTable.h"
#include "OGSimulationJolt/JoltWorld.h"
#include "OGSimulationJolt/OGJoltExport.h"

#include <Jolt/Core/Reference.h>
#include <Jolt/Physics/Body/BodyID.h>
#include <Jolt/Physics/Collision/Shape/Shape.h>

#include "glm/mat4x4.hpp"
#include "glm/vec3.hpp"
#include "OGSimulation/BodyId.h"
#include "OGSimulation/QueryGeometry.h"
#include "OGSimulation/SpatialQueryAdapter.h"
#include "OGSimulation/SpatialQueryResult.h"

using JoltWorldAccessCheckFn = bool (*)();

class JoltWorldAccessScope
{
public:
	OGSIMULATIONJOLT_API JoltWorldAccessScope();
	OGSIMULATIONJOLT_API ~JoltWorldAccessScope();

	JoltWorldAccessScope(const JoltWorldAccessScope&) = delete;
	JoltWorldAccessScope& operator=(const JoltWorldAccessScope&) = delete;

	[[nodiscard]] OGSIMULATIONJOLT_API static bool isOpenOnThisThread();
};

namespace joltQueryRule
{
	inline constexpr uint32_t kCategoryCount = 32;

	constexpr uint32_t categoryBit(uint32_t category)
	{
		return category < kCategoryCount ? (1u << category) : 0u;
	}

	constexpr uint32_t effectiveObjectMask(uint32_t searchCategories, uint32_t mappedCategories)
	{
		return searchCategories & mappedCategories;
	}

	constexpr bool isTraceChannelQuery(uint32_t searchCategories, uint32_t mappedCategories)
	{
		return effectiveObjectMask(searchCategories, mappedCategories) == 0u;
	}

	constexpr bool isSilentlyEmptyObjectQuery(uint32_t searchCategories, uint32_t mappedCategories)
	{
		return searchCategories != 0u && effectiveObjectMask(searchCategories, mappedCategories) == 0u;
	}

	constexpr uint32_t unmappedSearchMask(uint32_t searchCategories, uint32_t mappedCategories)
	{
		return searchCategories & ~mappedCategories;
	}

	constexpr bool respondsToTraceCategory(const JoltLayerKey& key, uint32_t traceCategory)
	{
		if (key == kParkedLayerKey)
		{
			return false;
		}
		return key.broadPhaseClass == JoltBroadPhaseClass::Moving || (key.blockingCategories & categoryBit(traceCategory)) != 0u;
	}

	constexpr bool blocksTraceCategory(const JoltLayerKey& key, uint32_t traceCategory)
	{
		return key != kParkedLayerKey && (key.blockingCategories & categoryBit(traceCategory)) != 0u;
	}

	static_assert(isTraceChannelQuery(0u, 0b111111u) && !isTraceChannelQuery(0b000011u, 0b111111u),
		"joltQueryRule: an empty search mask is a trace-channel query; a mapped one is an object query");
	static_assert(isTraceChannelQuery(0b1000000u, 0b111111u) && isSilentlyEmptyObjectQuery(0b1000000u, 0b111111u),
		"joltQueryRule: a non-empty search whose every bit is unmapped falls back to the trace channel, as the Chaos object-query "
		"params are then empty and UE builds a trace query (CreateQueryFilterData) - it is NOT a query that matches nothing");
	static_assert(effectiveObjectMask(0b1000011u, 0b111111u) == 0b11u && unmappedSearchMask(0b1000011u, 0b111111u) == 0b1000000u,
		"joltQueryRule: unmapped search bits are dropped from an object query, as Chaos drops them from FCollisionObjectQueryParams");
	static_assert(respondsToTraceCategory(JoltLayerKey{ JoltBroadPhaseClass::Moving, 0b1u, 0u }, 2u)
			&& !blocksTraceCategory(JoltLayerKey{ JoltBroadPhaseClass::Moving, 0b1u, 0u }, 2u),
		"joltQueryRule: a slot body overlaps every trace channel it does not block (the Chaos factory's ECR_Overlap default)");
	static_assert(respondsToTraceCategory(JoltLayerKey{ JoltBroadPhaseClass::Static, 0b10000u, 0b100u }, 2u)
			&& !respondsToTraceCategory(JoltLayerKey{ JoltBroadPhaseClass::Static, 0b10000u, 0b011u }, 2u),
		"joltQueryRule: a static answers a trace channel only where it blocks it (the importer records Block responses only)");
	static_assert(!respondsToTraceCategory(kParkedLayerKey, 2u) && !blocksTraceCategory(kParkedLayerKey, 2u),
		"joltQueryRule: a PARKED slot body answers no query of either kind");
	static_assert(categoryBit(32u) == 0u && categoryBit(31u) == 0x80000000u,
		"joltQueryRule: a category id a 32-bit mask cannot hold names no category (no shift UB)");
} // namespace joltQueryRule

struct JoltSpatialQueryConfig
{
	uint32_t mappedCategories = 0;
	JoltWorldAccessCheckFn accessCheck = nullptr;
};

struct JoltQueryHitKey
{
	uint8_t isStatic = 0;
	uint32_t simulatableId = 0;
	uint8_t declarationIndex = 0;
	uint32_t shapeIndex = 0;
	uint32_t staticBodyIndex = 0;
	uint32_t staticElementIndex = 0;

	auto operator<=>(const JoltQueryHitKey&) const = default;
	bool operator==(const JoltQueryHitKey&) const = default;
};

class JoltSpatialQueryAdapter
{
public:
	OGSIMULATIONJOLT_API JoltSpatialQueryAdapter(JoltWorld& world, JoltWorldLoggerFn logger, JoltSpatialQueryConfig config);

	JoltSpatialQueryAdapter(const JoltSpatialQueryAdapter&) = delete;
	JoltSpatialQueryAdapter& operator=(const JoltSpatialQueryAdapter&) = delete;

	OGSIMULATIONJOLT_API QueryVolumeId registerVolume(const QueryVolumeDescriptor& descriptor, BodyId ignoredRootBodyId);
	OGSIMULATIONJOLT_API ShapeId registerShape(BodyId body, uint32_t shapeIndex, std::optional<BodyId> rootBodyId);
	OGSIMULATIONJOLT_API void excludeStaticBodiesFromQueries(const std::vector<JPH::BodyID>& staticBodies);

	OGSIMULATIONJOLT_API SpatialQueryReport overlap(const std::vector<QueryVolumeId>& volumeIds);
	OGSIMULATIONJOLT_API SweepHit sweep(QueryVolumeId volumeId, const glm::mat4& transform, const glm::vec3& delta);
	OGSIMULATIONJOLT_API void setVolumeParentTransform(QueryVolumeId volumeId, const glm::mat4& transform);
	OGSIMULATIONJOLT_API void enableShape(ShapeId shapeId);
	OGSIMULATIONJOLT_API void disableShape(ShapeId shapeId);

	[[nodiscard]] OGSIMULATIONJOLT_API bool callerMayAccessWorld() const;
	[[nodiscard]] OGSIMULATIONJOLT_API std::optional<JoltQueryHitKey> hitKeyOf(BodyId bodyId, uint32_t staticElementIndex = 0) const;
	[[nodiscard]] uint32_t volumeCount() const { return static_cast<uint32_t>(m_volumes.size()); }
	[[nodiscard]] uint32_t mappedCategories() const { return m_mappedCategories; }

private:
	struct VolumeEntry
	{
		JPH::RefConst<JPH::Shape> shape;
		uint32_t objectMask = 0;
		uint32_t traceCategory = 0;
		BodyId ignoredRootBodyId;
		glm::mat4 parentTransform{ 1.f };
		glm::mat4 offsetTransform{ 1.f };
	};

	struct Candidate
	{
		JoltQueryHitKey key;
		JPH::BodyID body;
		float fraction = 0.f;
		float penetrationDepth = 0.f;
		JPH::Vec3 penetrationAxis = JPH::Vec3::sZero();
		JPH::Vec3 contactPointOnBody = JPH::Vec3::sZero();
		uint32_t subShapeId = 0;
	};

	enum class CandidatePurpose : uint8_t
	{
		Overlap,
		Sweep
	};

	[[nodiscard]] const VolumeEntry* volumeOf(QueryVolumeId volumeId) const;
	[[nodiscard]] std::vector<Candidate> collideAt(const VolumeEntry& volume, const glm::vec3& positionCm, CandidatePurpose purpose) const;
	[[nodiscard]] std::vector<Candidate> castFrom(const VolumeEntry& volume, const glm::vec3& startCm, const glm::vec3& deltaCm) const;
	[[nodiscard]] std::optional<JoltQueryHitKey> keyOf(const JPH::BodyID& body, uint32_t subShapeId) const;
	[[nodiscard]] BodyId rootOf(BodyId body) const;
	[[nodiscard]] SpatialQueryHit hitOf(const Candidate& candidate) const;
	[[nodiscard]] CollisionCategories categoriesOf(const JPH::BodyID& body) const;
	void diagnoseVolumeCategories(const QueryVolumeDescriptor& descriptor, QueryVolumeId volumeId) const;
	void log(const char* message) const;

	friend class JoltQueryBodyFilter;

	JoltWorld& m_world;
	JoltWorldLoggerFn m_logger;
	uint32_t m_mappedCategories = 0;
	JoltWorldAccessCheckFn m_accessCheck = nullptr;
	std::vector<VolumeEntry> m_volumes;
	std::vector<uint32_t> m_rootIndexOf;
	std::vector<uint8_t> m_shapeRegistered;
	std::vector<uint32_t> m_queryExcludedStatics;
};

static_assert(SpatialQueryAdapter<JoltSpatialQueryAdapter>, "JoltSpatialQueryAdapter must satisfy og-simulation's SpatialQueryAdapter concept");
