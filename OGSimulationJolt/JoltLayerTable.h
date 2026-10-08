#pragma once
// SPDX-License-Identifier: MPL-2.0
// docs/JoltLayerTable-rationale.md

#include <compare>
#include <cstdint>
#include <optional>
#include <vector>

#include "OGSimulationJolt/JoltDefineChecks.h"
#include "OGSimulationJolt/OGJoltExport.h"

#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseLayer.h>
#include <Jolt/Physics/Collision/ObjectLayer.h>

#include "OGSimulation/StaticGeometry.h"

enum class JoltBroadPhaseClass : uint8_t
{
	Static = 0,
	Moving = 1
};

inline constexpr uint32_t kJoltBroadPhaseClassCount = 2;

struct JoltLayerKey
{
	JoltBroadPhaseClass broadPhaseClass = JoltBroadPhaseClass::Moving;
	uint32_t categories = 0;
	uint32_t blockingCategories = 0;

	bool operator==(const JoltLayerKey&) const = default;
	auto operator<=>(const JoltLayerKey&) const = default;
};

inline constexpr JoltLayerKey kParkedLayerKey{ JoltBroadPhaseClass::Moving, 0u, 0u };

class JoltLayerTable;

class JoltBroadPhaseLayers final : public JPH::BroadPhaseLayerInterface
{
public:
	explicit JoltBroadPhaseLayers(const JoltLayerTable& table) : m_table(table) {}

	JPH::uint GetNumBroadPhaseLayers() const override;
	JPH::BroadPhaseLayer GetBroadPhaseLayer(JPH::ObjectLayer layer) const override;

private:
	const JoltLayerTable& m_table;
};

class JoltObjectVsBroadPhaseFilter final : public JPH::ObjectVsBroadPhaseLayerFilter
{
public:
	explicit JoltObjectVsBroadPhaseFilter(const JoltLayerTable& table) : m_table(table) {}

	bool ShouldCollide(JPH::ObjectLayer layer, JPH::BroadPhaseLayer broadPhaseLayer) const override;

private:
	const JoltLayerTable& m_table;
};

class JoltObjectLayerPairFilter final : public JPH::ObjectLayerPairFilter
{
public:
	explicit JoltObjectLayerPairFilter(const JoltLayerTable& table) : m_table(table) {}

	bool ShouldCollide(JPH::ObjectLayer first, JPH::ObjectLayer second) const override;

private:
	const JoltLayerTable& m_table;
};

class JoltLayerTable final
{
public:
	OGSIMULATIONJOLT_API explicit JoltLayerTable(std::vector<JoltLayerKey> keys);

	JoltLayerTable(const JoltLayerTable&) = delete;
	JoltLayerTable& operator=(const JoltLayerTable&) = delete;

	[[nodiscard]] OGSIMULATIONJOLT_API std::optional<JPH::ObjectLayer> findLayer(const JoltLayerKey& key) const;
	[[nodiscard]] OGSIMULATIONJOLT_API JPH::ObjectLayer layerOf(const JoltLayerKey& key) const;
	[[nodiscard]] OGSIMULATIONJOLT_API const JoltLayerKey& keyOf(JPH::ObjectLayer layer) const;

	[[nodiscard]] uint32_t layerCount() const { return static_cast<uint32_t>(m_keys.size()); }
	[[nodiscard]] JPH::ObjectLayer parkedLayer() const { return m_parkedLayer; }

	[[nodiscard]] OGSIMULATIONJOLT_API bool shouldCollide(JPH::ObjectLayer first, JPH::ObjectLayer second) const;
	[[nodiscard]] OGSIMULATIONJOLT_API bool queryMatches(JPH::ObjectLayer layer, uint32_t queryAnyOf) const;
	[[nodiscard]] OGSIMULATIONJOLT_API bool mayCollideWithClass(JPH::ObjectLayer layer, JoltBroadPhaseClass broadPhaseClass) const;

	[[nodiscard]] const JPH::BroadPhaseLayerInterface& broadPhaseLayers() const { return m_broadPhaseLayers; }
	[[nodiscard]] const JPH::ObjectVsBroadPhaseLayerFilter& objectVsBroadPhaseFilter() const { return m_objectVsBroadPhase; }
	[[nodiscard]] const JPH::ObjectLayerPairFilter& objectLayerPairFilter() const { return m_objectLayerPairs; }

private:
	std::vector<JoltLayerKey> m_keys;
	std::vector<uint8_t> m_collidesWithClassBits;
	JPH::ObjectLayer m_parkedLayer = 0;
	JoltBroadPhaseLayers m_broadPhaseLayers{ *this };
	JoltObjectVsBroadPhaseFilter m_objectVsBroadPhase{ *this };
	JoltObjectLayerPairFilter m_objectLayerPairs{ *this };
};

class JoltQueryLayerFilter final : public JPH::ObjectLayerFilter
{
public:
	JoltQueryLayerFilter(const JoltLayerTable& table, uint32_t queryAnyOf) : m_table(table), m_queryAnyOf(queryAnyOf) {}

	bool ShouldCollide(JPH::ObjectLayer layer) const override { return m_table.queryMatches(layer, m_queryAnyOf); }

private:
	const JoltLayerTable& m_table;
	uint32_t m_queryAnyOf = 0;
};

[[nodiscard]] OGSIMULATIONJOLT_API std::vector<JoltLayerKey> staticLayerKeysOf(const StaticWorldDescription& description);
