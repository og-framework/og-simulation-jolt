// SPDX-License-Identifier: MPL-2.0
// docs/JoltLayerTable-rationale.md

#include "OGSimulationJolt/JoltLayerTable.h"

#include <algorithm>
#include <limits>
#include <utility>

#include "OGSimulation/OGAssert.h"
#include "OGSimulationJolt/JoltCollisionRule.h"

JPH::uint JoltBroadPhaseLayers::GetNumBroadPhaseLayers() const
{
	return kJoltBroadPhaseClassCount;
}

JPH::BroadPhaseLayer JoltBroadPhaseLayers::GetBroadPhaseLayer(JPH::ObjectLayer layer) const
{
	return JPH::BroadPhaseLayer(static_cast<JPH::BroadPhaseLayer::Type>(m_table.keyOf(layer).broadPhaseClass));
}

bool JoltObjectVsBroadPhaseFilter::ShouldCollide(JPH::ObjectLayer layer, JPH::BroadPhaseLayer broadPhaseLayer) const
{
	return m_table.mayCollideWithClass(layer, static_cast<JoltBroadPhaseClass>(broadPhaseLayer.GetValue()));
}

bool JoltObjectLayerPairFilter::ShouldCollide(JPH::ObjectLayer first, JPH::ObjectLayer second) const
{
	return m_table.shouldCollide(first, second);
}

JoltLayerTable::JoltLayerTable(std::vector<JoltLayerKey> keys)
	: m_keys(std::move(keys))
{
	m_keys.push_back(kParkedLayerKey);
	std::sort(m_keys.begin(), m_keys.end());
	m_keys.erase(std::unique(m_keys.begin(), m_keys.end()), m_keys.end());

	OG_CHECK(m_keys.size() <= static_cast<size_t>(std::numeric_limits<JPH::ObjectLayer>::max()),
		"JoltLayerTable: more distinct (broadphase class, categories, blockingCategories) keys than object layers");

	m_parkedLayer = layerOf(kParkedLayerKey);

	m_collidesWithClassBits.assign(m_keys.size(), 0u);
	for (size_t first = 0; first < m_keys.size(); ++first)
	{
		for (size_t second = 0; second < m_keys.size(); ++second)
		{
			if (shouldCollide(static_cast<JPH::ObjectLayer>(first), static_cast<JPH::ObjectLayer>(second)))
			{
				m_collidesWithClassBits[first] |= static_cast<uint8_t>(1u << static_cast<uint32_t>(m_keys[second].broadPhaseClass));
			}
		}
	}
}

std::optional<JPH::ObjectLayer> JoltLayerTable::findLayer(const JoltLayerKey& key) const
{
	const auto it = std::lower_bound(m_keys.begin(), m_keys.end(), key);
	if (it == m_keys.end() || *it != key)
	{
		return std::nullopt;
	}
	return static_cast<JPH::ObjectLayer>(it - m_keys.begin());
}

JPH::ObjectLayer JoltLayerTable::layerOf(const JoltLayerKey& key) const
{
	const std::optional<JPH::ObjectLayer> layer = findLayer(key);
	OG_CHECK(layer.has_value(), "JoltLayerTable::layerOf - the key was not declared when the table was built; layers are allocated once, at world build");
	return layer.value_or(m_parkedLayer);
}

const JoltLayerKey& JoltLayerTable::keyOf(JPH::ObjectLayer layer) const
{
	OG_CHECK(layer < m_keys.size(), "JoltLayerTable::keyOf - object layer out of range");
	return m_keys[layer < m_keys.size() ? layer : m_parkedLayer];
}

bool JoltLayerTable::shouldCollide(JPH::ObjectLayer first, JPH::ObjectLayer second) const
{
	const JoltLayerKey& a = keyOf(first);
	const JoltLayerKey& b = keyOf(second);
	return joltCollisionRule::shouldCollide(a.categories, a.blockingCategories, b.categories, b.blockingCategories);
}

bool JoltLayerTable::queryMatches(JPH::ObjectLayer layer, uint32_t queryAnyOf) const
{
	return joltCollisionRule::queryMatches(keyOf(layer).categories, queryAnyOf);
}

bool JoltLayerTable::mayCollideWithClass(JPH::ObjectLayer layer, JoltBroadPhaseClass broadPhaseClass) const
{
	OG_CHECK(layer < m_collidesWithClassBits.size(), "JoltLayerTable::mayCollideWithClass - object layer out of range");
	return layer < m_collidesWithClassBits.size()
		&& (m_collidesWithClassBits[layer] & (1u << static_cast<uint32_t>(broadPhaseClass))) != 0u;
}

std::vector<JoltLayerKey> staticLayerKeysOf(const StaticWorldDescription& description)
{
	std::vector<JoltLayerKey> keys;
	keys.reserve(description.shapes.size());
	for (const StaticShapeDescriptor& shape : description.shapes)
	{
		keys.push_back(JoltLayerKey{ JoltBroadPhaseClass::Static, shape.categories.bits, shape.blockingCategories.bits });
	}
	return keys;
}
