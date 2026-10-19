// SPDX-License-Identifier: MPL-2.0
// docs/JoltContactCounter-rationale.md · docs/JoltContactCounter-guards.md

#include "OGSimulationJolt/JoltContactCounter.h"

#include <algorithm>
#include <optional>

#include <Jolt/Physics/Body/Body.h>

uint32_t JoltContactStepCounts::manifolds() const
{
	uint32_t total = 0;
	for (const uint32_t count : pairs)
	{
		total += count;
	}
	return total;
}

const JoltContactIdCounts* JoltContactStepCounts::findId(uint32_t simulatableId) const
{
	for (uint32_t index = 0; index < idCount; ++index)
	{
		if (ids[index].simulatableId == simulatableId)
		{
			return &ids[index];
		}
	}
	return nullptr;
}

// ⛔G-01  docs/JoltContactCounter-guards.md
void JoltContactCounter::OnContactAdded(const JPH::Body& first, const JPH::Body& second, const JPH::ContactManifold&, JPH::ContactSettings&) { countManifold(first, second, true); }

// ⛔G-02  docs/JoltContactCounter-guards.md
void JoltContactCounter::OnContactPersisted(const JPH::Body& first, const JPH::Body& second, const JPH::ContactManifold&, JPH::ContactSettings&) { countManifold(first, second, false); }

void JoltContactCounter::countManifold(const JPH::Body& first, const JPH::Body& second, bool added)
{
	const uint64_t firstUserData = first.GetUserData();
	const uint64_t secondUserData = second.GetUserData();
	const JoltContactPairClass pairClass =
		joltContactPairClassOf(joltContactBodyClassOf(firstUserData), joltContactBodyClassOf(secondUserData));
	m_pairs[static_cast<size_t>(pairClass)].fetch_add(1u, std::memory_order_relaxed);
	if (added)
	{
		m_added.fetch_add(1u, std::memory_order_relaxed);
	}

	const std::optional<JoltBodyBinding> firstBinding = joltBodyUserData::decode(firstUserData);
	const std::optional<JoltBodyBinding> secondBinding = joltBodyUserData::decode(secondUserData);
	const bool sameId = firstBinding.has_value() && secondBinding.has_value() && firstBinding->simulatableId == secondBinding->simulatableId;
	countBody(firstUserData, firstBinding, pairClass, true);
	countBody(secondUserData, secondBinding, pairClass, !sameId);
}

void JoltContactCounter::countBody(uint64_t userData, const std::optional<JoltBodyBinding>& binding, JoltContactPairClass pairClass, bool countPair)
{
	if (!binding.has_value())
	{
		return;
	}
	IdSlot* slot = findOrClaimId(binding->simulatableId);
	if (slot == nullptr)
	{
		return;
	}
	if (countPair)
	{
		slot->pairs[static_cast<size_t>(pairClass)].fetch_add(1u, std::memory_order_relaxed);
	}
	if (joltBodyUserData::isRoot(userData))
	{
		return;
	}
	if (binding->declarationIndex < JoltContactStepCounts::kRecordedDeclarations)
	{
		slot->childDeclarations.fetch_or(uint32_t(1) << binding->declarationIndex, std::memory_order_relaxed);
	}
	else
	{
		m_unrecordedDeclarations.fetch_add(1u, std::memory_order_relaxed);
	}
}

JoltContactCounter::IdSlot* JoltContactCounter::findOrClaimId(uint32_t simulatableId)
{
	const uint64_t key = uint64_t(simulatableId) + 1u;
	for (IdSlot& slot : m_ids)
	{
		uint64_t held = slot.key.load(std::memory_order_relaxed);
		if (held == 0u && slot.key.compare_exchange_strong(held, key, std::memory_order_relaxed))
		{
			return &slot;
		}
		if (held == key)
		{
			return &slot;
		}
	}
	m_unrecordedIds.fetch_add(1u, std::memory_order_relaxed);
	return nullptr;
}

JoltContactStepCounts JoltContactCounter::drain()
{
	JoltContactStepCounts counts;
	for (size_t index = 0; index < kJoltContactPairClassCount; ++index)
	{
		counts.pairs[index] = m_pairs[index].exchange(0u, std::memory_order_relaxed);
	}
	counts.added = m_added.exchange(0u, std::memory_order_relaxed);
	counts.unrecordedIds = m_unrecordedIds.exchange(0u, std::memory_order_relaxed);
	counts.unrecordedDeclarations = m_unrecordedDeclarations.exchange(0u, std::memory_order_relaxed);

	for (IdSlot& slot : m_ids)
	{
		const uint64_t key = slot.key.exchange(0u, std::memory_order_relaxed);
		JoltContactIdCounts idCounts;
		for (size_t index = 0; index < kJoltContactPairClassCount; ++index)
		{
			idCounts.pairs[index] = slot.pairs[index].exchange(0u, std::memory_order_relaxed);
		}
		idCounts.childDeclarations = slot.childDeclarations.exchange(0u, std::memory_order_relaxed);
		if (key == 0u)
		{
			continue;
		}
		idCounts.simulatableId = static_cast<uint32_t>(key - 1u);
		counts.ids[counts.idCount++] = idCounts;
	}
	std::sort(counts.ids.begin(), counts.ids.begin() + counts.idCount,
		[](const JoltContactIdCounts& first, const JoltContactIdCounts& second) { return first.simulatableId < second.simulatableId; });
	return counts;
}
