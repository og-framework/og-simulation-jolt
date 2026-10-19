#pragma once
// SPDX-License-Identifier: MPL-2.0
// docs/JoltContactCounter-rationale.md · docs/JoltContactCounter-guards.md

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <type_traits>

#include "OGSimulationJolt/JoltBodyUserData.h"
#include "OGSimulationJolt/JoltDefineChecks.h"
#include "OGSimulationJolt/OGJoltExport.h"

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Collision/ContactListener.h>

#include "OGSimulation/BodySlotOccupancy.h"

enum class JoltContactBodyClass : uint8_t
{
	Root = 0,
	Child = 1,
	Static = 2
};

enum class JoltContactPairClass : uint8_t
{
	RootRoot = 0,
	RootChild = 1,
	RootStatic = 2,
	ChildChild = 3,
	ChildStatic = 4,
	StaticStatic = 5,
	Count = 6
};

inline constexpr size_t kJoltContactPairClassCount = static_cast<size_t>(JoltContactPairClass::Count);

constexpr JoltContactBodyClass joltContactBodyClassOf(uint64_t userData)
{
	if (joltBodyUserData::isRoot(userData))
	{
		return JoltContactBodyClass::Root;
	}
	return joltBodyUserData::decode(userData).has_value() ? JoltContactBodyClass::Child : JoltContactBodyClass::Static;
}

constexpr JoltContactPairClass joltContactPairClassOf(JoltContactBodyClass first, JoltContactBodyClass second)
{
	const JoltContactBodyClass low = first < second ? first : second;
	const JoltContactBodyClass high = first < second ? second : first;
	if (low == JoltContactBodyClass::Root)
	{
		return high == JoltContactBodyClass::Root ? JoltContactPairClass::RootRoot
			: high == JoltContactBodyClass::Child ? JoltContactPairClass::RootChild
			: JoltContactPairClass::RootStatic;
	}
	if (low == JoltContactBodyClass::Child)
	{
		return high == JoltContactBodyClass::Child ? JoltContactPairClass::ChildChild : JoltContactPairClass::ChildStatic;
	}
	return JoltContactPairClass::StaticStatic;
}

static_assert(joltContactBodyClassOf(joltBodyUserData::encode(JoltBodyBinding{ 7u, 5u }, true)) == JoltContactBodyClass::Root);
static_assert(joltContactBodyClassOf(joltBodyUserData::encodeUnbound(true)) == JoltContactBodyClass::Root);
static_assert(joltContactBodyClassOf(joltBodyUserData::encode(JoltBodyBinding{ 7u, 0u }, false)) == JoltContactBodyClass::Child);
static_assert(joltContactBodyClassOf(joltBodyUserData::encodeUnbound(false)) == JoltContactBodyClass::Static);
static_assert(joltContactBodyClassOf(uint64_t(0x1234)) == JoltContactBodyClass::Static);
static_assert(joltContactPairClassOf(JoltContactBodyClass::Root, JoltContactBodyClass::Root) == JoltContactPairClass::RootRoot);
static_assert(joltContactPairClassOf(JoltContactBodyClass::Child, JoltContactBodyClass::Root) == JoltContactPairClass::RootChild);
static_assert(joltContactPairClassOf(JoltContactBodyClass::Root, JoltContactBodyClass::Child) == JoltContactPairClass::RootChild);
static_assert(joltContactPairClassOf(JoltContactBodyClass::Static, JoltContactBodyClass::Root) == JoltContactPairClass::RootStatic);
static_assert(joltContactPairClassOf(JoltContactBodyClass::Child, JoltContactBodyClass::Child) == JoltContactPairClass::ChildChild);
static_assert(joltContactPairClassOf(JoltContactBodyClass::Static, JoltContactBodyClass::Child) == JoltContactPairClass::ChildStatic);
static_assert(joltContactPairClassOf(JoltContactBodyClass::Static, JoltContactBodyClass::Static) == JoltContactPairClass::StaticStatic);

struct JoltContactIdCounts
{
	uint32_t simulatableId = 0;
	std::array<uint32_t, kJoltContactPairClassCount> pairs{};
	uint32_t childDeclarations = 0;

	[[nodiscard]] uint32_t pairCount(JoltContactPairClass pairClass) const { return pairs[static_cast<size_t>(pairClass)]; }
};

struct JoltContactStepCounts
{
	static constexpr uint32_t kMaxIds = kMaxSimulatableSlots;
	static constexpr uint32_t kRecordedDeclarations = 32;

	std::array<uint32_t, kJoltContactPairClassCount> pairs{};
	uint32_t added = 0;
	uint32_t idCount = 0;
	std::array<JoltContactIdCounts, kMaxIds> ids{};
	uint32_t unrecordedIds = 0;
	uint32_t unrecordedDeclarations = 0;

	[[nodiscard]] uint32_t pairCount(JoltContactPairClass pairClass) const { return pairs[static_cast<size_t>(pairClass)]; }
	[[nodiscard]] OGSIMULATIONJOLT_API uint32_t manifolds() const;
	[[nodiscard]] OGSIMULATIONJOLT_API const JoltContactIdCounts* findId(uint32_t simulatableId) const;
};

class JoltContactCounter final : public JPH::ContactListener
{
public:
	JoltContactCounter() = default;
	JoltContactCounter(const JoltContactCounter&) = delete;
	JoltContactCounter& operator=(const JoltContactCounter&) = delete;

	OGSIMULATIONJOLT_API void OnContactAdded(const JPH::Body& first, const JPH::Body& second, const JPH::ContactManifold&, JPH::ContactSettings&) override;
	OGSIMULATIONJOLT_API void OnContactPersisted(const JPH::Body& first, const JPH::Body& second, const JPH::ContactManifold&, JPH::ContactSettings&) override;

	[[nodiscard]] OGSIMULATIONJOLT_API JoltContactStepCounts drain();

private:
	struct IdSlot
	{
		std::atomic<uint64_t> key{ 0 };
		std::array<std::atomic<uint32_t>, kJoltContactPairClassCount> pairs{};
		std::atomic<uint32_t> childDeclarations{ 0 };
	};

	void countManifold(const JPH::Body& first, const JPH::Body& second, bool added);
	void countBody(uint64_t userData, const std::optional<JoltBodyBinding>& binding, JoltContactPairClass pairClass, bool countPair);
	IdSlot* findOrClaimId(uint32_t simulatableId);

	std::array<std::atomic<uint32_t>, kJoltContactPairClassCount> m_pairs{};
	std::atomic<uint32_t> m_added{ 0 };
	std::array<IdSlot, JoltContactStepCounts::kMaxIds> m_ids{};
	std::atomic<uint32_t> m_unrecordedIds{ 0 };
	std::atomic<uint32_t> m_unrecordedDeclarations{ 0 };
};

static_assert(std::is_same_v<decltype(&JoltContactCounter::OnContactValidate), decltype(&JPH::ContactListener::OnContactValidate)>,
	"JoltContactCounter must not override OnContactValidate: the default accepts every contact, exactly as a world with no "
	"listener does, and an override that rejects or filters a contact changes the physics the counter only observes. Was "
	"task 78's passivity rule (OnContactValidate is not overridden).");
static_assert(std::atomic<uint64_t>::is_always_lock_free && std::atomic<uint32_t>::is_always_lock_free,
	"JoltContactCounter's callbacks run on Jolt's job threads and must not take a lock: the counters are lock-free atomics.");
