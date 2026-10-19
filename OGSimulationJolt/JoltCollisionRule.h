#pragma once
// SPDX-License-Identifier: MPL-2.0
// docs/JoltCollisionRule-rationale.md

#include <cstdint>

namespace joltCollisionRule
{
	constexpr bool shouldCollide(uint32_t aIn, uint32_t aWith, uint32_t bIn, uint32_t bWith)
	{
		return (aWith & bIn) != 0u && (bWith & aIn) != 0u;
	}

	constexpr bool queryMatches(uint32_t objectIn, uint32_t queryAnyOf)
	{
		return (objectIn & queryAnyOf) != 0u;
	}

	static_assert(shouldCollide(0b01u, 0b10u, 0b10u, 0b01u),
		"joltCollisionRule: two shapes that each list the other's category collide (Chaos AND rule)");
	static_assert(!shouldCollide(0b01u, 0b10u, 0b10u, 0b00u),
		"joltCollisionRule: M1 is Chaos's AND rule - a pair where only one side lists the other does NOT collide; "
		"OR one-way is the post-MVP follow-on and needs a ContactListener as well");
	static_assert(!shouldCollide(0b01u, 0b01u, 0b10u, 0b10u) && !shouldCollide(0u, 0u, 0b10u, ~0u),
		"joltCollisionRule: neither listing the other, or a PARKED (0, 0) side, never collides");
	static_assert(shouldCollide(0b01u, 0b10u, 0b10u, 0b01u) == shouldCollide(0b10u, 0b01u, 0b01u, 0b10u),
		"joltCollisionRule: the AND rule is symmetric");
	static_assert(queryMatches(0b0100u, 0b1100u) && !queryMatches(0b0010u, 0b1100u) && !queryMatches(0u, ~0u),
		"joltCollisionRule: queries match any-of over membership; a PARKED object (no category) matches no query");
} // namespace joltCollisionRule
