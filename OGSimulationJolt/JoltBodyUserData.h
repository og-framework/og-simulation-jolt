#pragma once
// SPDX-License-Identifier: MPL-2.0
// docs/JoltBodyUserData-rationale.md

#include <cstdint>
#include <optional>

struct JoltBodyBinding
{
	uint32_t simulatableId = 0;
	uint8_t declarationIndex = 0;

	bool operator==(const JoltBodyBinding&) const = default;
};

namespace joltBodyUserData
{
	inline constexpr uint64_t kBoundBit = uint64_t(1) << 63;
	inline constexpr uint64_t kRootBit = uint64_t(1) << 62;

	constexpr uint64_t encodeUnbound(bool isRoot)
	{
		return isRoot ? kRootBit : uint64_t(0);
	}

	constexpr uint64_t encode(const JoltBodyBinding& binding, bool isRoot)
	{
		return kBoundBit | encodeUnbound(isRoot) | (uint64_t(binding.simulatableId) << 8) | uint64_t(binding.declarationIndex);
	}

	constexpr bool isRoot(uint64_t userData)
	{
		return (userData & kRootBit) != 0;
	}

	constexpr std::optional<JoltBodyBinding> decode(uint64_t userData)
	{
		if ((userData & kBoundBit) == 0)
		{
			return std::nullopt;
		}
		return JoltBodyBinding{ uint32_t((userData >> 8) & 0xFFFFFFFFu), uint8_t(userData & 0xFFu) };
	}

	static_assert(decode(encode(JoltBodyBinding{ 0xFFFFFFFFu, 0xFFu }, false)) == JoltBodyBinding{ 0xFFFFFFFFu, 0xFFu });
	static_assert(decode(encode(JoltBodyBinding{ 0xFFFFFFFFu, 0xFFu }, true)) == JoltBodyBinding{ 0xFFFFFFFFu, 0xFFu });
	static_assert(decode(encode(JoltBodyBinding{ 0u, 0u }, false)).has_value());
	static_assert(!decode(0u).has_value());
	static_assert(!decode(encodeUnbound(true)).has_value());
	static_assert(isRoot(encode(JoltBodyBinding{ 0u, 0u }, true)) && isRoot(encodeUnbound(true)));
	static_assert(!isRoot(encode(JoltBodyBinding{ 0xFFFFFFFFu, 0xFFu }, false)) && !isRoot(encodeUnbound(false)));
} // namespace joltBodyUserData
