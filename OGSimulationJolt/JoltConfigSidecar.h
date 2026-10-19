#pragma once
// SPDX-License-Identifier: MPL-2.0
// docs/JoltConfigSidecar-rationale.md

#include <bitset>
#include <cstdint>

#include "OGSimulation/BodySlotOccupancy.h"
#include "OGSimulation/OGAssert.h"
#include "OGSimulation/QueryGeometry.h"

inline constexpr uint32_t kShapeEnableCapacity = 256;

using ShapeEnableBits = std::bitset<kShapeEnableCapacity>;

class ShapeEnableTable
{
public:
	ShapeEnableTable() { m_enabled.set(); }

	void enable(ShapeId shape) { m_enabled.set(checkedIndex(shape)); }
	void disable(ShapeId shape) { m_enabled.reset(checkedIndex(shape)); }
	[[nodiscard]] bool isEnabled(ShapeId shape) const { return m_enabled.test(checkedIndex(shape)); }

	[[nodiscard]] const ShapeEnableBits& bits() const { return m_enabled; }
	void restore(const ShapeEnableBits& bits) { m_enabled = bits; }

private:
	static uint32_t checkedIndex(ShapeId shape)
	{
		OG_CHECK(shape.value < kShapeEnableCapacity, "ShapeEnableTable: ShapeId beyond kShapeEnableCapacity");
		return shape.value < kShapeEnableCapacity ? shape.value : kShapeEnableCapacity - 1u;
	}

	ShapeEnableBits m_enabled;
};

struct ConfigSidecar
{
	BodySlotOccupancy occupancy;
	ShapeEnableBits shapeEnables;

	bool operator==(const ConfigSidecar&) const = default;
};
