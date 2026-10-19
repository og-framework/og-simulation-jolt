// SPDX-License-Identifier: MPL-2.0
// docs/JoltTickWorldHost-rationale.md · docs/JoltTickWorldHost-guards.md

#include "OGSimulationJolt/JoltTickWorldHost.h"

#include <algorithm>
#include <cstddef>
#include <vector>

#include "OGSimulationJolt/JoltStaticWorldBuilder.h"

namespace joltTickWorldHost
{
JoltTickStatics splitStatics(const StaticWorldDescription& description, const std::vector<uint8_t>& physicsOnly)
{
	OG_CHECK(physicsOnly.size() == description.shapes.size(),
		"joltTickWorldHost::splitStatics: one physicsOnly flag per static shape. Was the brawler's split of its static import.");
	JoltTickStatics statics;
	for (std::size_t index = 0; index < description.shapes.size(); ++index)
	{
		StaticWorldDescription& target = physicsOnly[index] != 0u ? statics.physicsOnly : statics.queryable;
		target.shapes.push_back(description.shapes[index]);
	}
	return statics;
}

uint8_t rootDeclarationIndexOf(const std::vector<SlotBodyTemplate>& slotTemplate)
{
	const auto rootTemplate = std::find_if(slotTemplate.begin(), slotTemplate.end(),
		[](const SlotBodyTemplate& body) { return body.descriptor.body.isRoot; });
	OG_CHECK(rootTemplate != slotTemplate.end()
			&& std::none_of(rootTemplate + 1, slotTemplate.end(),
				[](const SlotBodyTemplate& body) { return body.descriptor.body.isRoot; }),
		"joltTickWorldHost::rootDeclarationIndexOf: the simulatable's physics composite must declare exactly one isRoot body "
		"(the capsule the render apply moves). Was the root checkf of the brawler's buildJoltWorld.");
	return static_cast<uint8_t>(rootTemplate - slotTemplate.begin());
}

JoltWorldConfig stepWorldConfigOf(const std::vector<SlotBodyTemplate>& slotTemplate, const StaticWorldDescription& queryableStatics,
	const StaticWorldDescription& physicsOnlyStatics, uint32_t ringDepthTicks, const glm::vec3& gravity)
{
	JoltWorldConfig config;
	config.simulatableSlots = kMaxSimulatableSlots;
	config.slotTemplate     = slotTemplate;
	config.staticLayers     = staticLayerKeysOf(queryableStatics);
	const std::vector<JoltLayerKey> physicsOnlyLayers = staticLayerKeysOf(physicsOnlyStatics);
	config.staticLayers.insert(config.staticLayers.end(), physicsOnlyLayers.begin(), physicsOnlyLayers.end());
	config.ringDepthTicks   = ringDepthTicks;
	config.gravityCmPerS2   = gravity;
	return config;
}

JoltTickStaticBodies buildStatics(JoltWorld& world, const StaticWorldDescription& queryableStatics,
	const StaticWorldDescription& physicsOnlyStatics, JoltRuntime::LogFn log)
{
	JoltStaticWorldBuilder queryableBuilder(world, log);
	queryableBuilder.build(queryableStatics);
	JoltStaticWorldBuilder physicsOnlyBuilder(world, log);
	physicsOnlyBuilder.build(physicsOnlyStatics);
	return JoltTickStaticBodies{ queryableBuilder.stats().bodies, physicsOnlyBuilder.stats().bodies };
}

BodyId slotRootBodyId(const JoltWorld& world, const std::vector<SlotBodyTemplate>& slotTemplate, uint32_t slot)
{
	for (uint32_t templateIndex = 0; templateIndex < slotTemplate.size(); ++templateIndex)
	{
		if (slotTemplate[templateIndex].descriptor.body.isRoot)
			return JoltPhysicsBodyAdapter::bodyIdOf(world.slotBodyId(slot, templateIndex));
	}
	OG_CHECK(false, "joltTickWorldHost::slotRootBodyId: the slot template has no isRoot body. Was the checkf of the brawler's "
		"joltSlotRootBodyId.");
	return BodyId{};
}

uint32_t boundBodyCount(const JoltWorld& world, const JoltBodyBindTable& bindTable)
{
	uint32_t bound = 0u;
	for (uint32_t slot = 0; slot < world.simulatableSlots(); ++slot)
	{
		for (uint32_t templateIndex = 0; templateIndex < world.bodiesPerSlot(); ++templateIndex)
		{
			if (bindTable.isBound(JoltPhysicsBodyAdapter::bodyIdOf(world.slotBodyId(slot, templateIndex))))
				++bound;
		}
	}
	return bound;
}

void prepareShadowStateSlot(JoltStateSlot& state, size_t bodyCapacity)
{
	state.bytes.assign(JoltWorldConfig{}.ringSlotBytes, 0u);
	state.bodyIds.reserve(bodyCapacity);
}

void copySavedState(const JoltStateSlot& saved, JoltStateSlot& target)
{
	OG_CHECK(saved.byteCount <= target.bytes.size(),
		"joltTickWorldHost::copySavedState: a saved tick must fit the pooled hand-over slot. Was the byte-count checkf of the "
		"brawler's publishShadowSlot_Step.");
	std::copy_n(saved.bytes.begin(), saved.byteCount, target.bytes.begin());
	target.byteCount = saved.byteCount;
	target.bodyIds.assign(saved.bodyIds.begin(), saved.bodyIds.end());
	target.sidecar   = saved.sidecar;
	target.stateHash = saved.stateHash;
	target.tick      = saved.tick;
	target.valid     = saved.valid;
}
} // namespace joltTickWorldHost
