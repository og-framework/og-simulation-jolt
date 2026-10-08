#pragma once
// SPDX-License-Identifier: MPL-2.0
// docs/JoltPhysicsFactory-rationale.md · docs/JoltPhysicsFactory-guards.md

#include <cstdint>
#include <functional>
#include <optional>
#include <vector>

#include "OGSimulationJolt/JoltBodyDefaults.h"
#include "OGSimulationJolt/JoltPhysicsBodyAdapter.h"
#include "OGSimulationJolt/JoltWorld.h"
#include "OGSimulationJolt/OGJoltExport.h"

#include "OGSimulation/BodyId.h"
#include "OGSimulation/PhysicsObjectFactory.h"
#include "OGSimulation/QueryGeometry.h"

struct JoltPhysicsFactoryOptions
{
	bool linearCastCharacters = false;
};

using JoltShapeRegistrarFn = std::function<ShapeId(BodyId body, uint32_t shapeIndex, std::optional<BodyId> rootBodyId)>;

class JoltPhysicsFactory
{
public:
	struct PhysicalObjectResult
	{
		BodyId bodyId;
		std::vector<ShapeId> shapeIds;
	};

	OGSIMULATIONJOLT_API JoltPhysicsFactory(JoltPhysicsBodyAdapter& bodyAdapter,
		std::vector<SlotBodyTemplate> slotTemplate,
		uint32_t slot,
		uint32_t simulatableId,
		JoltPhysicsFactoryOptions options = {},
		JoltShapeRegistrarFn shapeRegistrar = {});

	OGSIMULATIONJOLT_API PhysicalObjectResult createPhysicalObject(const PhysicalObjectDescriptor& descriptor, const char* name);

	[[nodiscard]] BodyId parentBodyId() const { return m_parentBodyId; }
	[[nodiscard]] uint32_t boundBodyCount() const { return m_nextTemplateIndex; }

	[[nodiscard]] OGSIMULATIONJOLT_API static ShapeId defaultShapeIdOf(BodyId body, uint32_t shapeIndex);
	[[nodiscard]] OGSIMULATIONJOLT_API static bool sameDescriptor(const PhysicalObjectDescriptor& first, const PhysicalObjectDescriptor& second);

private:
	void applyBodyDefaults(const JPH::BodyID& id, const SlotBodyTemplate& slotTemplate,
		const joltBodyDefaults::ChaosParityMassProperties& massProperties);

	JoltPhysicsBodyAdapter& m_bodyAdapter;
	std::vector<SlotBodyTemplate> m_template;
	uint32_t m_slot = 0;
	uint32_t m_simulatableId = 0;
	JoltPhysicsFactoryOptions m_options;
	JoltShapeRegistrarFn m_shapeRegistrar;
	BodyId m_parentBodyId;
	uint32_t m_nextTemplateIndex = 0;
};

static_assert(PhysicsObjectFactory<JoltPhysicsFactory>, "JoltPhysicsFactory must satisfy og-simulation's PhysicsObjectFactory concept");
