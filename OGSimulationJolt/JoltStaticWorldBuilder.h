#pragma once
// SPDX-License-Identifier: MPL-2.0
// docs/JoltStaticWorldBuilder-rationale.md · docs/JoltStaticWorldBuilder-guards.md

#include <cstdint>
#include <optional>
#include <vector>

#include "OGSimulationJolt/JoltDefineChecks.h"
#include "OGSimulationJolt/JoltWorld.h"
#include "OGSimulationJolt/OGJoltExport.h"

#include <Jolt/Physics/Body/BodyID.h>

#include "OGSimulation/StaticGeometry.h"

struct JoltStaticSurface
{
	float friction = 0.f;
	float restitution = 0.f;
};

inline constexpr JoltStaticSurface kUnrealDefaultPhysicalMaterialSurface{ 0.7f, 0.3f };

struct JoltStaticWorldBuilderOptions
{
	uint32_t maxShapesPerChunk = 256;
	float convexRadiusFractionOfMinExtent = 0.05f;
	float maxConvexRadiusCm = 10.f;
	std::optional<JoltStaticSurface> surfaceForZeroFrictionAndRestitution = kUnrealDefaultPhysicalMaterialSurface;
};

struct JoltStaticBuildStats
{
	uint32_t shapesBuilt = 0;
	uint32_t groups = 0;
	uint32_t chunks = 0;
	uint32_t chunksSplitIntoSingles = 0;
	uint32_t skippedInvalidShape = 0;
	uint32_t skippedNoLayer = 0;
	uint32_t zeroSurfaceElements = 0;
	uint32_t duplicateStableKeys = 0;
	uint32_t approximatedScales = 0;
	std::vector<JPH::BodyID> bodies;
};

class JoltStaticWorldBuilder
{
public:
	OGSIMULATIONJOLT_API JoltStaticWorldBuilder(JoltWorld& world, JoltWorldLoggerFn logger, JoltStaticWorldBuilderOptions options = {});

	OGSIMULATIONJOLT_API StaticWorldBuildReport build(const StaticWorldDescription& description);

	[[nodiscard]] const JoltStaticBuildStats& stats() const { return m_stats; }

private:
	void log(const char* message) const;

	JoltWorld& m_world;
	JoltWorldLoggerFn m_logger;
	JoltStaticWorldBuilderOptions m_options;
	JoltStaticBuildStats m_stats;
	bool m_built = false;
};

static_assert(StaticWorldBuilder<JoltStaticWorldBuilder>, "JoltStaticWorldBuilder must satisfy og-simulation's StaticWorldBuilder concept");
