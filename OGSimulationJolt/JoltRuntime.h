#pragma once
// SPDX-License-Identifier: MPL-2.0
// docs/JoltRuntime-rationale.md

#include <cstdint>

#include "OGSimulationJolt/JoltDefineChecks.h"
#include "OGSimulationJolt/OGJoltExport.h"

class JoltRuntime final
{
public:
	using LogFn = void (*)(const char* message);

	[[nodiscard]] static JoltRuntime& acquire(LogFn logger)
	{
		using JPH::uint64;
		return acquireForVersion(JPH_VERSION_ID, logger);
	}

	OGSIMULATIONJOLT_API static void release();

	[[nodiscard]] OGSIMULATIONJOLT_API static uint32_t referenceCount();

	[[nodiscard]] OGSIMULATIONJOLT_API static uint64_t libraryVersionId();

	[[nodiscard]] OGSIMULATIONJOLT_API static JoltRuntime& acquireForVersion(uint64_t callerVersionId, LogFn logger);

	JoltRuntime(const JoltRuntime&) = delete;
	JoltRuntime& operator=(const JoltRuntime&) = delete;

private:
	JoltRuntime() = default;
};
