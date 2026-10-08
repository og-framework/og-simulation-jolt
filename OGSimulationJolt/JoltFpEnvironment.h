#pragma once
// SPDX-License-Identifier: MPL-2.0
// docs/JoltFpEnvironment-rationale.md

#include <cstdint>

#include "OGSimulationJolt/JoltDefineChecks.h"
#include "OGSimulationJolt/OGJoltExport.h"

#include <Jolt/Core/FPControlWord.h>
#include <Jolt/Core/FPFlushDenormals.h>

struct JoltFpMode
{
	bool readable = false;
	bool flushToZero = false;
	bool denormalsAreZero = false;
	bool roundToNearest = false;
	uint64_t rawControlWord = 0;
};

[[nodiscard]] OGSIMULATIONJOLT_API JoltFpMode readJoltFpMode();

[[nodiscard]] OGSIMULATIONJOLT_API bool isExpectedJoltStepFpMode(const JoltFpMode& mode);

#if defined(JPH_CPU_X86)
using JoltDenormalsAreZero = JPH::FPControlWord<_MM_DENORMALS_ZERO_ON, _MM_DENORMALS_ZERO_MASK>;
#else
struct JoltDenormalsAreZero
{
};
#endif

class JoltStepFpScope
{
public:
	JoltStepFpScope() = default;
	JoltStepFpScope(const JoltStepFpScope&) = delete;
	JoltStepFpScope& operator=(const JoltStepFpScope&) = delete;

private:
	JPH::FPFlushDenormals m_flushToZero;
	JoltDenormalsAreZero m_denormalsAreZero;
};
