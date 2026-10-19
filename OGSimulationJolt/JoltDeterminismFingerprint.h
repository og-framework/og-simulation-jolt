#pragma once
// SPDX-License-Identifier: MPL-2.0
// docs/JoltDeterminismFingerprint-rationale.md

#include <cstdint>
#include <string>

#include "OGSimulationJolt/JoltRuntime.h"
#include "OGSimulationJolt/OGJoltExport.h"

struct JoltDeterminismFingerprint
{
	const char* engineName = "Jolt";
	uint32_t versionMajor = 0;
	uint32_t versionMinor = 0;
	uint32_t versionPatch = 0;
	uint64_t joltVersionId = 0;
	uint32_t simdWidthBits = 0;
	const char* instructionSet = "";
	bool fusedMultiplyAdd = false;
	bool stepFlushToZero = false;
	bool stepDenormalsAreZero = false;
	bool stepRoundToNearest = false;
	uint64_t probeStateHash = 0;

	[[nodiscard]] OGSIMULATIONJOLT_API uint64_t value() const;
};

[[nodiscard]] OGSIMULATIONJOLT_API JoltDeterminismFingerprint determinismFingerprint(JoltRuntime& runtime);

[[nodiscard]] OGSIMULATIONJOLT_API const char* joltBuildConfigurationOf(uint64_t joltVersionId);

[[nodiscard]] OGSIMULATIONJOLT_API std::string describeFingerprintMismatch(
	const JoltDeterminismFingerprint& local, const JoltDeterminismFingerprint& remote);
