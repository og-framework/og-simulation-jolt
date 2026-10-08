// SPDX-License-Identifier: MPL-2.0
// docs/JoltDeterminismFingerprint-rationale.md

#include "OGSimulationJolt/JoltDeterminismFingerprint.h"

#include <cstdio>
#include <cstring>

#include <Jolt/Physics/Collision/Shape/BoxShape.h>

#include "OGSimulationJolt/JoltFpEnvironment.h"
#include "OGSimulationJolt/JoltWorld.h"

namespace
{
	class FingerprintHash
	{
	public:
		void bytes(const void* data, size_t count)
		{
			const auto* bytes = static_cast<const uint8_t*>(data);
			for (size_t i = 0; i < count; ++i)
			{
				m_value ^= bytes[i];
				m_value *= 1099511628211ull;
			}
		}

		void u64(uint64_t value)
		{
			uint8_t littleEndian[8];
			for (int i = 0; i < 8; ++i)
			{
				littleEndian[i] = static_cast<uint8_t>(value >> (8 * i));
			}
			bytes(littleEndian, sizeof(littleEndian));
		}

		void text(const char* value) { bytes(value, std::strlen(value) + 1u); }

		[[nodiscard]] uint64_t value() const { return m_value; }

	private:
		uint64_t m_value = 14695981039346656037ull;
	};

	constexpr uint32_t kProbeTicks = 60;
	constexpr float kProbeDtSeconds = 1.f / 60.f;
	constexpr uint32_t kProbeCharacterCategory = 1u << 0;
	constexpr uint32_t kProbeWorldCategory = 1u << 1;

	uint64_t runProbeSimulation(JoltRuntime& runtime)
	{
		SlotBodyTemplate capsule;
		capsule.descriptor.body = BodyDescriptor{ .simulatePhysics = true, .enableGravity = true, .isRoot = true, .lockRotation = true };
		capsule.descriptor.shapes = { ShapeDescriptor{ CapsuleGeometry{ 42.f, 96.f },
			CollisionCategories{ kProbeCharacterCategory }, CollisionCategories{ kProbeCharacterCategory | kProbeWorldCategory } } };

		const JoltLayerKey floorKey{ JoltBroadPhaseClass::Static, kProbeWorldCategory, kProbeCharacterCategory };

		JoltWorldConfig config;
		config.simulatableSlots = 2;
		config.slotTemplate = { capsule };
		config.staticLayers = { floorKey };
		config.ringDepthTicks = 0;
		config.ringSlotBytes = 4096;
		config.tempAllocatorBytes = 1u << 20;
		config.maxBodies = 16;
		config.maxBodyPairs = 64;
		config.maxContactConstraints = 64;

		JoltWorld world(runtime, config, nullptr);

		JPH::BodyCreationSettings floor(new JPH::BoxShape(JPH::Vec3(5.f, 5.f, 0.5f)), JPH::RVec3(0.f, 0.f, -0.5f),
			JPH::Quat::sIdentity(), JPH::EMotionType::Static, world.layers().layerOf(floorKey));
		const JPH::BodyID floorId = world.createStaticBody(floor);
		world.bodies().AddBody(floorId, JPH::EActivation::DontActivate);
		world.physics().OptimizeBroadPhase();

		BodySlotOccupancy occupancy;
		occupancy.occupied.set(0);
		occupancy.occupied.set(1);
		world.applyOccupancy(occupancy);
		world.bodies().SetPosition(world.slotBodyId(0, 0), JPH::RVec3(-0.40f, 0.f, 1.0f), JPH::EActivation::Activate);
		world.bodies().SetPosition(world.slotBodyId(1, 0), JPH::RVec3(0.40f, 0.05f, 1.3f), JPH::EActivation::Activate);

		for (uint32_t tick = 0; tick < kProbeTicks; ++tick)
		{
			world.step(kProbeDtSeconds);
		}
		return world.liveStateHash();
	}

	constexpr const char* kVersionFeatureNames[] = {
		"JPH_DOUBLE_PRECISION",
		"JPH_CROSS_PLATFORM_DETERMINISTIC",
		"JPH_FLOATING_POINT_EXCEPTIONS_ENABLED",
		"JPH_PROFILE_ENABLED",
		"JPH_EXTERNAL_PROFILE",
		"JPH_DEBUG_RENDERER",
		"JPH_DISABLE_TEMP_ALLOCATOR",
		"JPH_DISABLE_CUSTOM_ALLOCATOR",
		"JPH_OBJECT_LAYER_BITS=32",
		"JPH_ENABLE_ASSERTS",
		"JPH_OBJECT_STREAM"
	};
	constexpr uint32_t kVersionFeatureShift = 24;
	constexpr uint32_t kEnableAssertsFeatureBit = 9;

	void appendDifference(std::string& out, const char* text)
	{
		out += out.empty() ? "" : "; ";
		out += text;
	}
}

uint64_t JoltDeterminismFingerprint::value() const
{
	FingerprintHash hash;
	hash.text(engineName);
	hash.u64(versionMajor);
	hash.u64(versionMinor);
	hash.u64(versionPatch);
	hash.u64(joltVersionId);
	hash.u64(simdWidthBits);
	hash.text(instructionSet);
	hash.u64((fusedMultiplyAdd ? 1u : 0u) | (stepFlushToZero ? 2u : 0u) | (stepDenormalsAreZero ? 4u : 0u) | (stepRoundToNearest ? 8u : 0u));
	hash.u64(probeStateHash);
	return hash.value();
}

JoltDeterminismFingerprint determinismFingerprint(JoltRuntime& runtime)
{
	JoltDeterminismFingerprint fingerprint;
	fingerprint.engineName = "Jolt";
	fingerprint.versionMajor = JPH_VERSION_MAJOR;
	fingerprint.versionMinor = JPH_VERSION_MINOR;
	fingerprint.versionPatch = JPH_VERSION_PATCH;
	fingerprint.joltVersionId = JoltRuntime::libraryVersionId();

#if defined(JPH_USE_AVX512)
	fingerprint.simdWidthBits = 512;
	fingerprint.instructionSet = "AVX512";
#elif defined(JPH_USE_AVX2)
	fingerprint.simdWidthBits = 256;
	fingerprint.instructionSet = "AVX2";
#elif defined(JPH_USE_AVX)
	fingerprint.simdWidthBits = 256;
	fingerprint.instructionSet = "AVX";
#elif defined(JPH_USE_SSE4_2)
	fingerprint.simdWidthBits = 128;
	fingerprint.instructionSet = "SSE4.2";
#elif defined(JPH_USE_SSE4_1)
	fingerprint.simdWidthBits = 128;
	fingerprint.instructionSet = "SSE4.1";
#elif defined(JPH_USE_SSE)
	fingerprint.simdWidthBits = 128;
	fingerprint.instructionSet = "SSE2";
#elif defined(JPH_USE_NEON)
	fingerprint.simdWidthBits = 128;
	fingerprint.instructionSet = "NEON";
#else
	fingerprint.simdWidthBits = 0;
	fingerprint.instructionSet = "scalar";
#endif

#if defined(JPH_USE_FMADD)
	fingerprint.fusedMultiplyAdd = true;
#endif

	{
		JoltStepFpScope fpScope;
		const JoltFpMode mode = readJoltFpMode();
		fingerprint.stepFlushToZero = mode.flushToZero;
		fingerprint.stepDenormalsAreZero = mode.denormalsAreZero;
		fingerprint.stepRoundToNearest = mode.roundToNearest;
	}

	fingerprint.probeStateHash = runProbeSimulation(runtime);
	return fingerprint;
}

const char* joltBuildConfigurationOf(uint64_t joltVersionId)
{
	const bool assertsOn = ((joltVersionId >> (kVersionFeatureShift + kEnableAssertsFeatureBit)) & 1u) != 0u;
	return assertsOn
		? "Debug configuration (JPH_ENABLE_ASSERTS on: the build does not define NDEBUG)"
		: "Development/Shipping configuration (JPH_ENABLE_ASSERTS off)";
}

std::string describeFingerprintMismatch(const JoltDeterminismFingerprint& local, const JoltDeterminismFingerprint& remote)
{
	char line[256];
	if (local.value() == remote.value())
	{
		std::snprintf(line, sizeof(line), "Jolt determinism fingerprints match (0x%016llx)", static_cast<unsigned long long>(local.value()));
		return line;
	}

	std::string differences;
	if (std::strcmp(local.engineName, remote.engineName) != 0)
	{
		std::snprintf(line, sizeof(line), "physics engine %s vs %s", local.engineName, remote.engineName);
		appendDifference(differences, line);
	}
	if (local.versionMajor != remote.versionMajor || local.versionMinor != remote.versionMinor || local.versionPatch != remote.versionPatch)
	{
		std::snprintf(line, sizeof(line), "Jolt version %u.%u.%u vs %u.%u.%u", local.versionMajor, local.versionMinor, local.versionPatch,
			remote.versionMajor, remote.versionMinor, remote.versionPatch);
		appendDifference(differences, line);
	}
	const uint64_t featureDifference = (local.joltVersionId ^ remote.joltVersionId) >> kVersionFeatureShift;
	for (uint32_t bit = 0; bit < sizeof(kVersionFeatureNames) / sizeof(kVersionFeatureNames[0]); ++bit)
	{
		if (((featureDifference >> bit) & 1u) != 0u)
		{
			const bool localOn = ((local.joltVersionId >> (kVersionFeatureShift + bit)) & 1u) != 0u;
			std::snprintf(line, sizeof(line), "%s %s locally, %s remotely", kVersionFeatureNames[bit], localOn ? "on" : "off", localOn ? "off" : "on");
			appendDifference(differences, line);
		}
	}
	if (local.simdWidthBits != remote.simdWidthBits || std::strcmp(local.instructionSet, remote.instructionSet) != 0)
	{
		std::snprintf(line, sizeof(line), "SIMD %s/%u-bit vs %s/%u-bit", local.instructionSet, local.simdWidthBits, remote.instructionSet, remote.simdWidthBits);
		appendDifference(differences, line);
	}
	if (local.fusedMultiplyAdd != remote.fusedMultiplyAdd)
	{
		std::snprintf(line, sizeof(line), "fused multiply-add %s vs %s", local.fusedMultiplyAdd ? "on" : "off", remote.fusedMultiplyAdd ? "on" : "off");
		appendDifference(differences, line);
	}
	if (local.stepFlushToZero != remote.stepFlushToZero || local.stepDenormalsAreZero != remote.stepDenormalsAreZero
		|| local.stepRoundToNearest != remote.stepRoundToNearest)
	{
		std::snprintf(line, sizeof(line), "step FP mode FTZ/DAZ/nearest %d%d%d vs %d%d%d",
			local.stepFlushToZero, local.stepDenormalsAreZero, local.stepRoundToNearest,
			remote.stepFlushToZero, remote.stepDenormalsAreZero, remote.stepRoundToNearest);
		appendDifference(differences, line);
	}
	if (local.probeStateHash != remote.probeStateHash)
	{
		std::snprintf(line, sizeof(line), "probe simulation state hash 0x%016llx vs 0x%016llx",
			static_cast<unsigned long long>(local.probeStateHash), static_cast<unsigned long long>(remote.probeStateHash));
		appendDifference(differences, line);
	}

	std::string message = "Jolt determinism fingerprint mismatch: local ";
	std::snprintf(line, sizeof(line), "0x%016llx [%s]", static_cast<unsigned long long>(local.value()), joltBuildConfigurationOf(local.joltVersionId));
	message += line;
	message += " vs remote ";
	std::snprintf(line, sizeof(line), "0x%016llx [%s]", static_cast<unsigned long long>(remote.value()), joltBuildConfigurationOf(remote.joltVersionId));
	message += line;
	message += "; differs in: ";
	message += differences;
	return message;
}
