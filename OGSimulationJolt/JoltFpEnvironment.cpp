// SPDX-License-Identifier: MPL-2.0
// docs/JoltFpEnvironment-rationale.md

#include "OGSimulationJolt/JoltFpEnvironment.h"

JoltFpMode readJoltFpMode()
{
	JoltFpMode mode;
#if defined(JPH_CPU_X86)
	const unsigned int mxcsr = _mm_getcsr();
	mode.readable = true;
	mode.rawControlWord = mxcsr;
	mode.flushToZero = (mxcsr & _MM_FLUSH_ZERO_MASK) == _MM_FLUSH_ZERO_ON;
	mode.denormalsAreZero = (mxcsr & _MM_DENORMALS_ZERO_MASK) == _MM_DENORMALS_ZERO_ON;
	mode.roundToNearest = (mxcsr & _MM_ROUND_MASK) == _MM_ROUND_NEAREST;
#elif defined(JPH_CPU_ARM) && JPH_CPU_ARCH_BITS == 64 && !defined(JPH_COMPILER_MSVC)
	uint64_t fpcr = 0;
	asm volatile("mrs %0, fpcr" : "=r"(fpcr));
	constexpr uint64_t kFlushToZero = uint64_t(1) << 24;
	constexpr uint64_t kRoundingModeMask = uint64_t(3) << 22;
	mode.readable = true;
	mode.rawControlWord = fpcr;
	mode.flushToZero = (fpcr & kFlushToZero) != 0u;
	mode.denormalsAreZero = mode.flushToZero;
	mode.roundToNearest = (fpcr & kRoundingModeMask) == 0u;
#endif
	return mode;
}

bool isExpectedJoltStepFpMode(const JoltFpMode& mode)
{
	return !mode.readable || (mode.flushToZero && mode.denormalsAreZero && mode.roundToNearest);
}
