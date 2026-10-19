<!-- SPDX-License-Identifier: MPL-2.0 -->
# `JoltFpEnvironment.h` / `JoltFpEnvironment.cpp` — rationale

The floating-point mode every Jolt step runs under, and how it is read back.

**If this file and the sources disagree, the sources are authoritative and this file is stale.**

---

## §1 Why the step pins the FP mode

The floating-point control state is per thread. In this game a step can run on a task-graph worker
(client), inline on the game thread (dedicated server) and later on a dedicated simulation thread.
If those threads differ in how they treat denormals, the same inputs give different results. So
`JoltWorld::step` sets the mode itself for the duration of the step and checks it.

## §2 `JoltStepFpScope`

Two RAII members, restored in reverse order when the scope ends:

1. Jolt's FPFlushDenormals (vendored Jolt, Core/FPFlushDenormals.h). On x86-64 it sets only the MXCSR
   **flush-to-zero** bit (results that would be denormal become zero). On ARM64 it sets FPCR.FZ,
   which flushes both denormal results and denormal inputs.
2. `JoltDenormalsAreZero`: on x86-64, Jolt's own FPControlWord template set to the MXCSR
   **denormals-are-zero** bit (denormal inputs read as zero); elsewhere an empty struct.

With both, x86-64 treats denormals the way ARM64's FZ does, so the two architectures agree on this
point. Only the bits each member sets are restored on exit; the caller's mode is unchanged
(`JoltWorld.StepLeavesTheCallersFpModeUnchanged`). That test compares the MXCSR **control** bits only:
the low six bits are sticky exception flags that the step legitimately sets (measured 2026-10-07: the
invalid-operation flag, 0x1fa4 before a step and 0x1fa5 after, on Win64 Development).

## §3 `readJoltFpMode` and the check in `step`

`readJoltFpMode` reads MXCSR on x86-64 (flush-to-zero, denormals-are-zero, round-to-nearest) and FPCR
on ARM64 with GCC-style compilers (FZ, which covers both denormal flags, and RMode = nearest). On
other targets `readable` is false. `isExpectedJoltStepFpMode` is true when the mode is unreadable or
all three hold.

`JoltWorld::step` checks `isExpectedJoltStepFpMode` on the result of `readJoltFpMode` with an `OG_CHECK` right after
entering the scope, so every platform and every thread that steps the world is checked in builds that
keep checks; the read is not compiled in where checks are compiled out. Witnessed 2026-10-07: with the
denormals-are-zero member removed, the check fails in the Win64 Development test runner with the
message "JoltWorld::step - the FP environment is not FTZ + DAZ + round-to-nearest inside the step
scope". The ARM64 read compiles in the Android arm64 client build (2026-10-07, "Result: Succeeded");
it has not been run on a device.

`determinismFingerprint` records the three flags as read inside the same scope
(`JoltDeterminismFingerprint-rationale.md`).

## §4 Guards

**None.** The `OG_CHECK` enforces the mode wherever checks are on.
