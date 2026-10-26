<!-- SPDX-License-Identifier: MPL-2.0 -->
# `JoltDeterminismFingerprint.h` / `JoltDeterminismFingerprint.cpp` — rationale

A value two peers can compare before a match to find out whether their Jolt builds can simulate in
lockstep, and a message that says why when they cannot.

**If this file and the sources disagree, the sources are authoritative and this file is stale.**

---

## §1 What it contains

`determinismFingerprint(runtime)` fills a `JoltDeterminismFingerprint`:

| field | source |
|---|---|
| `engineName`, `versionMajor`, `versionMinor`, `versionPatch` | "Jolt" and Jolt's version macros |
| `joltVersionId` | `JoltRuntime::libraryVersionId`: Jolt's version id, which folds the version and the feature bits (double precision, cross-platform determinism, FP exceptions, profiler, debug renderer, asserts, …) into one number |
| `simdWidthBits`, `instructionSet` | the widest instruction set the library was compiled for (AVX-512, AVX2, AVX, SSE4.2, SSE4.1, SSE2, NEON or scalar) |
| `fusedMultiplyAdd` | whether Jolt uses fused multiply-add (a different rounding) |
| `stepFlushToZero`, `stepDenormalsAreZero`, `stepRoundToNearest` | the FP mode read inside the step's FP scope (`JoltFpEnvironment-rationale.md`) |
| `probeStateHash` | `JoltWorld::liveStateHash` after a fixed probe simulation (§2) |

`value()` folds every field into one 64-bit FNV-1a hash. Two peers compare `value()`; when the values
differ, `describeFingerprintMismatch(local, remote)` names each field that differs.

The fields that are compile-time facts describe the library as compiled, which is the code that steps
the world. Wiring the value into the host's build-label check is the host's job.

## §2 The probe simulation

Two capsule slots (42 cm radius, 96 cm total half height, rotation locked, gravity on) on a static box,
placed so that they overlap each other slightly and fall onto the box, stepped 60 times at 1/60 s,
through a real `JoltWorld` with depth-0 ring. It exercises gravity, capsule-box and capsule-capsule
contacts and the solver; two builds that produce different results for this scene produce different
hashes.

The probe's numbers are og-simulation metres in og-simulation's axes (+Y up, -Z forward): the capsule's
0.42 m radius and 0.96 m total half height, a floor box of half extents (5, 0.5, 5) m centred at
(0, -0.5, -0) so its top is at height 0, and the two slots at (0, 1, 0.4) and (0.05, 1.3, -0.4) m. Each is
the literal the centimetre wrapper it was first written with returned in metres and Y-up, bit for bit (the
floor centre keeps the -0 the wrapper placed). The world's default gravity (`JoltWorld-rationale.md` §2) and the capsule shape
(`JoltFrame-rationale.md`) follow the same frame.

Stable across runs of the same binary: `JoltDeterminismFingerprint.StableAcrossTwoRunsOfTheSameBinary`.

Pinned on x64: `JoltDeterminismFingerprint.PinnedValueWin64` checks that the value is
`0xe54d2bc380288c01`. It is compiled only where `_M_X64` is defined (64-bit MSVC); other architectures
are not pinned. A change to the probe, to Jolt or to its build flags that moves the value fails it, and
an intended change re-pins it in the same change.

The pinned value has changed once, at the axis flip (2026-10-26). In Unreal's axes it was
`0x835b9656d68aa407`, in centimetres and again in metres: flipping the units did not move it. Since the
simulation's axis frame became the right-handed Y-up one it is `0xe54d2bc380288c01`. The scene, the world's gravity and the
capsule now lie along Jolt's Y axis, and the capsule is a plain capsule instead of a rotated-translated
shape.

## §3 The cross-platform-determinism toggle

`JoltDeterminismFingerprint.ChangesWhenCrossPlatformDeterminismIsToggled` uses a second test
translation unit that defines JPH_CROSS_PLATFORM_DETERMINISTIC before including Jolt and returns the
version id it sees. That id differs from the library's in exactly the cross-platform-determinism
feature bit, and a fingerprint carrying it has a different `value()` and a mismatch message that names
the define. The library itself is not rebuilt with the define: the test proves the fingerprint is
sensitive to the bit, not that the toggled build simulates differently.

## §4 Debug builds have a different fingerprint, on purpose

A build that does not define NDEBUG (Unreal's Debug configurations; the standalone CMake Debug build)
makes Jolt define its debug flag, which turns on Jolt's asserts, and the asserts are one of the feature
bits in the version id. So a Debug build's fingerprint differs from a Development or Shipping build's
even with identical sources. That is correct: asserts change what the code does (a failed assert breaks
into the debugger, and Jolt's update asserts on dropped contacts). It is also the most likely mismatch a
developer will meet.

So the mismatch message names the configuration of each side, decoded from that feature bit by
`joltBuildConfigurationOf`: "Debug configuration (JPH_ENABLE_ASSERTS on: the build does not define
NDEBUG)" or "Development/Shipping configuration (JPH_ENABLE_ASSERTS off)", and lists the define with its
local and remote state. A refusal at join between a Debug and a Development build therefore reads as
exactly that (`JoltDeterminismFingerprint.MismatchMessageNamesTheBuildConfiguration`).

## §5 Guards

**None.**
