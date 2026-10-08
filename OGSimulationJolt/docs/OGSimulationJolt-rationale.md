<!-- SPDX-License-Identifier: MPL-2.0 -->
# The `OGSimulationJolt` library and its builds — rationale

How og-simulation-jolt is built: the standalone CMake build, the Unreal module shell that wraps it in
the og-simulation-ue plugin, the Unreal module stub and the export macro. It is the docs target of
`CMakeLists.txt`, `OGSimulationJolt/CMakeLists.txt`, `OGSimulationJolt/OGSimulationJolt.cpp`,
`OGSimulationJolt/OGJoltExport.h` and the plugin's `OGSimulationJolt.Build.cs`.

**If this file and those build files disagree, the build files are authoritative and this file is
stale.**

<!-- lint-external-ref: Jolt/Core/Core.h -- vendored Jolt v5.6.0 under ThirdParty/, which the lint does not index -->
<!-- lint-external-ref: Jolt/Math/Math.h -- vendored Jolt v5.6.0 under ThirdParty/, which the lint does not index -->
<!-- lint-external-ref: JPH::Allocate -- free function of vendored Jolt v5.6.0 under ThirdParty/, which the lint does not index -->
<!-- lint-external-ref: JPH::AlignedAllocate -- free function of vendored Jolt v5.6.0 under ThirdParty/, which the lint does not index -->

The defines themselves (which are set, which are forbidden, and why) are in
`JoltDefineChecks-rationale.md`.

---

## §1 Engine-free, with one stub

The library uses the C++ standard library, og-simulation (and glm through it) and the vendored Jolt,
and nothing else. The one exception is `OGSimulationJolt/OGSimulationJolt.cpp`, the Unreal module's
`IMPLEMENT_MODULE` stub: Unreal needs it to load the module, and it is the only Unreal include in the
repository. `OGSimulationJolt/CMakeLists.txt` removes it from the CMake build. og-simulation uses the
same arrangement for its own module stub.

`OGJoltExport.h` defines `OGSIMULATIONJOLT_API` as empty when `OG_STANDALONE_BUILD` is defined (the
CMake build). In Unreal, UnrealBuildTool defines it for the module (a DLL export or import in modular
builds, empty in monolithic ones).

## §2 CMake

- `OGSimulationJolt/CMakeLists.txt` defines two static libraries. `og_jolt_physics` is every `.cpp`
  under `ThirdParty/JoltPhysics/Jolt/`, with the Jolt defines as `PUBLIC` compile definitions, so every
  consumer sees the same set. `og_simulation_jolt` is this library; it links `og_simulation` and
  `og_jolt_physics` publicly. It expects the parent to provide `og_simulation` and to define
  `OG_STANDALONE_BUILD`, the same contract og-simulation's own `CMakeLists.txt` has.
- The root `CMakeLists.txt` is the standalone entry point. When no `og_simulation` target exists it
  adds og-simulation's `glm/` and `OGSimulation/` from `OG_SIMULATION_DIR`, a path to an og-simulation
  checkout. og-simulation is never found through a nested submodule of this repository.
- Exceptions and RTTI are disabled for the Jolt sources only (`PRIVATE` options on `og_jolt_physics`):
  Jolt uses neither (its own CMake turns both off by default, `CPP_EXCEPTIONS_ENABLED` and
  `CPP_RTTI_ENABLED`), and a consumer such as a Catch2 test executable may need exceptions.
  On MSVC the compiler prints a harmless `D9025` warning ("overriding '/EHs' with '/EHs-'") once per
  compiler invocation (twice for a full Jolt build, measured 2026-10-07), because CMake's default flags
  hold `/EHsc`.
- On x86-64 the Jolt sources get `JPH_USE_SSE4_1` and `JPH_USE_SSE4_2`; with gcc or clang also
  `-msse4.1 -msse4.2 -mpopcnt`, because the SSE4.2 path calls `_mm_popcnt_u32`
  (`Jolt/Math/Math.h`). MSVC needs no flag for these intrinsics.
- `OGJOLT_ENABLE_ASSERTS` (default `OFF`) defines `JPH_ENABLE_ASSERTS` publicly.

## §3 The Unreal module shell (`OGSimulationJolt.Build.cs`, og-simulation-ue)

The module directory is `Source/OGSimulationJolt/` of the plugin, and this repository sits under it at
`og-simulation-jolt/`. UnrealBuildTool compiles every `.cpp` below the module directory, so the module
compiles the library, the stub and all vendored Jolt sources; nothing is listed by hand.

- **Dependencies:** `Core`, `glm` and `OGSimulation`, all public. Nothing else.
- **Include paths, public:** the repository root (for `#include "OGSimulationJolt/…"`) and
  `ThirdParty/JoltPhysics` (for `#include <Jolt/…>`).
- **Defines, all public:** `JPH_OBJECT_LAYER_BITS=16`; `JPH_USE_SSE4_1=1` and `JPH_USE_SSE4_2=1` on
  Win64 and Linux only. ARM64 targets (Android) get NEON from Jolt's own detection.
- **`bEnableExceptions = false`, `bUseRTTI = false`:** as in §2.
- **`bUseUnity = false`:** carried over from the predecessor `JoltPhysicsModule`, which compiled Jolt
  non-unity; a unity build of Jolt has not been tried.
- **`IWYUSupport = IWYUSupport.None`:** without it UnrealBuildTool's first-include check reports
  "Expected X.h to be first header included" for each of the 138 vendored Jolt `.cpp` files (measured
  2026-10-07 with Unreal Engine 5.6.1). The vendored files are not edited to satisfy it.
- **`PCHUsage = UseExplicitOrSharedPCHs`:** as for the `OGSimulation` module. `OG_CHECK`'s Unreal branch
  (`OGSimulation/OGAssert.h`) uses `checkf`, which reaches `JoltRuntime.cpp` through the shared
  precompiled header.

## §4 The DLL boundary in modular Unreal builds

In a modular build (the editor) `OGSimulationJolt` is its own DLL. Jolt's own symbols are **not**
exported from it: `JPH_EXPORT` is empty unless `JPH_SHARED_LIBRARY` is defined (`Jolt/Core/Core.h`
lines 256–287), and that define is forbidden. So another module may include Jolt headers and use Jolt
types, inline functions and virtual calls, but a call to a non-inline Jolt function, or a use of a Jolt
global such as `JPH::Trace` or `JPH::Factory::sInstance` (Jolt's `operator new` overrides go through
the global allocator hooks `JPH::Allocate` and `JPH::AlignedAllocate`), links only in monolithic builds (the test executable, cooked game
builds). Code outside this module reaches Jolt through functions of this library marked
`OGSIMULATIONJOLT_API`.
