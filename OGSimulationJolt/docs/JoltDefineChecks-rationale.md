<!-- SPDX-License-Identifier: MPL-2.0 -->
# `JoltDefineChecks.h` — rationale

`JoltDefineChecks.h` turns every forbidden Jolt configuration into a compile error. `JoltRuntime.h`
includes it, so every translation unit of this library, and every translation unit of a consumer that
uses `JoltRuntime`, is checked. Vendored Jolt sources do not include it (they are not edited), so the
checks fire in the library's own `JoltRuntime.cpp` whenever a forbidden define reaches the whole build.

**If this file and the header disagree, the header is authoritative and this file is stale.**

Line references below are to the vendored Jolt v5.6.0.

<!-- lint-external-ref: Jolt/Core/Core.h -- vendored Jolt v5.6.0 under ThirdParty/, which the lint does not index -->
<!-- lint-external-ref: Core.h -- vendored Jolt v5.6.0 under ThirdParty/, which the lint does not index -->
<!-- lint-external-ref: Jolt/Physics/Collision/Shape/Shape.h -- vendored Jolt v5.6.0 under ThirdParty/, which the lint does not index -->
<!-- lint-external-ref: Jolt/Core/Profiler.h -- vendored Jolt v5.6.0 under ThirdParty/, which the lint does not index -->
<!-- lint-external-ref: Jolt/Compute/ComputeSystem.h -- vendored Jolt v5.6.0 under ThirdParty/, which the lint does not index -->
<!-- lint-external-ref: Jolt/Physics/Collision/ObjectLayer.h -- vendored Jolt v5.6.0 under ThirdParty/, which the lint does not index -->
<!-- lint-external-ref: ThirdParty/JoltPhysics/VERSION.md -- in-repo file under ThirdParty/, not indexed -->

---

## §1 Why a define set must be identical everywhere

Jolt's headers change class layouts and virtual tables with its defines, and Jolt folds the
ABI-relevant ones into `JPH_VERSION_ID` (`Jolt/Core/Core.h`, the `JPH_VERSION_FEATURE_BIT_*` block,
lines 13–71). Two translation units that disagree on one of them disagree on the layout of the types
they share, which the linker does not detect. So every `JPH_*` define is set **publicly** by the build
(the Unreal module's `PublicDefinitions`, CMake's `PUBLIC` compile definitions) and never privately,
and `JoltRuntime::acquire` checks the caller's `JPH_VERSION_ID` at run time
(`JoltRuntime-rationale.md` §3).

`JPH_VERSION_ID` is also meant to be part of the determinism fingerprint two peers compare. A define that
differs between build configurations (editor and shipping, say) would then make those builds refuse each
other.

## §2 The checks

**`JPH_DEBUG_RENDERER`.** Jolt tests it with `#ifdef` (`Core.h` line 38; `Jolt/Physics/Collision/Shape/Shape.h`
lines 286–304), so `JPH_DEBUG_RENDERER=0` turns it **on**. When on, `JPH::Shape` gains the `Draw*`
virtuals in the middle of its virtual table, and feature bit 6 of `JPH_VERSION_ID` flips. It is never
enabled, in any configuration: debug drawing is to come from the host, not from Jolt. The predecessor module
`JoltPhysicsModule` defined `JPH_DEBUG_RENDERER=0` and `JPH_PROFILE_ENABLED=0` as *private*
definitions, which turned both on inside that module and off everywhere else.

**`JPH_PROFILE_ENABLED`.** Also tested with `#ifdef` (`Core.h` line 28; `Jolt/Core/Profiler.h` line 86 tests it with `defined`), so `=0`
turns it on; it flips feature bit 4. Never defined.

**`JPH_DOUBLE_PRECISION`.** Makes `JPH::RVec3` positions double precision and flips feature bit 1. The
og-simulation core is float throughout, so the library is single precision only.

**`JPH_SHARED_LIBRARY`.** Makes `JPH_EXPORT` a DLL export or import (`Core.h` lines 256–287). Jolt is
compiled into the `OGSimulationJolt` Unreal module, or into the `og_jolt_physics` static library in
CMake, never as a separate shared library.

**`JPH_USE_DX12`, `JPH_USE_VK`, `JPH_USE_MTL`, `JPH_USE_CPU_COMPUTE`.** The GPU-compute backends are not
vendored (`ThirdParty/JoltPhysics/VERSION.md`), so with any of these defined the declarations they enable
in `Jolt/Compute/ComputeSystem.h` have no definitions to link against.

**`sizeof(JPH::ObjectLayer) == 2`.** `JPH_OBJECT_LAYER_BITS` is 16 (`Jolt/Physics/Collision/ObjectLayer.h`
lines 12–20 make `ObjectLayer` a `uint16` at 16 and a `uint32` at 32; 16 is also Jolt's default when the
define is unset). The size is asserted rather than the macro, because the size is what the layouts
depend on.

## §3 What is not checked here

- **`JPH_ENABLE_ASSERTS`** is allowed: the standalone CMake build turns it on in Debug. Unreal builds
  leave it off (`JoltRuntime-rationale.md` §4).
- **`JPH_CROSS_PLATFORM_DETERMINISTIC`** and the floating-point compiler settings are a separate,
  later decision; they are not set today.
- **SSE4.1/4.2** (`JPH_USE_SSE4_1`, `JPH_USE_SSE4_2`) are set publicly on x86-64 only. On ARM64 Jolt
  selects NEON itself (`Core.h` lines 120–126).
