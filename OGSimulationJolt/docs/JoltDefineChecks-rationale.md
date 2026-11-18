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
<!-- lint-external-ref: Jolt/Math/Math.h -- vendored Jolt v5.6.0 under ThirdParty/, which the lint does not index -->
<!-- lint-external-ref: Jolt/Math/Vec3.inl -- vendored Jolt v5.6.0 under ThirdParty/, which the lint does not index -->
<!-- lint-external-ref: Jolt/Jolt.cmake -- vendored Jolt v5.6.0 under ThirdParty/, which the lint does not index -->
<!-- lint-external-ref: DifferenceOfProducts -- vendored Jolt v5.6.0 under ThirdParty/, which the lint does not index -->
<!-- lint-external-ref: Vec3::ReduceSum -- vendored Jolt v5.6.0 under ThirdParty/, which the lint does not index -->
<!-- lint-external-ref: Vec3::Dot -- vendored Jolt v5.6.0 under ThirdParty/, which the lint does not index -->

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
JoltPhysicsModule (historical: deleted after the parity gate) defined `JPH_DEBUG_RENDERER=0` and `JPH_PROFILE_ENABLED=0` as *private*
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

**`JPH_CROSS_PLATFORM_DETERMINISTIC` must be defined** (og-input-latency task 21). It is the one Jolt define
this file requires instead of forbidding. `OGSimulationJolt.Build.cs` adds it to `PublicDefinitions` on every
platform, and CMake sets it `PUBLIC` on `og_jolt_physics`, so every translation unit that sees Jolt's headers
sees it. Jolt tests it with `#ifdef`. In Jolt v5.6.0 it:

- sets feature bit 2 of `JPH_VERSION_ID` (`Core.h` lines 18–22), so a module compiled without it fails
  `JoltRuntime::acquire`'s version check, and a build without it has a different determinism fingerprint
  (`JoltDeterminismFingerprint-rationale.md`);
- keeps Jolt's automatic `JPH_USE_FMADD` off (`Core.h` lines 133–137 for ARM, 175–187 for x86). On arm64,
  where `__ARM_FEATURE_FMA` is always defined, Jolt would otherwise compute `DifferenceOfProducts`
  (`Jolt/Math/Math.h`) with `std::fma`, a single rounding where x64 rounds twice;
- negates as `0 - x` instead of flipping the sign bit in the NEON, RISC-V and scalar paths (`Vec3::operator -`,
  `Vec4::operator -`, `DVec3::operator -`), which is what the SSE path already does;
- makes `Vec3::ReduceSum`, which `Vec3::Dot` uses, add `+0` instead of the vector's unused fourth lane on
  SSE4.1 (`Jolt/Math/Vec3.inl` lines 898–929), so `-0` sums as on NEON.

On x64 with SSE4 the last item is the only arithmetic change: it differs only in the sign of a zero dot
product. Jolt's own CMake pairs the option with not enabling fused multiply-add (`Jolt/Jolt.cmake` lines
859–862, 963 and 1023); the floating-point compiler mode it also needs is og-simulation's
(`FloatingPointMode-rationale.md`).

**`JPH_USE_FMADD` must not be defined.** Jolt's detection only adds it when it is absent, so a build that
defines it explicitly would turn fused multiply-add back on despite the define above.

**`sizeof(JPH::ObjectLayer) == 2`.** `JPH_OBJECT_LAYER_BITS` is 16 (`Jolt/Physics/Collision/ObjectLayer.h`
lines 12–20 make `ObjectLayer` a `uint16` at 16 and a `uint32` at 32; 16 is also Jolt's default when the
define is unset). The size is asserted rather than the macro, because the size is what the layouts
depend on.

## §3 What is not checked here

- **`JPH_ENABLE_ASSERTS`** is allowed: the standalone CMake build turns it on in Debug. Unreal builds
  leave it off (`JoltRuntime-rationale.md` §4).
- **The floating-point compiler mode** (`/fp:precise`, `-ffp-contract=off`) is set by the build files, not
  checked here (og-simulation's `FloatingPointMode-rationale.md`).
- **SSE4.1/4.2** (`JPH_USE_SSE4_1`, `JPH_USE_SSE4_2`) are set publicly on x86-64 only. On ARM64 Jolt
  selects NEON itself (`Core.h` lines 120–126).
