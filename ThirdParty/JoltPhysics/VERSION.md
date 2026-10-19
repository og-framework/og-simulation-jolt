<!-- SPDX-License-Identifier: MPL-2.0 -->
# Vendored Jolt Physics

| | |
|---|---|
| Upstream | https://github.com/jrouwe/JoltPhysics |
| Release tag | `v5.6.0` (GitHub release "5.6.0", published 2026-07-11; the newest release on 2026-10-07, when it was vendored) |
| Commit | `e77f175595e64cb44218cc9d9d56fc365ad0e36a` |
| Source | the release archive `https://github.com/jrouwe/JoltPhysics/archive/refs/tags/v5.6.0.tar.gz` (SHA-256 `6e069ee0172478cc78182047aac87e5310ba14a67a53348ae14cc37801fd3f8e`) |
| Licence | MIT, see [`LICENSE`](LICENSE) (upstream's file, unmodified) |

## What is vendored

`Jolt/` and `LICENSE` from the archive, byte for byte, **minus the GPU-compute backends**:

- `Jolt/Compute/CPU/`, `Jolt/Compute/DX12/`, `Jolt/Compute/MTL/`, `Jolt/Compute/VK/` (46 files)
- `Jolt/Shaders/*.cpp` and `Jolt/Shaders/*.hlsl` (2 + 17 files)

Upstream compiles those only when `JPH_USE_CPU_COMPUTE`, `JPH_USE_DX12`, `JPH_USE_VK` or `JPH_USE_MTL` is
set (`Jolt/Jolt.cmake`, the `if (JPH_USE_...)` blocks). This library sets none of them, and the Unreal
build compiles every `.cpp` under the module directory, so the files are left out instead of being compiled
as empty translation units. `Jolt/Compute/*.h` and `Jolt/Compute/ComputeSystem.cpp` stay (upstream's base
source list holds them, and `Jolt/Physics/Hair/HairSettings.h` includes `ComputeBuffer.h` and `ComputeSystem.h` unconditionally), and so do the
`Jolt/Shaders/*.h` headers (`HairSettings.h` and `HairShaders.cpp` include `Jolt/Shaders/HairStructs.h`; the
other headers are kept so the header set matches upstream).

Nothing else from the archive is vendored (`Build/`, `Docs/`, `Samples/`, `UnitTests/`, `HelloWorld/`, …).
No vendored file is edited.

## Upgrading

1. Download the new release archive from the upstream releases page (never a branch head).
2. Replace `Jolt/` and `LICENSE` with the archive's, then delete the same backend files listed above
   (re-check `Jolt/Jolt.cmake` for new optional backends).
3. Update this file: tag, commit (`git ls-remote --tags https://github.com/jrouwe/JoltPhysics.git`),
   archive hash, date.
4. Re-check the define list against the new `Jolt/Core/Core.h` (`JPH_VERSION_FEATURE_BIT_*`) and the
   checks in `OGSimulationJolt/JoltDefineChecks.h`.
