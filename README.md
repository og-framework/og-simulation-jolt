<!-- SPDX-License-Identifier: MPL-2.0 -->
# og-simulation-jolt

The Jolt Physics backend for [og-simulation](https://github.com/og-framework/og-simulation): an
engine-free, rollback-capable physics world. It depends on og-simulation (C++20, glm) and on a vendored
copy of [Jolt Physics](https://github.com/jrouwe/JoltPhysics), and on nothing else: no game engine, no
engine types.

## Status

Scaffold. The library currently holds `JoltRuntime` (the reference-counted global Jolt initialisation
and shutdown, with Jolt's trace and assert output routed to an injected logger) and the build. The
physics world, the body adapters and the query adapter follow.

## Position in the og-framework graph

```
og-simulation         (engine-free simulation core)
    ↓
og-simulation-jolt    (this repo: Jolt backend, engine-free)
    ↓ consumed by
og-simulation-ue      (UE plugin shell: the `OGSimulationJolt` module is a Build.cs only)
og-simulation-tests   (Catch2 tests, `Jolt/`)
```

In the og-simulation-ue plugin this repository sits at `Source/OGSimulationJolt/og-simulation-jolt`.

## Layout

| Path | What |
|---|---|
| `OGSimulationJolt/` | The library sources (`#include "OGSimulationJolt/JoltRuntime.h"`) and its `CMakeLists.txt` |
| `OGSimulationJolt/docs/` | Rationale for each source file |
| `ThirdParty/JoltPhysics/` | Vendored Jolt (`Jolt/`, its MIT `LICENSE`) and `VERSION.md` with the release tag and commit |
| `CMakeLists.txt` | Standalone CMake entry point |
| `LICENSES/` | Licence texts: `MPL-2.0.txt` (this library), `MIT.txt` (Jolt) |

## Jolt version and configuration

- Jolt **v5.6.0** (see `ThirdParty/JoltPhysics/VERSION.md` for the commit and what is vendored).
- **Single precision only.** `JPH_DOUBLE_PRECISION` is never defined, matching og-simulation's float core.
- `JPH_OBJECT_LAYER_BITS=16`. SSE4.1/4.2 on x86-64; NEON on ARM64 (Jolt detects it itself).
- Jolt is compiled without exceptions and without RTTI.
- Never defined: `JPH_DEBUG_RENDERER`, `JPH_PROFILE_ENABLED`, `JPH_DOUBLE_PRECISION`, `JPH_SHARED_LIBRARY`,
  and the GPU-compute options (`JPH_USE_DX12`, `JPH_USE_VK`, `JPH_USE_MTL`, `JPH_USE_CPU_COMPUTE`).
  `OGSimulationJolt/JoltDefineChecks.h` turns each of these into a compile error.

## Building

**CMake, standalone:** point `OG_SIMULATION_DIR` at the root of an og-simulation checkout (the directory
holding `OGSimulation/` and `glm/`).

```bash
cmake -S . -B build -DOG_SIMULATION_DIR=/path/to/og-simulation
cmake --build build --config Release
```

This builds two static libraries: `og_jolt_physics` (Jolt) and `og_simulation_jolt` (this library, which
links `og_simulation` and `og_jolt_physics`). `-DOGJOLT_ENABLE_ASSERTS=ON` defines `JPH_ENABLE_ASSERTS`.

**CMake, from a parent build** that already defines the `og_simulation` target (for example
[og-tests-cmake-runner](https://github.com/og-framework/og-tests-cmake-runner)):

```cmake
add_subdirectory(extern/og-simulation-jolt/OGSimulationJolt)
target_link_libraries(MyTarget PRIVATE og_simulation_jolt)
```

The parent defines `OG_STANDALONE_BUILD`, as it does for og-simulation.

**Unreal Engine:** through the `OGSimulationJolt` module of the
[og-simulation-ue](https://github.com/og-framework/og-simulation-ue) plugin.
`OGSimulationJolt/OGSimulationJolt.cpp` is that module's `IMPLEMENT_MODULE` stub, the only Unreal code in
this repository; the CMake build leaves it out.

## Usage

```cpp
#include "OGSimulationJolt/JoltRuntime.h"

void logLine(const char* message);           // your logger: a plain function, no captures

JoltRuntime& jolt = JoltRuntime::acquire(&logLine);   // first acquire initialises Jolt
// ... create and step JPH::PhysicsSystem instances ...
JoltRuntime::release();                               // last release shuts Jolt down
```

Every `acquire` needs one `release`. Two worlds (for example two play-in-editor worlds) acquire twice and
release twice.

## License and contributing

Mozilla Public License 2.0, see [LICENSE](LICENSE). The vendored Jolt is MIT, see
[ThirdParty/JoltPhysics/LICENSE](ThirdParty/JoltPhysics/LICENSE).

Contributions are welcome under MPL-2.0 (inbound = outbound). See [CONTRIBUTING.md](CONTRIBUTING.md).
