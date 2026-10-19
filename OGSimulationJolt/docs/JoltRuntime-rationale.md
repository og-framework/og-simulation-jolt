<!-- SPDX-License-Identifier: MPL-2.0 -->
# `JoltRuntime.h` / `JoltRuntime.cpp` — rationale

The narrative behind `JoltRuntime`, the reference-counted owner of Jolt's process-global state. The
sources keep their licence, a docs pointer and code; everything that explains them lives here.

**If this file and the sources disagree, the sources are authoritative and this file is stale.**

<!-- lint-external-ref: Jolt/Core/Core.h -- vendored Jolt v5.6.0 under ThirdParty/, which the lint does not index -->
<!-- lint-external-ref: Shape.h -- vendored Jolt v5.6.0 under ThirdParty/, which the lint does not index -->
<!-- lint-external-ref: Jolt/Core/IssueReporting.h -- vendored Jolt v5.6.0 under ThirdParty/, which the lint does not index -->

---

## §1 What it owns

Jolt keeps process-global state that must exist before any `JPH::PhysicsSystem` is built and must
outlive the last one:

- the allocator hooks (`JPH::RegisterDefaultAllocator`, which installs the `malloc`-based defaults),
- the type factory (`JPH::Factory::sInstance`) and the registered types (`JPH::RegisterTypes`),
- the trace hook (`JPH::Trace`) and, when `JPH_ENABLE_ASSERTS` is defined, the assert hook
  (`JPH::AssertFailed`).

`JoltRuntime` sets these up on the **first** `acquire` and tears them down on the **last** `release`
(`JPH::UnregisterTypes`, then the factory is deleted). In between it only counts. A host with two
worlds alive at once (two play-in-editor worlds, a client and a listen server in one process) acquires
twice and releases twice; the second world neither re-initialises Jolt nor has it torn down under it.
After the count returns to 0 a later `acquire` initialises Jolt again (covered by
`JoltRuntime.TwoAcquiresNeedTwoReleases`).

`acquire` returns a reference to the one `JoltRuntime` object. It carries no state; it exists so that a
later constructor can take a `JoltRuntime&` as proof that the caller acquired the runtime.

`release` without a matching `acquire` fails the `OG_CHECK`; with checks compiled out it is ignored
(the count never goes below 0). The first `acquire` checks that `JPH::Factory::sInstance` is unset, so
code outside `JoltRuntime` that initialised Jolt itself is caught instead of being overwritten.

`acquire`, `release` and `referenceCount` take one mutex, so they may be called from any thread. The logger
pointer is an atomic because Jolt may trace from whatever thread is running a physics job.

## §2 The injected logger

The logger is a **plain function pointer**, `void (*)(const char*)`, not a `std::function`. A capturing
lambda therefore does not compile. That is deliberate: Jolt's hooks are process-global, so the logger
installed by the first world stays installed while a second world is alive and can outlive the first
world. A logger that captured per-world state would dangle; a function pointer cannot hold any.

Only the **first** `acquire` (count 0 → 1) installs its logger. Later acquirers' loggers are ignored
until the count returns to 0. `release` at count 0 restores the trace and assert hooks that were
installed before the first `acquire` (Jolt's defaults, unless the host had replaced them), and clears
the logger, so nothing is routed to it after shutdown (covered by
`JoltRuntime.TraceReachesTheInjectedLoggerUntilRelease`).

Messages:

- every Jolt `Trace` line, formatted into a 1024-byte buffer and passed on unchanged;
- with `JPH_ENABLE_ASSERTS` defined, a failed Jolt assertion as
  `[Warning] Jolt assert failed: <expression> (<message>) at <file>:<line>`; the hook then returns
  `true`, which makes Jolt break into the debugger (`JPH_BREAKPOINT`);
- `JoltRuntime: Jolt <major>.<minor>.<patch> initialised (JPH_VERSION_ID 0x…)` on initialisation and
  `JoltRuntime: Jolt shut down` on shutdown;
- the version-mismatch line in §3, sent to the caller's logger.

The `[Warning]` prefix follows og-simulation's logging convention: a host's log emitter picks a
non-default verbosity from it.

## §3 The `JPH_VERSION_ID` agreement check

Jolt folds its ABI-relevant defines into `JPH_VERSION_ID` (`Jolt/Core/Core.h`, the
`JPH_VERSION_FEATURE_BIT_*` block). `JPH::RegisterTypes` is an inline function that passes the calling
translation unit's `JPH_VERSION_ID` to the library, and the library aborts on a mismatch. But only the
translation unit that calls `RegisterTypes` is checked, and that is `JoltRuntime.cpp`, inside the
library. A module that includes Jolt headers with a different define set (for example a private
`JPH_DEBUG_RENDERER`, which changes `JPH::Shape`'s virtual table) would not be caught.

So `acquire` is an inline function defined in the header, outside the exported symbols: it is
compiled in the **caller's** translation unit and passes the caller's `JPH_VERSION_ID` to
`acquireForVersion`, which compares it with the library's (`JPH::VerifyJoltVersionIDInternal`). On a
mismatch it logs both values to the caller's logger, fails the `OG_CHECK`, and calls `std::abort`, so
the failure is fatal in every configuration, including those that compile checks out.

`acquire` is deliberately not marked `OGSIMULATIONJOLT_API`. In a modular Unreal build, a call to an
imported inline function could run the library's own copy and compare the library with itself.

The `using JPH::uint64;` before each use of `JPH_VERSION_ID` is needed because the macro expands to an
unqualified `uint64(…)` cast. Unreal happens to declare a global `uint64`; a standalone build does not.

The test translation unit `Jolt/JoltVersionAgreementTest.cpp` in og-simulation-tests repeats the check
from outside the library and calls `JPH::Shape` virtuals on a shape the library constructed, among them
`GetVolume`, which `Shape.h` declares after its debug-renderer block, so a shifted virtual table shows up
as a wrong value.

## §4 Asserts

`JPH_ENABLE_ASSERTS` is not defined by the Unreal module. Jolt defines it itself when `JPH_DEBUG` is set
(`Jolt/Core/IssueReporting.h`), and `JPH_DEBUG` is set when `NDEBUG` is not (`Jolt/Core/Core.h`).
UnrealBuildTool defines `NDEBUG=1` for every build that does not use the debug C runtime, so Unreal
builds normally run without Jolt asserts, and the assert hook in `JoltRuntime.cpp` is compiled out.

Asserts are left off on purpose: `JPH_ENABLE_ASSERTS` is one of the `JPH_VERSION_ID` feature bits, so
turning it on for development builds only would give development and shipping binaries different
version ids. The standalone CMake build gets asserts in its Debug configuration (no `NDEBUG`) or with
`-DOGJOLT_ENABLE_ASSERTS=ON`, which is where the assert hook is exercised.
