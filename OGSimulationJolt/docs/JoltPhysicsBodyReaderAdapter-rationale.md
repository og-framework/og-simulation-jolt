<!-- SPDX-License-Identifier: MPL-2.0 -->
# `JoltPhysicsBodyReaderAdapter.h` / `JoltPhysicsBodyReaderAdapter.cpp` — rationale

og-simulation's read-only body seam (`PhysicsBodyReaderAdapter`) on a `JoltWorld`.

**If this file and the sources disagree, the sources are authoritative and this file is stale.**

---

## §1 What it satisfies

`static_assert(PhysicsBodyReaderAdapter<JoltPhysicsBodyReaderAdapter>)` at the end of the header:
transform, inertia, state capture and `isBodyResolvable`. `getBodyMass` (kg) is extra, for tests and
diagnostics.

It is built from a `const JoltWorld&` and the body adapter's `const JoltBodyBindTable&`, so it reads
exactly what the body adapter writes and binds.

## §2 Same reads as the body adapter

Every read goes through the free functions the body adapter uses (`joltBodyTransformOf`,
`joltBodyStateOf`, `joltBodyMassOf`, `joltBodyInertiaOf`; `JoltPhysicsBodyAdapter-rationale.md` §2-§4),
so units, quaternion order and the locked-rotation inertia rule are identical. The reader's inertia is
therefore in kg·cm² and equals the Chaos value for every brawler body
(`JoltPhysicsFactory.EveryBrawlerDeclarationBindsAChaosParityBody`).

## §3 `isBodyResolvable` means bound

As on the body adapter: the body is bound to a simulatable (`JoltBodyBindTable::isBound`), never "the
slot is occupied". An unbound body reads as the Chaos reader's unresolved body does: identity transform,
zero inertia, default state, zero mass.

## §4 Threads

Jolt's body interface is not internally locked here (`JoltWorld::bodies` is the no-lock interface), so
the reader is a step-thread object unless the host serialises it with the step (the M1 world mutex).
That is a change from the old UE-only Jolt reader, which assumed a locked interface and could be called
from anywhere. `isBodyResolvable` reads the bindings, which are game-thread data in M1
(`JoltPhysicsBodyAdapter-rationale.md` §5).

## §5 Guards

**None.**
