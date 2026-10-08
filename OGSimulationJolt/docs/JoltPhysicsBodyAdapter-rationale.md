<!-- SPDX-License-Identifier: MPL-2.0 -->
# `JoltPhysicsBodyAdapter.h` / `JoltPhysicsBodyAdapter.cpp` — rationale

og-simulation's body seam (`PhysicsBodyAdapter`) implemented on a `JoltWorld`, plus the two things
every Jolt adapter shares: the seam-unit conversions (`joltSeamUnits`) and the bind table
(`JoltBodyBindTable`). Prohibitions are in `JoltPhysicsBodyAdapter-guards.md`.

**If this file and the sources disagree, the sources are authoritative and this file is stale.**

---

## §1 What it satisfies

`static_assert(PhysicsBodyAdapter<JoltPhysicsBodyAdapter>)` at the end of the header checks the concept
members: transform get/set, torque, angular and linear velocity set, acceleration, velocity change,
inertia and state capture. Every member is defined in the `.cpp` and exported: in the modular Editor
build the library is a DLL that exports no Jolt symbol, so a header-inline Jolt call made from another
module would not link.

Members outside the concept:

- `isBodyResolvable` — "this slot body is bound to a simulatable" (§5). Not in the concept; the UE host
  calls it on the body adapter, as it does on the Chaos body adapter today.
- `getBodyMass` — kg, what Jolt integrates (1 / inverse mass). Used by the force seam and by tests.
- `bodyIdOf` / `joltBodyIdOf` — the id mapping (§6).
- `world`, `bindTable` — for the factory and the reader adapter.

## §2 The seam's units (pinned)

The seam speaks the units of the Chaos body adapter (UE-sim's ChaosPhysicsBodyAdapter.h), which is
Unreal's centimetre system. Confirmed from that adapter:

| Quantity | Seam unit | Evidence in the Chaos adapter |
|---|---|---|
| position | cm | X / P pass through the UE↔glm conversion unscaled |
| linear velocity, velocity change | cm/s | V passes through unscaled |
| acceleration | cm/s² | multiplied by the body mass M() (kg) and handed to AddForce |
| force (internal) | kg·cm/s² | the AddForce argument above |
| torque | kg·cm²/s² | handed to AddTorque unscaled; the radial weapon computes it as angular acceleration × the inertia this seam reports (og-brawler's DAttackRadialSimulation.h, applyTorque) |
| inertia | kg·cm² | Chaos's world inertia, unscaled |
| angular velocity | rad/s | W passes through unscaled |
| rotation | unit quaternion | glm (w, x, y, z) |

Jolt runs in metres (`JoltUnits-rationale.md`). Every conversion happens in `joltSeamUnits`, in one
place:

- lengths, velocities and accelerations: `centimetreVectorToJolt` (×0.01, through
  `joltUnits::centimetresToMetres`) and `metreVectorToSeam` (×100);
- torque: `torqueToJolt` (×1e-4: kg·cm²/s² → N·m);
- inertia: `inertiaToJolt` (×1e-4) and `inertiaToSeam` (×1e4);
- rotation: `rotationToJolt` / `rotationToSeam` reorder glm (w, x, y, z) and Jolt (x, y, z, w);
- angular velocity: unchanged (rad/s on both sides).

Because torque and inertia are scaled by the same factor, Δω = τ / I · dt holds in seam units
(`JoltBodyAdapter.ATorqueInSeamUnitsYieldsTauOverInertiaTimesDt`).

The old UE-only adapter (Source/JoltPhysicsModule, retired after the parity gate) passed torque through
unconverted and reported inertia in kg·m². Both are bugs, not precedent: with the brawler's own
torque = a·I they cancel for the weapon, but any torque not built from this seam's inertia would be off
by 10⁴. Both were reproduced as poison arms against the tests while this adapter was built.

## §3 Force-seam semantics

Kept from the Chaos adapter and from the comments of the old Jolt adapter:

- **`addBodyAcceleration`** accumulates an acceleration for this step; the engine applies force =
  a × mass. The mass is read from the body (1 / inverse mass), never from the shape: the factory
  overrides the shape's mass (`JoltBodyDefaults-rationale.md` §3), so the shape's own mass properties
  are wrong for every slot body. Two bodies of different mass given the same acceleration gain the
  same velocity (`JoltBodyAdapter.AnAccelerationOverOneStepYieldsTheExpectedVelocityChange`).
- **`addBodyVelocityChange`** adds a velocity before the step's solve, the same as an impulse of
  dv × mass; it uses Jolt's add-linear-velocity, which needs no mass and so cannot disagree with Chaos's
  SetV(GetV() + dv).
- Both are then integrated with Jolt's damping, v ← (v + a·dt) · max(0, 1 − c·dt), which is the same
  formula and the same order as Chaos's integrate step (Chaos/Private/Chaos/PBDRigidsEvolutionGBF.cpp,
  Integrate: acceleration, impulses, then ether drag). c is `joltBodyDefaults::kLinearDamping`.
- **`setBodyTransform`** writes position and rotation and does not activate the body (slot bodies
  never sleep, `JoltWorld-rationale.md`). The rotation is normalised first: Jolt expects a unit
  quaternion, and a rotation extracted from a matrix is unit only to float precision.
- **`captureBodyState`** reads the body after the step. Jolt has no separate start-of-step and solved
  pose (Chaos's X/R versus P/Q), so the body's position after an update is the solved pose, the
  counterpart of what the Chaos adapter reads.
- Torque on a rotation-locked body changes nothing (Jolt's inverse inertia is zero there).

## §4 Inertia: what Jolt integrates

`getBodyInertiaTensor` returns the diagonal of the world-space inertia (the inverse of Jolt's
world-space inverse inertia, ×1e4). That is the inertia Jolt actually integrates with, so the weapon's
torque a·I produces exactly the angular acceleration a (`JoltBodyAdapter.TheRadialSpunByItsAuthoredTorqueFollowsTheAnalyticOmega`:
60 ticks of the left-to-left swing match the analytic ω to 1.6e-6 rad/s).

A **rotation-locked** body (the character capsule) is the exception: Jolt removes its rotational
degrees of freedom by zeroing its inverse inertia, so there is nothing to invert. Chaos locks rotation
with a joint and keeps reporting the body's inertia, so the adapter reports the parity inertia the
factory recorded at bind time (`JoltBodyBindTable::lockedRotationInertiaOf`). A body with only some
rotation axes locked has a singular inverse inertia; no M1 body has one, and `joltBodyInertiaOf` fails
an `OG_CHECK` on it.

The four read paths (transform, state, mass, inertia) are free functions (`joltBodyTransformOf`,
`joltBodyStateOf`, `joltBodyMassOf`, `joltBodyInertiaOf`) shared with the reader adapter, so the two
adapters cannot read differently.

## §5 The bind table and "resolvable"

`JoltBodyBindTable` has one entry per slot body: the binding (simulatable id, declaration index) or
nothing, plus the locked-rotation inertia recorded at bind time.

`isBodyResolvable` means **bound**: the factory has bound this fixed body to a simulatable. It never
means "occupied". The UE host's registration waits for every body of a character to be resolvable
before it adds the character to the simulation's storage, and occupancy starts only after that add, so
"occupied" would never become true (guard G-01). "Bound" is true from the frame after the factory runs.
A bound but parked body sits in the PARKED layer, which no query matches, so code that asks
"resolvable?" about a body a query just returned cannot see one (`JoltWorld-rationale.md`, occupancy).

Thread rules (the host's, recorded here because the table's layout depends on them):

- The **bindings** are game-thread data in M1: written by the factory's bind (which the host runs under
  the world mutex) and by `releaseSlot` (game thread, no mutex), read by `isBodyResolvable` (game
  thread).
- The **locked-rotation inertia** records are written at bind (under the world mutex) and read on the
  step thread by `getBodyInertiaTensor`.
- So `releaseSlot` clears the bindings only (guard G-02). The stale inertia record and the body's stale
  user data are harmless: a vacated slot is parked, and the next bind overwrites both.

`indexOf` maps a `BodyId` to the table index (§6) and returns nothing for id 0 and for any body after
the slot range (statics).

## §6 Id mapping

A `BodyId` holds the Jolt body id's index-and-sequence number. Slot bodies are created with explicit
ids from 1 and sequence 0 (`JoltWorld-rationale.md`, the fixed body set), so `BodyId::value` = 1 +
slot × bodiesPerSlot + template index, and the table index is `value - 1`. The adapter's members fail an
`OG_CHECK` on a non-slot body and return the default value (identity transform, zero state) with checks
compiled out, like the Chaos adapter's unresolved-body path.

## §7 Body user data

`joltBodyUserData::encode` packs a binding into Jolt's 64-bit body user data: bit 63 = bound, bits 8-39
= simulatable id, bits 0-7 = declaration index. `decode` returns nothing when bit 63 is clear (Jolt's
default user data is 0). `static_assert`s pin the round trip at the field limits. User data is body
configuration, not simulation state: Jolt's snapshot does not save it, so it is the same in every
restored tick.

## §8 Guards

- **G-01** — `isBodyResolvable` means bound, never occupied.
- **G-02** — `releaseSlot` clears the bindings and nothing else.
