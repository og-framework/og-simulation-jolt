<!-- SPDX-License-Identifier: MPL-2.0 -->
# `JoltPhysicsBodyAdapter.h` / `JoltPhysicsBodyAdapter.cpp` — rationale

og-simulation's body seam (`PhysicsBodyAdapter`) implemented on a `JoltWorld`, plus the two things
every Jolt adapter shares: the seam's glm-to-Jolt type conversions (`joltSeam`) and the bind table
(`JoltBodyBindTable`). Prohibitions are in `JoltPhysicsBodyAdapter-guards.md`. The binding the table
stores (`JoltBodyBinding`) and the body user data helpers (`joltBodyUserData`) are in
`JoltBodyUserData.h`, which this header includes (`JoltBodyUserData-rationale.md`).

**If this file and the sources disagree, the sources are authoritative and this file is stale.**

---

## §1 What it satisfies

`static_assert(PhysicsBodyAdapter<JoltPhysicsBodyAdapter>)` at the end of the header checks the concept
members: transform get/set, torque, angular and linear velocity set, acceleration, velocity change,
inertia and state capture. Every member is defined in the `.cpp` and exported: in the modular Editor
build the library is a DLL that exports no Jolt symbol, so a header-inline Jolt call made from another
module would not link.

Members outside the concept:

- `setBodyPose` — writes a pose from a position and a quaternion (§3). It models og-simulation's optional
  `PhysicsBodyPoseWriter` capability, which is checked by a second `static_assert` beside the first: a
  signature slip then fails the build instead of making the restore push fall back to the matrix path.
- `isBodyResolvable` — "this slot body is bound to a simulatable" (§5). Not in the concept; the UE host
  calls it on the body adapter, as it does on the Chaos body adapter today.
- `getBodyMass` — kg, what Jolt integrates (1 / inverse mass). Used by the force seam and by tests.
- `bodyIdOf` / `joltBodyIdOf` — the id mapping (§6).
- `world`, `bindTable` — for the factory and the reader adapter.

## §2 The seam's units (pinned)

The seam speaks og-simulation's units (`SimulationFrame.h`): metres, kilograms and seconds, in
og-simulation's axes. Jolt runs in the same units, so nothing is scaled at this seam:

| Quantity | Seam unit | Jolt |
|---|---|---|
| position | m | m |
| linear velocity, velocity change | m/s | m/s |
| acceleration | m/s² | multiplied by the body mass (kg) and handed to AddForce, so N |
| torque | kg·m²/s² (N·m) | N·m |
| inertia | kg·m² | kg·m² |
| angular velocity | rad/s | rad/s |
| rotation | unit quaternion, glm (w, x, y, z) | unit quaternion, Jolt (x, y, z, w) |

The Chaos body adapter converts to Unreal's centimetres and axes at its own edge (UE-sim's
ChaosPhysicsBodyAdapter.h); this adapter has no edge to convert.

The conversions in `joltSeam` are type-only: `toJolt` and `toGlm` copy the three components of a vector,
and `rotationToJolt` / `rotationToSeam` reorder a quaternion between glm's (w, x, y, z) and Jolt's
(x, y, z, w). Each carries every bit both ways (`JoltBodyAdapter.VectorAndQuaternionRoundTrips`). Until the
simulations moved to metres the seam scaled lengths by 0.01 into Jolt and by 100 back, and torque and
inertia by 1e-4; flip U made every factor 1, and Stage C deleted them.

Because torque and inertia are in the same units on both sides, Δω = τ / I · dt holds in seam units
(`JoltBodyAdapter.ATorqueInSeamUnitsYieldsTauOverInertiaTimesDt`).

The old UE-only adapter (Source/JoltPhysicsModule; historical: deleted after the parity gate) passed torque through
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
- **`setBodyPose`** writes position and rotation with no matrix in between: the position is copied
  like every vector (`joltSeam::toJolt`), the quaternion only reordered (`rotationToJolt`), and the
  body is not activated. It does **not** normalise. The restore push
  (`SimulationIntegrationExecutor::pushCorrectedBodyStatesAll`) hands it either an adopted rotation or the
  body's own captured one, and either is a capture of a rotation Jolt already held as a unit quaternion;
  Jolt's set-position-and-rotation stores the quaternion as given. So a pose written this way captures
  back with the same quaternion bits (`JoltSuite.SetBodyPoseWritesTheQuaternionBitForBitAndTheMatrixPathDoesNot`:
  0 of 1,000 random unit quaternions change). A normalise would re-round about a third of them (318 of
  1,000 when tried as a poison), and the matrix path (`setBodyTransform`) changes most of them (930 of
  1,000). The position is still not always exact: Jolt's centre-of-mass offset can change it by an ULP
  (§9).
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

The packing of a binding and the root mark into Jolt's body user data moved, with the helpers, to
`JoltBodyUserData.h`; its text is `JoltBodyUserData-rationale.md` §3.

## §8 Guards

- **G-01** — `isBodyResolvable` means bound, never occupied.
- **G-02** — `releaseSlot` clears the bindings and nothing else.

## §9 The restore skip and its tripwire (moved from the retired JoltUnits rationale, §5)

A seam write is not always exact. Jolt stores a body's centre of mass, not its origin: `setBodyPose`
hands Jolt the origin, Jolt adds the rotated centre-of-mass offset to the position it stores, and a capture
subtracts it again. For a body whose centre of mass is offset from its origin, a value written into Jolt and
captured back does not always keep its bits. Until the simulations moved to metres the write also
multiplied centimetres by 0.01 into metres; since flip U the seam carries metres unconverted (§2).

The restore skip in `SimulationIntegrationExecutor::pushCorrectedBodyStatesAll` exists because of that loss.
After a resim restores Jolt's own snapshot of the anchor, the push compares each declaration with the
restored body and skips the write when they are equal. Without the skip every resim moves the bodies nobody
corrected by an ULP, the next correction compares exactly and disagrees, and a torque-driven body (the
weapon) keeps the error and starts a resim storm.

The test `JoltSuite.Tripwire.TheSeamWriteIsLossyUntilTheSimulationsUseMetres` (og-simulation-tests,
Jolt/JoltRollbackSuiteTest.cpp) writes 1,000 seeded states through the body adapter directly
(`setBodyPose`, then both velocity setters), on a slot body whose centre of mass is offset from its origin,
counts the states whose capture differs in any bit, prints the count on every run:

`[JoltSuite] tripwire: N of 1000 seam writes changed bits (0 = remove the restore skip in pushCorrectedBodyStatesAll)`

and requires N > 0. N was 793 at its introduction, in centimetres, and is 984 in metres (measured
2026-10-26 at the metre, Y-up knobs): the move to metres did not make the write exact, because the
centre-of-mass step still loses bits.

The skip is kept by the 2026-10-25 ruling (Q3 of og-sim-metres-yup), which also retires this tripwire: it
measures an artificial offset centre of mass, and no shipped body has one.
