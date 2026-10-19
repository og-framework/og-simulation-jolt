<!-- SPDX-License-Identifier: MPL-2.0 -->
# `JoltUnits.h` — rationale

The one place og-simulation-jolt converts the og-simulation core's units into Jolt's.

**If this file and `JoltUnits.h` disagree, the header is authoritative and this file is stale.**

---

## §1 Why a conversion exists at all

og-simulation core is centimetres, Z-up, single-precision float. Jolt has no fixed length unit, but
its tuned defaults (convex radius, speculative contact distance, penetration slop, sleep thresholds)
assume metres. The backend therefore runs Jolt in metres and converts at its boundary.

`joltUnits::kMetresPerCentimetre` is `0.01f`, and `joltUnits::centimetresToMetres` multiplies by it,
for one float or for a glm vector into a Jolt vector. Axes are not remapped: Jolt has no up axis of
its own, so Z-up is kept by passing gravity along -Z and by rotating Y-axis shapes (Jolt's capsule) to
Z where they are built (`JoltWorld-rationale.md`, the slot bodies section).

## §2 Where it is used

- `JoltWorld`'s constructor converts `JoltWorldConfig::gravityCmPerS2` (default -980 cm/s² on Z) and
  `JoltWorldConfig::parkingPositionCm`.
- `JoltWorld.cpp` converts the template shapes' sizes (sphere radius, box half extents, capsule radius
  and height).

Nothing else in the library converts lengths. The body adapter (a later addition) owns the other
direction and the force, torque and inertia conversions.

## §3 What is deliberately not converted

`JoltWorld::stateHash` hashes Jolt's own metre values, not values converted back to centimetres:
multiplying by 100 can map two adjacent floats to the same float, which would hide a one-ULP
difference (`JoltWorld-rationale.md`, the state hash section).

## §4 Guards

**None.**

## §5 The planned move to metres, and the restore-skip tripwire

The simulations are expected to move to metres and to adopt Jolt's centre-of-mass convention (a body's
pose is its centre of mass, not its shape origin). Until then, a seam write is lossy: the body adapter
multiplies centimetres by 0.01 into metres, and Jolt adds the rotated centre-of-mass offset to the
position it stores (and subtracts it again on read). A value written into Jolt and captured back does
not always give the same bits.

The restore skip in `SimulationIntegrationExecutor::pushCorrectedBodyStatesAll` exists only because of
that loss. After a resim restores Jolt's own snapshot of the anchor, the push compares each declaration
with the restored body and skips the write when they are equal. Without the skip every resim moves the
bodies nobody corrected by an ULP, the next correction compares exactly and disagrees, and a
torque-driven body (the weapon) keeps the error and starts a resim storm. Once the write is exact the
skip changes nothing and is dead weight.

The test `JoltSuite.Tripwire.TheSeamWriteIsLossyUntilTheSimulationsUseMetres` (og-simulation-tests,
Jolt/JoltRollbackSuiteTest.cpp) says when that has happened. It writes 1,000 seeded states through
the body adapter directly (`setBodyPose`, then both velocity setters), on a slot body whose centre of
mass is offset from its origin, and counts the states whose capture differs in any bit. It prints the
count on every run:

`[JoltSuite] tripwire: N of 1000 seam writes changed bits (0 = remove the restore skip in pushCorrectedBodyStatesAll)`

and requires N > 0. Reading the count:

- **N stays near today's value** (793 at introduction): nothing changed.
- **N falls but does not reach 0**: metres landed but the centre-of-mass step still loses bits. Keep the
  skip.
- **N = 0, so the test fails**: the write is exact.

When it fails: remove the restore-skip comparison from `pushCorrectedBodyStatesAll` (keep the
`setBodyPose` pose path), then delete the test. To keep the skip on purpose instead, delete only the
test and record why in this section.

The test's failure message carries the same instructions, so they reach whoever sees it fail.
