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
