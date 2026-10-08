<!-- SPDX-License-Identifier: MPL-2.0 -->
# `JoltConfigSidecar.h` — rationale

The configuration that a Jolt snapshot does not contain but that must roll back with it.

**If this file and `JoltConfigSidecar.h` disagree, the header is authoritative and this file is
stale.**

---

## §1 Why a sidecar

Jolt's saved state holds only what a step changes (positions, rotations, velocities, forces, sleep
data, the contact cache). It does not hold body configuration: object layer, gravity factor, motion
type, materials. Two pieces of configuration in this backend change during play and must therefore
travel with each saved tick:

- **occupancy** (`BodySlotOccupancy`, from og-simulation core): which character slots are live. It
  decides each slot body's layer, gravity factor and, for a vacant slot, zero velocity;
- **shape-enable bits** (`ShapeEnableBits`): which registered shapes queries may find.

`ConfigSidecar` holds both. Each ring slot stores one next to its snapshot bytes
(`JoltStateRing-rationale.md`); `JoltWorld::restoreTick` re-applies it after the bytes
(`JoltWorld-rationale.md`).

## §2 `ShapeEnableTable`

One bit per registered shape, indexed by `ShapeId::value`, `kShapeEnableCapacity` (256) bits in all.
Every bit starts **enabled**, matching an engine shape that is query-enabled when created.
`enable`, `disable` and `isEnabled` take a `ShapeId`; `bits` and `restore` move the whole set in and out
of a sidecar. A `ShapeId` at or beyond the capacity fails an `OG_CHECK` (with checks compiled out it is
clamped to the last bit, never out of bounds).

The table only stores the bits. The spatial query adapter (a later addition) hands out the shape ids,
calls `enable`/`disable` and reads `isEnabled` in its query filter. Because the bits restore with the
tick, the first query after a restore sees the restored flags
(`JoltWorld.ShapeEnableBitsRestoreWithTheTick`).

256 covers 8 slots × 6 bodies × one shape each (48) with room for multi-shape bodies.

## §3 Guards

**None.**
