<!-- SPDX-License-Identifier: MPL-2.0 -->
# `JoltSpatialQueryAdapter.h` / `JoltSpatialQueryAdapter.cpp` — guards

Every prohibition that governs a line of `JoltSpatialQueryAdapter.cpp`. Each entry has an **opaque,
stable id**. In the source a single line `// ⛔G-nn` sits exactly where the forbidden edit would be typed.

**If this file and the sources disagree, the sources are authoritative and this file is stale.** Fix
this file; do not soften the sources to match it.

⛔ **An id is never reused.** A guard that is deleted, or that becomes a compile-time check, is moved to
a "Retired" section and its number is spent forever.

⛔ **Nothing in this file is a rationale.** The reasons and the evidence live in
`JoltSpatialQueryAdapter-rationale.md`.

---

## G-01 — `overlap` reports the LAST volume's hits only; never the union of every volume

**Tag site:** `JoltSpatialQueryAdapter.cpp`, in `JoltSpatialQueryAdapter::overlap`, directly above the
statement that picks the last element of `volumeIds`.

**The prohibition.** Do not query every volume of the list and concatenate (or merge) the hits, and do
not pick the first volume. `overlap` answers with the hits of the last volume in the list.

**The consequence.** That is what the Chaos adapter does: its engine call resets the output array on
every call, so of a multi-volume list only the last call's hits survive (rationale §4). The brawler's
radial attack registers two co-centred spheres (radius r and 2r) and passes both. A concatenation reports
every target inside the inner sphere twice, and the order-dependent hit loop then merges or routes the
duplicate as a second hit: the Jolt build stops matching the Chaos build. A deduplicated union happens to
equal the outer sphere for that one caller and differs for any two volumes that do not nest.
(Witnessed as a poison arm: `JoltSpatialQuery.OverlapReportsOnlyTheLastVolumeAsTheChaosAdapterDoes`
failed.)

**What breaks if it moves.** Nothing while the tag stays on the volume selection. If the user rules that
the union is the intended semantics, this guard is retired together with the matching change in the
Chaos adapter.

## G-02 — the query pose is the volume's TRANSLATION only; never apply its rotation

**Tag site:** `JoltSpatialQueryAdapter.cpp`, in the anonymous-namespace `queryPoseOf`, directly above the
statement that builds the pose.

**The prohibition.** Do not build the query shape's pose from the rotation (or scale) of the parent
transform, of the volume's `offsetTransform`, or of a sweep's `transform`. The pose is a pure translation
(plus the shape's centre of mass).

**The consequence.** The Chaos adapter passes the identity rotation and only the translation of
the parent transform times the offset (the sweep transform times the offset for a sweep) to the engine. Every box and capsule volume is
therefore axis-aligned in Chaos, and the ground probe's capsule stays upright however the character is
turned. Applying the rotation turns boxes and capsules on this backend only: hit sets and sweep fractions
differ from Chaos whenever a parent transform carries a yaw. (Witnessed as a poison arm:
`JoltSpatialQuery.TheQueryPoseIgnoresTheVolumeRotationAsTheChaosAdapterDoes` failed.)

**What breaks if it moves.** Nothing while every query goes through `queryPoseOf`.

## G-03 — `registerShape` re-enables the shape it registers

**Tag site:** `JoltSpatialQueryAdapter.cpp`, in `JoltSpatialQueryAdapter::registerShape`, directly above
the statement that enables the shape's bit in the world's `ShapeEnableTable`.

**The prohibition.** Do not delete the enable, and do not make it conditional on the bit's current
value.

**The consequence.** Shape ids are fixed per slot body, so a slot that a new character joins reuses the
previous occupant's ids and bits. A Chaos join creates a new component, whose shape is query-enabled.
Without the enable, the newcomer inherits whatever the previous occupant left (a lowered guard stays
disabled), and a peer that never saw the previous occupant (a late joiner) holds the default, enabled
bit: the peers' queries disagree about the same character. (Witnessed as a poison arm:
`JoltSpatialQuery.DisabledShapesDoNotHit` failed.)

**What breaks if it moves.** Nothing while the enable stays in `registerShape`.

## G-04 — readiness is the body's user data, never `JoltBodyBindTable`

**Tag site:** `JoltSpatialQueryAdapter.cpp`, in `JoltQueryBodyFilter::ShouldCollideLocked`, directly
above the statement that decodes the body's user data.

**The prohibition.** Do not replace the user-data decode with a read of the body adapter's bind table
(`JoltBodyBindTable::isBound` / `bindingOf`), and do not drop it.

**The consequence.** The bind table's bindings are game-thread data in M1: `releaseSlot` writes them on
the game thread without the world mutex, and the query runs on the step thread. Reading them here is a
data race. The user data is written by the factory's bind under the world mutex and is never cleared, and
a vacated slot is excluded by its PARKED layer. Dropping the decode lets an occupied but unbound slot body
(occupancy applied before the bind) through the filter. (Witnessed as a poison arm together with the
matching decode in `keyOf`: `JoltSpatialQuery.ParkedAndUnboundSlotBodiesAreNeverReturned` failed. The
`keyOf` decode alone is a second barrier, so a poison of this line alone stays green.)

**What breaks if it moves.** Nothing while the readiness test stays in the body filter.

## G-05 — a volume with no mapped search category is a TRACE-CHANNEL query, not a miss

**Tag site:** `JoltSpatialQueryAdapter.cpp`, in the anonymous-namespace `QueryLayerFilters::selected`,
directly above the statement that chooses the object filter or the trace filter.

**The prohibition.** Do not make a volume whose mapped search mask is empty match nothing, and do not
send it through the object filter. It answers on its `traceCategory` with the trace filter.

**The consequence.** The Chaos adapter stores an empty object-type set for such a volume, and UE builds
a trace-channel query whenever the object-type set is empty (rationale §5). It is the documented mode of a
volume with empty `searchCategories`, and it is also what a volume whose every search bit is unmapped
does. A "matches nothing" rewrite makes those volumes silently blind on this backend only. (Witnessed as
a poison arm: `JoltSpatialQuery.AnEmptySearchIsATraceChannelQuery` failed.)

**What breaks if it moves.** Nothing while both `collideAt` and `castFrom` take their layer filter from
`QueryLayerFilters::selected`.

## G-06 — hits are ordered by the peer-stable key; never by Jolt body id

**Tag site:** `JoltSpatialQueryAdapter.cpp`, in the anonymous-namespace `peerStableOrderOf`, directly
above the statement that builds the ordering tuple.

**The prohibition.** Do not put the Jolt body id (or anything derived from the slot) ahead of, or in
place of, `JoltQueryHitKey`, and do not drop the ordering.

**The consequence.** Body ids are slot-local: two peers that assigned the same characters to different
slots hold different ids for the same body. The game's hit loop is order-dependent, so an id-ordered
report resolves the same overlap differently on each peer. The key (owner simulatable id, declaration
index, shape index, then the static body and element) is the same on every peer. (Witnessed as a poison
arm: `JoltSpatialQuery.HitsAreSortedByAPeerStableKeyUnderADifferentSlotAssignment` failed.)

**What breaks if it moves.** Nothing while both comparators go through `peerStableOrderOf`.

## G-07 — a sweep collides with back faces

**Tag site:** `JoltSpatialQueryAdapter.cpp`, in `JoltSpatialQueryAdapter::castFrom`, directly above the
statement that sets the cast's back-face mode.

**The prohibition.** Do not restore Jolt's default back-face mode (IgnoreBackFaces) for the cast.

**The consequence.** With the default, Jolt drops a hit on a convex target the sweep starts inside
whenever the sweep direction points against the contact normal, i.e. out of the target (it counts as a
back-face hit). The Chaos adapter's sweep sets bFindInitialOverlaps and reports every initial overlap
at fraction 0, whatever the direction. Under the default, a sweep that starts inside the floor and moves
up returns a miss on this backend only, where Chaos returns `blocked` at fraction 0 (measured: the
poison arm below). Today's only sweep caller, the ground probe, sweeps down into its support; by Jolt's
code (a greater-or-equal-zero dot-product test in the convex cast) that hit is kept even under the default, so the probe
is not affected in that case, and every sweep moving out of a body it starts in is. (Witnessed as a
poison arm:
`JoltSpatialQuery.ASweepThatStartsInsideATargetReportsItAtFractionZero` failed.)

**What breaks if it moves.** Nothing while the tag stays on the back-face setting.

## G-08 — a slot hit's `objectPosition` is the ROOT body's position, not the hit body's

**Tag site:** `JoltSpatialQueryAdapter.cpp`, in `JoltSpatialQueryAdapter::hitOf`, directly above the
statement that reads the position for a slot-body hit.

**The prohibition.** Do not report the position of the body that was hit (the weapon, guard or projectile
body) as `objectPosition`.

**The consequence.** The Chaos adapter reports the hit's actor location, and every shape the Chaos
factory creates belongs to the character actor, so a hit on a guard or weapon reports the character's
(capsule's) location. The game's hit detection measures the hit direction from `objectPosition`; the
hit body's position turns that direction and the Jolt build stops matching. (Witnessed as a poison arm:
`JoltSpatialQuery.OverlapHitAndMiss` failed.)

**What breaks if it moves.** Nothing while the tag stays on the slot-hit branch.

---

## Retired

None.
