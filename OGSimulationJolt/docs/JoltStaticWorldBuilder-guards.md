<!-- SPDX-License-Identifier: MPL-2.0 -->
# `JoltStaticWorldBuilder.h` / `JoltStaticWorldBuilder.cpp` — guards

Every prohibition that governs a line of `JoltStaticWorldBuilder.cpp`. Each entry has an **opaque, stable
id**. In the source a single line `// ⛔G-nn` sits exactly where the forbidden edit would be typed.

**If this file and the sources disagree, the sources are authoritative and this file is stale.** Fix
this file; do not soften the sources to match it.

⛔ **An id is never reused.** A guard that is deleted, or that becomes a compile-time check, is moved to
a "Retired" section and its number is spent forever.

⛔ **Nothing in this file is a rationale.** The reasons live in `JoltStaticWorldBuilder-rationale.md`.

---

## G-01 — Do not build statics in description order, and do not drop or reorder the sort's tie-breaks

**Tag site:** `JoltStaticWorldBuilder.cpp`, in `JoltStaticWorldBuilder::build`, on the sort of the
elements, before the duplicate-key count and the group loop.

**The prohibition.** Do not remove the sort, replace it with a sort on fewer fields, or reorder its
fields. The order is: layer key, then surface (friction and restitution bits), then `stableKey`, then
the content hash. Do not make the comparison depend on anything that differs between peers, such as a
pointer, the element's position in the description, or a value the host does not send.

**The consequence.** The sorted order fixes which elements share a chunk, the order of the children in
each chunk's compound shape, and the order in which the static bodies are created, which fixes their
body ids. Contacts against statics record the static body id and the child's sub-shape id, and they are
part of every saved tick. Two peers whose hosts list the same statics in a different order would build
different chunks and different ids, so their snapshots and their physics would differ. Witnessed on
2026-10-07: with the `stableKey` and content-hash fields removed, all three shuffled descriptions in
`JoltStaticWorldBuilder.ShuffledDescriptionGivesIdenticalStateAfter300TicksOfRestingCapsules` ended 300
ticks with a different `stateHash` and different bytes. The content hash breaks ties between elements
that share a `stableKey`; without it, duplicates keep their description order.

**What breaks if it moves.** The grouping loop that follows assumes the elements of one body (same
layer key and surface) are adjacent. A sort that does not lead with the layer key and the surface
splits a group into several groups.

## G-02 — Do not reorder a triangle's indices

**Tag site:** `JoltStaticWorldBuilder.cpp`, in the triangle-mesh arm of the element shape builder, on
the line that appends each triangle.

**The prohibition.** Pass each triangle's three indices to Jolt in the order the description gives
them. Do not swap the second and third index to "convert handedness".

**The consequence.** The description's winding is the host's physics winding: the first host importer
copies the triangles of the host's cooked physics mesh, and that engine's triangle normal is
(B − A) × (C − A), the same front face Jolt uses. Jolt ignores the back faces of mesh triangles in
contact generation and in default ray casts, so a swapped winding turns every mesh in the arena inside
out: capsules fall through floors from above and collide from below (the contact half is read
from Jolt's contact code, which keeps the default back-face mode; not measured). Witnessed on 2026-10-07: with the
indices swapped, the mesh checks in `JoltStaticWorldBuilder.EachShapeTypeIsQueryableAtTheExpectedPlace`
failed (no hit from above, a hit from below).

**What breaks if it moves.** Nothing: the tag stays on the line that appends the triangle. A mirrored
host transform is the host's job (the importer swaps the indices itself when the scale is mirrored),
and a mirrored `localToWorld` reaches Jolt as a negative scale on a scaled shape, which Jolt accepts for meshes and hulls (not
tested here).
