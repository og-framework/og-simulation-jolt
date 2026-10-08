<!-- SPDX-License-Identifier: MPL-2.0 -->
# `JoltPhysicsBodyAdapter.h` / `JoltPhysicsBodyAdapter.cpp` — guards

Every prohibition that governs a line of `JoltPhysicsBodyAdapter.h` or `JoltPhysicsBodyAdapter.cpp`.
Each entry has an **opaque, stable id**. In the source a single line `// ⛔G-nn` sits exactly where the
forbidden edit would be typed.

**If this file and the sources disagree, the sources are authoritative and this file is stale.** Fix
this file; do not soften the sources to match it.

⛔ **An id is never reused.** A guard that is deleted, or that becomes a compile-time check, is moved to
a "Retired" section and its number is spent forever.

⛔ **Nothing in this file is a rationale.** The reasons live in `JoltPhysicsBodyAdapter-rationale.md`.

---

## G-01 — `isBodyResolvable` means "bound", never "occupied"

**Tag site:** `JoltPhysicsBodyAdapter.h`, on the line directly above the inline `isBodyResolvable`
member of `JoltPhysicsBodyAdapter`.

**The prohibition.** Do not make the body adapter's `isBodyResolvable` test slot occupancy (the
world's applied occupancy, or anything that becomes true only once the character is in the
simulation). It answers one question: has the factory bound this fixed body to a simulatable
(`JoltBodyBindTable::isBound`).

**The consequence.** The UE host's registration polls this member for every body of a character and
adds the character to the simulation's storage only once all of them answer true; a slot becomes
occupied only after that add. An "occupied" test therefore never turns true, and no character ever
finishes registering: a silent deadlock, not a crash. (Witnessed as a poison arm while this file was
written: `JoltBodyAdapter.ResolvableMeansBoundNeverOccupied` failed.)

**What breaks if it moves.** Nothing as long as the tag stays on the member's declaration. If the
member moves to the `.cpp`, the tag moves with its definition.

## G-02 — `releaseSlot` clears the bindings and nothing else

**Tag site:** `JoltPhysicsBodyAdapter.cpp`, in `JoltBodyBindTable::releaseSlot`, directly above the
loop that resets the slot's bindings.

**The prohibition.** Do not make `releaseSlot` touch a Jolt body (user data, layer, velocity, material,
mass), the world, or the locked-rotation inertia records. It resets the slot's bindings only.

**The consequence.** The host calls `releaseSlot` on the game thread when a character leaves, without
the world mutex, while the step may be running on another thread. A Jolt body write there races the
step (and changes simulated state outside the per-tick occupancy that the snapshot ring rolls back); a
write to the inertia records races the step's `getBodyInertiaTensor` reads. The stale values are
harmless because a vacated slot is parked and the next bind overwrites them.

**What breaks if it moves.** If the bindings and the inertia records are ever merged into one entry,
this guard must say which fields `releaseSlot` may clear; until then the tag stays on the reset loop.
