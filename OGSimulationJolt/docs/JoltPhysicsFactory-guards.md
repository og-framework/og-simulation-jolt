<!-- SPDX-License-Identifier: MPL-2.0 -->
# `JoltPhysicsFactory.h` / `JoltPhysicsFactory.cpp` — guards

Every prohibition that governs a line of `JoltPhysicsFactory.cpp`. Each entry has an **opaque, stable
id**. In the source a single line `// ⛔G-nn` sits exactly where the forbidden edit would be typed.

**If this file and the sources disagree, the sources are authoritative and this file is stale.** Fix
this file; do not soften the sources to match it.

⛔ **An id is never reused.** A guard that is deleted, or that becomes a compile-time check, is moved to
a "Retired" section and its number is spent forever.

⛔ **Nothing in this file is a rationale.** The reasons live in `JoltPhysicsFactory-rationale.md`.

---

## G-01 — the root body takes `kAdoptedRootMaterial`, never the created-body material

**Tag site:** `JoltPhysicsFactory.cpp`, in `JoltPhysicsFactory::applyBodyDefaults`, directly above the
statement that picks the material from the descriptor's `isRoot`.

**The prohibition.** Do not give every slot body one material, and do not give the root (the
`isRoot` declaration, the character capsule) `kCreatedBodyMaterial`. The root takes
`joltBodyDefaults::kAdoptedRootMaterial` (friction 0, restitution 0); every other body takes
`joltBodyDefaults::kCreatedBodyMaterial`.

**The consequence.** In Chaos the character capsule carries the game's own material (friction 0,
restitution 0), and the game's capsule guards rely on restitution 0: the movement simulation adopts the
solver's push-out each tick, so a bouncy capsule feeds a rebound back into the movement state. Giving
the capsule 0.7 / 0.3 changes every capsule contact (combined with a default static: friction 0.7
instead of 0.35, restitution 0.3 instead of 0.15) and the Jolt build stops matching the Chaos build.
(Witnessed as a poison arm: `JoltPhysicsFactory.EveryBrawlerDeclarationBindsAChaosParityBody` failed.)

**What breaks if it moves.** The tag stays on the material choice. If materials become descriptor
fields (a later step), this guard is retired in favour of the descriptor.

## G-02 — a bind never writes the object layer, gravity factor, position or velocity

**Tag site:** `JoltPhysicsFactory.cpp`, in `JoltPhysicsFactory::applyBodyDefaults`, directly above the
line that takes the body's motion properties (where further body writes would be added).

**The prohibition.** Do not set the object layer, the gravity factor, the position, the rotation or
the velocities of the slot body in the factory, and do not activate or deactivate it there.

**The consequence.** The world owns those through occupancy (`JoltWorld::applyOccupancy` sets the
layer, gravity factor and, for a vacant slot, zero velocity) and through the snapshot ring (position,
rotation, velocities). The host binds a character's bodies a frame or more before the step driver
occupies the slot. A layer or gravity write at bind would make a vacant slot collide or fall before
its first occupied tick, on this peer only. `JoltWorld::applyOccupancy` rewrites only slots whose
occupancy changed, so the wrong layer stays until the slot is occupied or a restore re-applies every
slot (the authority never restores): peers step different worlds for those ticks, and the snapshot does
not contain the layer, so nothing shows it.

**What breaks if it moves.** Nothing as long as the tag stays inside the body write block.
