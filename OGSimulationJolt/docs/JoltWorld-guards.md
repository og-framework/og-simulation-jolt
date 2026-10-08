<!-- SPDX-License-Identifier: MPL-2.0 -->
# `JoltWorld.h` / `JoltWorld.cpp` — guards

Every prohibition that governs a line of `JoltWorld.cpp`. Each entry has an **opaque, stable id**. In
the source a single line `// ⛔G-nn` sits exactly where the forbidden edit would be typed.

**If this file and the sources disagree, the sources are authoritative and this file is stale.** Fix
this file; do not soften the sources to match it.

⛔ **An id is never reused.** A guard that is deleted, or that becomes a compile-time check, is moved to
a "Retired" section and its number is spent forever.

⛔ **Nothing in this file is a rationale.** The reasons live in `JoltWorld-rationale.md`.

---

## G-01 — Do not set or reference mDeterministicSimulation; change no physics setting but the restitution threshold

**Tag site:** `JoltWorld.cpp`, in the `JoltWorld` constructor, immediately after the call to the
physics system's Init and before SetGravity: the line above the GetPhysicsSettings /
SetPhysicsSettings block.

**The prohibition.** Do not add code that reads or writes PhysicsSettings::mDeterministicSimulation
(vendored Jolt), and do not change any other physics setting in that block without changing this
guard and the world's rationale with it. The world runs Jolt's default settings with exactly two
exceptions: mMinVelocityForRestitution, which the block sets from
`JoltWorldConfig::minVelocityForRestitutionCmPerS` (Chaos parity, rationale §2), and one collision
step per update, which is the argument to the update call in `step`.

**The consequence.** mDeterministicSimulation defaults to true, and Jolt upstream has already removed
the field (the simulation is now always deterministic). A reference to it breaks the build on the next
Jolt upgrade, and setting it to false stops Jolt sorting contacts and constraints per island, so the
solver order follows the order in which pairs are found (broadphase layout, job order). A restore
refits every body's bounds and can leave the broadphase in a different layout from the original run,
so byte-identical replays are no longer guaranteed (inferred from Jolt's restore code; not measured). Any other non-default setting (solver iterations, slop, speculative
distance) changes the physics the parity work is measured against, and is not stored in a snapshot,
so peers that disagreed on it would diverge without any snapshot difference to show it. The restitution
threshold is the same kind of setting: it is not in a snapshot either, so it comes from the world
configuration, which every peer builds identically.

**What breaks if it moves.** The block must run after Init and before the first step: a threshold set
later would let the first steps run with Jolt's 100 cm/s default. If the constructor is restructured,
the tag and the block move together with the Init call.
