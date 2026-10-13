<!-- SPDX-License-Identifier: MPL-2.0 -->
# `JoltContactCounter.h` / `JoltContactCounter.cpp` — guards

Every prohibition that governs a line of `JoltContactCounter.cpp`. Each entry has an **opaque, stable
id**. In the source a single line `// ⛔G-nn` sits exactly where the forbidden edit would be typed.

**If this file and the sources disagree, the sources are authoritative and this file is stale.** Fix
this file; do not soften the sources to match it.

⛔ **An id is never reused.** A guard that is deleted, or that becomes a compile-time check, is moved to
a "Retired" section and its number is spent forever.

⛔ **Nothing in this file is a rationale.** The reasons live in `JoltContactCounter-rationale.md`.

---

## G-01 — OnContactAdded only counts: never write the contact settings

**Tag site:** `JoltContactCounter.cpp`, the line above the one-line definition of
`JoltContactCounter::OnContactAdded`.

**The prohibition.** Do not name the settings parameter, and do not write any field of it (the
combined friction or restitution, the inverse mass or inertia scales, the sensor flag, the relative
surface velocities). Do not add anything to the definition but the call to `countManifold`.

**The consequence.** Jolt builds the contact constraint from these settings after the callback
returns, so a write changes the physics of every world the counter is installed on, and only when it is
installed: a session that counts its contacts would no longer play the physics the counts are meant
to explain, and a peer that counts would diverge from a peer that does not. The passivity test
(`JoltContactCounter.AWorldWithTheCounterStepsBitIdenticallyToOneWithout`) fails on a friction write at
the first step.

**What breaks if it moves.** Nothing: the tag stays with the definition.

## G-02 — OnContactPersisted only counts: never write the contact settings

**Tag site:** `JoltContactCounter.cpp`, the line above the one-line definition of
`JoltContactCounter::OnContactPersisted`.

**The prohibition.** As G-01, for the persisted callback: do not name or write the settings parameter,
and add nothing to the definition but the call to `countManifold`.

**The consequence.** As G-01. Jolt calls this callback for every contact that survives from the previous
step, so in a resting scene nearly every contact passes through it, and a write here changes nearly
every step.

**What breaks if it moves.** Nothing: the tag stays with the definition.
