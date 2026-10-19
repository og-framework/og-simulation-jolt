<!-- SPDX-License-Identifier: MPL-2.0 -->
# `JoltTickWorldHost.h` / `JoltTickWorldHost.cpp` — guards

Every prohibition that governs a line of `JoltTickWorldHost.h`. Each entry has an **opaque, stable id**. In
the source a single line `// ⛔G-nn` sits exactly where the forbidden edit would be typed. The `.cpp` has
no guard: its one ordering rule (the queryable statics before the physics-only ones) is held by building
both worlds through that one function and by the test named in G-03.

**If this file and the sources disagree, the sources are authoritative and this file is stale.** Fix this
file; do not soften the sources to match it.

⛔ **An id is never reused.** A guard that is deleted, or that becomes a compile-time check, is moved to a
"Retired" section and its number is spent forever.

⛔ **Nothing in this file is a rationale.** The reasons live in `JoltTickWorldHost-rationale.md`.

**Predecessors.** These ids succeed two guards of the brawler game's Jolt backend host (the
TickDriverBackendUImpl guards doc, then in the game module; OGSimulationUnreal's since og-tickdriver task 15e),
whose code moved here:
- its G-02, "the Jolt bind runs under the world mutex, and the game thread never applies occupancy", had
  one tag, at the bind's lock. Its occupancy edit is typed at the release's occupancy push, so it is split:
  **G-01** (the bind under the lock) and **G-02** (the release pushes the vacate, never applies it);
- its G-03, "the shadow world is built by the step world's calls, and every bind and release is mirrored
  into it", had one tag, at the shadow world's construction. The two mirrors are typed at other statements,
  so it is split: **G-03** (the construction), **G-04** (the bind mirror) and **G-05** (the release mirror).

The game retires its G-02 and G-03 when its host is replaced by this class, naming these ids.

---

## G-01 — the bind runs under the world lock

**Tag site:** `JoltTickWorldHost.h`, in `bind`, immediately above
`WorldLock worldLock(m_worldLock, m_threadContext);`.

**The prohibition.** ⛔ Do not move any part of the bind out of this lock's scope: the `JoltPhysicsFactory`
construction, every `createPhysicalObject` (body defaults, user data, the bind table, the shape registrations
through the query adapter's `registerShape`) and every declaration volume's `registerVolume`. ⛔ Do not
replace the lock with a queue: the bind hands back body, shape and volume ids synchronously, because the
declaration bindings store them before `bind` returns. ⛔ Do not set a slot's occupancy in the bind: the
slot stays parked until the step applies the occupancy command the caller pushes (G-02).

**The consequence.** On a worker client the step runs on another thread and holds the lock for each whole
`runTick`. A bind outside the scope writes Jolt bodies, the bind table's inertia and the query adapter's
tables while the step reads them: a data race that corrupts a tick silently. Occupancy set here would change
the world between two saved ticks without the step's occupancy timeline recording it, so a replay would step
with a different body set than the prediction did.

**What breaks if it moves.** The tag marks the scope's opening line; code typed above it, or after the
scope's closing brace, runs unlocked. Only the query adapter's access predicate checks it, and only on a
worker client and with checks compiled in: releasing the lock before the factory aborted
`JoltTickWorldHost.Bind.BindReleaseAndRebindLeaveTheTwoBindTablesEqual` with the adapter's `registerShape`
assertion (measured 2026-10-16). On an inline host nothing catches it, and the factory's body writes have no
check anywhere.

---

## G-02 — a release pushes the vacated slot to the caller; it never applies occupancy itself

**Tag site:** `JoltTickWorldHost.h`, in `release`, immediately above `pushBodySlotOccupancy(*slot, false);`.

**The prohibition.** ⛔ Do not replace the push with a call that changes the world's occupancy
(`JoltWorld::applyBodySlotOccupancy`, or anything that parks or unparks bodies), here or anywhere on the game
thread, and do not drop it. The vacate goes through the caller's door to the step's occupancy command ring.

**The consequence.** The same as G-01's occupancy half: a slot parked from the game thread changes the
world between two saved ticks without the step driver's occupancy timeline, so a replay steps with a body
set the prediction did not have; on a worker client it also races the running step. A dropped push leaves
the slot occupied in the step after its character left.

**What breaks if it moves.** Replacing the push with `applyBodySlotOccupancy` fails
`JoltTickWorldHost.Bind.BindReleaseAndRebindLeaveTheTwoBindTablesEqual` (no push recorded; measured
2026-10-16). An occupancy write typed elsewhere has no check.

---

## G-03 — the shadow world is built from the step world's configuration with two fields changed

**Tag site:** `JoltTickWorldHost.h`, in `buildShadowWorld`, immediately above
`m_worlds.shadowWorld.emplace(m_worlds.runtime->runtime, shadowConfig, m_log);`.

**The prohibition.** ⛔ Do not build the shadow world from any configuration but the step world's, with
exactly two fields changed: `ringDepthTicks` (0) and `tempAllocatorBytes` (`kShadowTempAllocatorBytes`).
The slot count, the slot template, the static layers, gravity and every other field stay the step world's.
⛔ Do not build its statics by any call but `joltTickWorldHost::buildStatics` with the step world's two
descriptions, and do not add a static or a body to either world alone. ⛔ Do not share one adapter between
the two worlds.

**The consequence.** The restore copies only what a saved state holds: the slot bodies' Jolt state, their
occupancy and the shape enables, and it refuses a state only when the slot bodies differ. Everything else
must already agree, or the visualizations query a world that is not the step world's copy, silently: a
different slot template or slot count changes the slot bodies, and every restore is refused, so the shadow
never moves; a missing, extra or reordered static is invisible to the restore, so the arena the
visualizations query differs from the one the step collides with; a shared adapter puts the game thread's
queries and the step's writes on one object.

**What breaks if it moves.** The configuration is copied, and its two fields changed, in the three lines
above the tag; an edit there is typed off the tagged statement. Keeping the step's ring depth fails
`JoltTickWorldHost.Lines.MatchTheGoldens` and
`JoltTickWorldHost.Build.WorkerClientBuildsAShadowWorldWithTheSameStatics` (the built line; measured
2026-10-16). A different static set is caught at build by the body-count `OG_CHECK`, with diagnostics on;
statics built in another order in both worlds fail the second test's comparison with a reference world.

---

## G-04 — every bind is mirrored into the shadow world

**Tag site:** `JoltTickWorldHost.h`, in `bind`, immediately above `if (m_worlds.shadowWorld.has_value())`, the
statement that calls `bindShadowSlot`.

**The prohibition.** ⛔ Do not bind a slot in the step world without the same bind in the shadow world:
the whole `JoltPhysicsFactory` bind, body defaults included, with the same slot template, slot, id and
options. Do not drop the call, and do not make it conditional on anything but the shadow world existing.

**The consequence.** A bind writes things no saved state carries: the bind table, the shape registrations
and each body's user data and defaults. A bind that is not mirrored leaves the shadow's bodies without user
data, so the query adapter drops every hit on them and the visualizations find no one; a different id,
friction or mass changes hits or their order.

**What breaks if it moves.** Deleting the statement fails
`JoltTickWorldHost.Bind.BindReleaseAndRebindLeaveTheTwoBindTablesEqual` and the line goldens (measured
2026-10-16). With diagnostics on, the next mirrored release logs `bindCompare … equal=0`.

---

## G-05 — every release is mirrored into the shadow world

**Tag site:** `JoltTickWorldHost.h`, in `release`, immediately above `if (m_worlds.shadowWorld.has_value())`,
the statement that calls `releaseShadowSlot`.

**The prohibition.** ⛔ Do not release a slot in the step world without clearing the same slot in the shadow
world's bind table. Do not drop the call, and do not make it conditional on anything but the shadow world
existing.

**The consequence.** A release that is not mirrored leaves the shadow's bind table claiming bodies the step
world freed, so the reader still resolves a character that has left, and the next bind of that slot is
compared against stale entries.

**What breaks if it moves.** Deleting the statement fails
`JoltTickWorldHost.Bind.BindReleaseAndRebindLeaveTheTwoBindTablesEqual` (measured 2026-10-16).

---

## Retired

*(None yet.)*
