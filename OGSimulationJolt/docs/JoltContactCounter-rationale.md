<!-- SPDX-License-Identifier: MPL-2.0 -->
# `JoltContactCounter.h` / `JoltContactCounter.cpp` — rationale

`JoltContactCounter` is a Jolt contact listener that only observes. It counts the contact manifolds a
physics step touched, splits them by what kind of body is on each side, and records which simulatables
took part. A host installs it on a world, steps, and calls `drain` after each step to read and reset
that step's counts. It exists so a session log can tell contact-driven divergence from different play;
it changes nothing about the physics. The sources hold licence, docs pointers, code and two guard tags;
this file carries the why. Prohibitions are in `JoltContactCounter-guards.md`.

**If this file and the sources disagree, the sources are authoritative and this file is stale.**

---

## §1 What is counted

- **One count per callback.** Jolt calls the listener's OnContactAdded (the manifold was not in the
  previous step's contact cache) or OnContactPersisted (it was) once per contact manifold per step.
  The counter adds one for either, so a step's `manifolds` is the number of manifolds the step's narrow
  phase produced or reused, and `added` is how many of them were new.
- **A manifold, not a body pair.** A manifold belongs to one pair of sub-shapes. Every slot body has one
  shape, so a pair of slot bodies gives one manifold, but a body touching a static mesh can give several
  in one step: a mesh's triangles are sub-shapes, and Jolt merges only those whose contacts share
  nearly the same normal (manifold reduction).
- **"New" is relative to the contact cache the step started from.** After a restore, that is the cache
  in the restored snapshot, so `added` on a replayed step says what the restored cache did not hold.
- **CCD contacts.** For a body with LinearCast motion quality Jolt's CCD pass also calls the two
  callbacks, and can call OnContactPersisted more than once for one pair in one step (Jolt's own comment
  says so). Slot bodies are Discrete unless `JoltPhysicsFactoryOptions::linearCastCharacters` is set
  (`JoltPhysicsFactory-rationale.md` §4); with it set, the counts include CCD contacts as Jolt reports
  them.
- **Not counted:** OnContactRemoved, which carries only the two body ids, and OnContactValidate, which
  the counter does not override (§3).

## §2 Body classes and pair classes

Each body is classified from its user data alone, through `JoltBodyUserData.h`
(`joltContactBodyClassOf`):

| Body class | User data | In the game |
|---|---|---|
| **root** | `joltBodyUserData::isRoot` | a character's root capsule, bound or not |
| **child** | bound (`joltBodyUserData::decode` has a value) and not root | the weapon, the guard, a projectile |
| **static** | anything else: 0, or unbound and not root | the level's static bodies; an unbound non-root slot body |

The root bit is checked first, because a slot body carries it from construction, before any binding
(`JoltBodyUserData-rationale.md`). A parked slot body sits on the parked layer, which collides with
nothing, so the "unbound and not root" case is reached only by bodies a host leaves unbound in a live
layer; a test exercises it.

`joltContactPairClassOf` folds two body classes into one of six pair classes, regardless of order:
root-root, root-child, root-static, child-child, child-static and static-static. Static-static is the
residue that keeps the six counts summing to `manifolds`: two static bodies never collide in Jolt, so
it counts unbound non-root dynamic bodies. Both functions are `constexpr`, and every row is pinned by a
`static_assert` in the header.

## §3 Passive by construction

The physics a world steps with the counter installed is the physics it steps without it. Read in the
vendored Jolt (5.6):

- **Validation.** With no listener, the contact manager's ValidateContactPoint returns
  AcceptAllContactsForThisBodyPair; with a listener it returns the listener's OnContactValidate, whose
  default returns the same value. The counter does not override it, and a `static_assert` in the header
  compares the member pointer's type with the base class's, so an override does not compile.
- **Contact settings.** Jolt computes the combined friction, the combined restitution and the sensor
  flag before it calls OnContactAdded or OnContactPersisted, passes them by reference, and builds the
  contact constraint from them after the call. The counter's overrides leave the settings parameter
  unnamed and write nothing; guards G-01 and G-02 forbid adding a write.
- **What a listener adds.** Building the manifold the callback receives (fixed-capacity arrays, no heap)
  and, at the end of the update, a pass over the old contact cache to call OnContactRemoved, whose base
  version does nothing. Neither writes body or constraint state.
- **CCD.** With a listener, Jolt's CCD pass also records a CCD manifold (no contact points) in the
  contact cache so it can tell added from persisted, and a saved state then carries those entries. That
  path runs only for LinearCast bodies (§1), and the passivity test below runs Discrete bodies only.

The counter's own callbacks only classify the two bodies from their user data and add to atomic
counters.

**Measured:** `JoltContactCounter.AWorldWithTheCounterStepsBitIdenticallyToOneWithout` steps two worlds
built the same way, 8 brawler slots on a floor with their projectiles colliding (friction and
restitution in play) under the rig's scripted forces, one with the counter installed and drained after
every step through `JoltWorld::setAfterStepObserver`, one with no listener. After each of 240 steps the
position, rotation and linear and angular velocity of every slot body compare equal bytewise; after the
last, the full saved state (contacts included) and `liveStateHash` do too. The test also asserts the
scene moved and touched root-static, root-child, child-static and child-child contacts. A counter that
halved the combined friction made it fail at the first step.

## §4 Simulatable ids

For each simulatable the step touched, `JoltContactIdCounts` holds:

- `pairs`: how many manifolds involved a body of this simulatable, per pair class. A manifold between
  two bodies of the same simulatable (a character's weapon against its own capsule) counts once for it;
  a manifold between two simulatables counts once for each.
- `childDeclarations`: bit i is set when a child body of declaration index i touched anything. Indices
  from `JoltContactStepCounts::kRecordedDeclarations` (32) up are counted in `unrecordedDeclarations`
  instead. The declaration index is the one the binding carries, so the host maps it to a name.

The table holds `JoltContactStepCounts::kMaxIds` simulatables, `kMaxSimulatableSlots` (8): a world binds
at most one simulatable per slot, and a `JoltWorld` has at most that many slots. A simulatable beyond
the table is counted in `unrecordedIds` and its per-id counts are lost; the pair class counts stay
complete. `drain` returns the ids sorted, so the order does not depend on which job saw them first;
`findId` looks one up.

## §5 Threads

Jolt calls the callbacks from its job threads, and from several at once when a world uses a
multi-threaded job system.

- **Every count is a lock-free atomic** (`std::atomic`, relaxed order; a `static_assert` checks
  `is_always_lock_free`). Two callbacks adding to one count cannot lose an update.
- **An id claims a slot with a compare-and-swap.** A callback scans the slots in order and takes the
  first that holds its id, or that is empty and which it wins. A slot only goes from empty to an id
  within a step, and a scan never passes an empty slot without trying to claim it, so an id ends up in
  exactly one slot.
- **`drain` runs between steps,** on the thread that called the update, after the update returned. Jolt's
  job system has finished every job of the update by then, and the update's own wait orders those jobs'
  writes before the drain's reads. `drain` resets each count with an exchange. A `drain` during an
  update would split that step's counts; the host calls it from the after-step observer
  (`JoltWorld-rationale.md` §14), which runs after the update.
- **The configuration the game uses:** `JoltWorld` steps with Jolt's single-threaded job system
  (`JoltWorld-rationale.md` §2), so every callback runs on the stepping thread, one after another. The
  atomics make the counts exact anyway, so a world built on a thread pool needs no other counter.

**Measured:** `JoltContactCounter.CountsAreExactUnderAThreadPoolJobSystem` steps a busy 8-slot world
240 times through Jolt's thread-pool job system with 4 worker threads and compares each step's drain
with a mutex-guarded tally of the same callbacks: equal on every step, with callbacks seen on more than
one thread. With the counters' additions replaced by a non-atomic load and store, the same test lost
updates on hundreds of assertions.

## §6 Cost and allocation

Nothing allocates: the counts are fixed arrays, and `drain` returns a `JoltContactStepCounts` by value
(a few hundred bytes). A callback does one pair-class add, at most one `added` add, and per bound body a
scan of at most 8 slots. With no counter installed nothing runs at all, so a host installs it only when
a session asks for contact counts.

A physics system holds one contact listener. A second listener, such as the one the OR, one-way
collision rule would add (`JoltCollisionRule-rationale.md` §3), would have to forward to this one.

## §7 Tests

`JoltContactCounterTest.cpp` (og-simulation-tests, the Jolt folder), tag `[Jolt]`:

- `JoltContactCounter.ACapsuleRestingOnAStaticIsOneRootStaticManifold`: one root-static manifold,
  added on the first step and persisted on the second; a second drain is empty.
- `JoltContactCounter.TwoTouchingCapsulesAreOneRootRootManifoldForEachId`: one root-root manifold,
  counted once for each of the two ids.
- `JoltContactCounter.AChildTouchingACapsuleAndAStaticCarriesItsDeclaration`: root-static, root-child
  and child-static, one each; the child's id carries its declaration bit.
- `JoltContactCounter.ChildChildSameIdAndUnboundBodies`: child-child between two ids, a root-child
  manifold within one id counted once, an unbound root capsule (root-static, no id) and an unbound
  sphere (static-static).
- `JoltContactCounter.AWorldWithTheCounterStepsBitIdenticallyToOneWithout` (§3).
- `JoltContactCounter.CountsAreExactUnderAThreadPoolJobSystem` (§5).

Each was seen to fail first against a counter whose callbacks counted nothing.
