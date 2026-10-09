<!-- SPDX-License-Identifier: MPL-2.0 -->
# `JoltWorld.h` / `JoltWorld.cpp` — rationale

`JoltWorld` is og-simulation's `PhysicsWorldAdapter` implemented on Jolt: one Jolt physics system, a
fixed set of slot bodies, a per-tick snapshot ring keyed by sim tick, and the occupancy and
shape-enable configuration that rolls back with it. The sources hold licence, docs pointers, code and
one guard tag; this file carries the why. Prohibitions are in `JoltWorld-guards.md`.

**If this file and the sources disagree, the sources are authoritative and this file is stale.**

---

## §1 What it satisfies

`static_assert(PhysicsWorldAdapter<JoltWorld>)` at the end of `JoltWorld.h` checks every member against
og-simulation's concept: `step`, `saveTick`, `restoreTick`, `hasTick`, `oldestHeldTick`,
`invalidateAllTicks`, `saveScratch`, `commitScratch`, `applyOccupancy`, `stateHash` and `coverage`. The
contracts behind them are in og-simulation's `PhysicsWorldAdapter-rationale.md`; this file says how
Jolt meets them. `restoreFromSnapshot` is not part of the concept: it restores a slot saved by
another world (§13).

`JoltWorld::coverage` is `SnapshotCoverage{}`: a Jolt snapshot does not restore the body set and ids,
the broadphase structure or the statics. The compensations are the fixed body set (§3), sorted query
results (the spatial query adapter's job) and statics built once and never saved (§6).

The constructor takes a `JoltRuntime&` only as proof that Jolt's global state is initialised; the world
must be destroyed before the matching release.

## §2 The physics system

- **Job system:** Jolt's single-threaded job system. A handful of characters forms a handful of
  islands, so threads would cost more than they save, and single-threaded stepping removes any
  thread-order effect on listener callbacks.
- **Temp allocator:** a preallocated `JoltWorldConfig::tempAllocatorBytes` (16 MiB default). Jolt
  allocates its per-step scratch here and **aborts the process** when it runs out (after a trace line,
  which `JoltRuntime` routes to the first acquirer's logger). The contact constraint buffer alone is
  `maxContactConstraints` × 480 bytes per step on Jolt 5.6 (983,040 bytes for 2,048); measured
  2026-10-07, a 1 MiB allocator with the default caps aborted on its first step, contacts or not. 4 MiB
  is enough for the tests; 16 MiB is the default.
- **Settings:** Jolt's defaults, with one collision step per update (the argument to the update call in
  `step`) and one changed setting, the restitution velocity threshold (§12). Guard G-01 forbids
  touching mDeterministicSimulation (vendored Jolt's physics settings) or any other setting.
- **Gravity:** `JoltWorldConfig::gravityCmPerS2` converted once, in the constructor, through
  `joltUnits::centimetresToMetres` (`JoltUnits-rationale.md`).
- **Combine rules:** the constructor sets the physics system's friction and restitution combine
  functions to `joltBodyDefaults::combineFrictionAverage` / `combineRestitutionAverage`, Chaos's
  effective rule for every pair, statics included (`JoltBodyDefaults-rationale.md` §2). They are not
  physics settings (guard G-01's block does not hold them) and not in a snapshot, so like the threshold
  they come from construction, which every peer runs identically. Each `JoltPhysicsFactory`
  constructor used to set them (idempotently); a world-wide rule belongs to the world, and
  a world with no factory, such as a test world or the determinism fingerprint's, now has the same
  contact rule as a world with characters.
- **Caps** (construction parameters, `JoltWorldConfig`): `maxBodies` 1024, `maxBodyPairs` 4096,
  `maxContactConstraints` 2048. The worst case of the game this was built for is 8 slots × 6 bodies = 48
  slot bodies plus a few static chunks; under the M1 collision rule only the 8 capsules collide (with
  each other and with statics), at most 28 capsule pairs plus 8 × (static chunks) capsule-static
  pairs. The caps are two orders of magnitude above that.

## §3 The fixed body set

Jolt's snapshot does not include which bodies exist, and a body removed and re-added gets a new id.
So no slot body is ever removed: the world is built from a **slot template** (`SlotBodyTemplate`, one
per body a character declares; 6 for the brawler: radial, guard, three projectiles, capsule) and
instantiates it for `JoltWorldConfig::simulatableSlots` slots (at most `kMaxSimulatableSlots`) when it
is constructed. Every slot body is created with an explicit id, added once, and destroyed only with the
world.

- **Ids.** `JoltWorld::slotBodyId(slot, templateIndex)` = 1 + slot × bodies-per-slot + template index.
  Index 0 is never used, so no body id (the seam's `BodyId` is Jolt's index-and-sequence value, and the
  sequence is 0 for an explicit id) is ever 0. Static bodies get the next explicit ids through
  `createStaticBody`; a body created with Jolt's automatic ids could pick up the free index 0, so all
  bodies are created through the world (`JoltWorld.BodyIdsAreNeverZero` checks every body).
- **Shape.** The template's single shape (`ShapeDescriptor`; multi-shape slot bodies fail an
  `OG_CHECK`) is converted from centimetres: sphere, box (convex radius capped by the smallest half
  extent) or capsule. `CapsuleGeometry::halfHeight` is the total half height, hemispheres included, and
  Jolt's capsule is Y-up with a cylinder half height, so the capsule is built with
  `halfHeight - radius` and rotated a quarter turn about X to stand along Z.
- **Body settings.** Dynamic when `BodyDescriptor::simulatePhysics`, else kinematic; sleeping off on
  every slot body (a sleeping body would make the active set history-dependent); rotation locked to
  translation-only degrees of freedom when `BodyDescriptor::lockRotation`. Materials, damping, mass
  and motion quality are Jolt's defaults here; the body factory sets the parity values.
- **Parking position.** Every slot body starts at `JoltWorldConfig::parkingPositionCm`, on the PARKED
  layer with gravity factor 0.

## §4 Occupancy

`applyOccupancy(occ)` sets, for each slot whose bit changed since the last call:

- **occupied:** every body of the slot on its template layer (`templateLayer`) with its template
  gravity factor (1 when `BodyDescriptor::enableGravity`, else 0);
- **vacant:** every body on the PARKED layer, gravity factor 0, linear and angular velocity zero.

Unchanged slots are skipped, so calling it before every step (as the step driver does) is cheap. The
layer is only written when it differs, so an unchanged body is not moved inside the broadphase. A
parked body collides with nothing and matches no query (`JoltLayerTable-rationale.md`), and with no
gravity and no velocity it stays where it is (`JoltWorld.ParkedBodiesMakeNoContactsAndKeepZeroVelocity`:
24 projectile bodies parked at one point, 100 steps, no contact; the same bodies occupied do collide).

## §5 Save, scratch and commit

`saveTick(t)` and `saveScratch()` save into a ring slot or the scratch slot
(`JoltStateRing-rationale.md`):

- Jolt's save with every part (global, bodies, contacts, constraints) and a filter that **skips static
  bodies** (they never change) and **keeps every contact with at least one non-static side**. Contacts
  against statics are the warm-start cache; dropping them makes a replay start differently from the
  original run (witnessed 2026-10-07: keeping only non-static/non-static contacts made six tests fail,
  among them both byte-identical replay tests).
- the list of saved body ids, recorded by the filter;
- the `ConfigSidecar`: the occupancy last applied and the shape-enable bits;
- the canonical state hash (§8).

`commitScratch(t)` copies the scratch slot into the ring under `t` with the same overwrite and eviction
rules. A Skip's backfilled tick therefore holds the pre-step world
(`JoltWorld.CommitScratchRestoresThePreStepWorld`).

## §6 Atomic restore

og-simulation's contract says a restore applies fully or not at all. Jolt's restore writes bodies as it
reads them and returns false when it meets a body id that does not exist, after it has already changed
the bodies before it. It also restores a body that exists but is no longer in the broadphase without
complaint (witnessed 2026-10-07: with the pre-validation disabled, restoring after one slot body was
removed from the broadphase returned true and changed that body). So `restoreTick(t)` returns false
if `t` is not held, and otherwise runs the private helper `restoreSlot` on the held slot with the undo
save taken. `restoreFromSnapshot` (§13) runs the same helper without it, so the two restores cannot
drift apart. The helper:

1. **pre-validates**: every recorded body id must exist, be in the broadphase and be non-static, and the
   live set of non-static bodies in the broadphase must have the same size. Otherwise it logs
   "restoreTick(t) refused" (or "restoreFromSnapshot(t) refused") and returns false with the world
   untouched (`JoltWorld.RestoreOfAMismatchedBodySetIsRefusedAndLeavesTheWorldUntouched`);
2. for `restoreTick` only, saves the live world into the restore-undo slot;
3. runs Jolt's restore and requires it to succeed, to leave the recorder unfailed and to consume every
   byte;
4. if that fails anyway, restores the undo slot when one was taken, logs, fails an `OG_CHECK` and
   returns false (fail loud: Jolt's own asserts exist only in builds without NDEBUG, such as Unreal
   Debug);
5. applies the sidecar: occupancy with **every** slot forced (layers, gravity, zero velocity for vacant
   slots), then the shape-enable bits.

Step 4's undo uses its own slot rather than the scratch slot so that a restore can never destroy a
scratch snapshot that a later `commitScratch` expects. Step 4 is not covered by a test: it needs Jolt's
restore to fail after pre-validation passed, which no reachable input produces. With the
pre-validation disabled it is reachable (a slot recording bodies this world does not have): the
`OG_CHECK` then fails, and the Unreal test runner reports the failed check as a test failure
(witnessed 2026-10-08).

The step driver also re-applies the anchor's occupancy after `restoreTick`; doing it twice is harmless.

## §7 Contact-cap overflow

Jolt's update returns error flags when a cache or buffer was full and contacts were dropped
(manifold cache, body-pair cache, contact constraints). `step` checks the flags after every update.
For each flag set it counts the occurrence (`contactOverflowCount`) and logs at most one line per flag
per second:

> [Warning] JoltWorld: Jolt dropped contacts: FLAG at physics step N (last saved sim tick T); K step(s)
> with this flag since the last line; raise JoltWorldConfig::CAP

`step` does not receive the sim tick, so the line names the world's physics step count and the last
tick saved or committed, which is the tick before the overflowing step in the normal flow. The clock is
`JoltWorldConfig::clock` (steady clock by default; the test injects one). In a build with Jolt asserts
(standalone Debug), Jolt itself asserts inside the update on any of these flags before the world can log
it, so `JoltWorld.ContactOverflowIsLoggedOncePerFlagPerSecond` skips itself there.

## §8 The state hash

`stateHash(t)` returns the hash stored with held tick `t`; `liveStateHash` computes it for the live
world. It is a 64-bit FNV-1a over, for every **occupied** slot in slot order and every body in template
order: the body id (index and sequence), the centre-of-mass position, the rotation quaternion
(x, y, z, w), the linear and the angular velocity, each float as its bit pattern; then the occupancy
bits. It never hashes snapshot bytes, so a second backend can produce the same value from the same
seam-level state.

The values are Jolt's own (metres, radians), not converted: a conversion to centimetres can map
adjacent floats to the same value and hide a one-ULP difference. Signed zeros and NaN payloads are
hashed as they are. A one-ULP change in one position changes the hash
(`JoltWorld.StateHashMatchesOnReplayAndSeesOneUlp`, which also checks that the hash of a restore +
replay equals the original at every tick).

## §9 FP environment

`step` runs inside `JoltStepFpScope` and checks the mode (`JoltFpEnvironment-rationale.md`).

## §10 Threading and access

The world is single-threaded: every member, and `bodies`, `query` and `physics`, must be called from the
thread that steps it. `bodies` returns Jolt's non-locking body interface. Nothing here checks the
thread; the host decides which thread owns the world.

## §11 What the tests show (2026-10-07, Win64 Development and standalone Debug with Jolt asserts)

- 8 occupied brawler slots on a static floor with scripted forces, 300 ticks: an unfiltered restore of
  tick 200 and a replay to 300 give byte-identical full state; the same through the filtered ring
  (stateHash equal at every replayed tick); and a restore of tick t followed by one step equals the
  original tick t+1, byte for byte, for 43 values of t.
- A slot joining at tick 100: restore 95, replay to 105, byte-identical at every tick, the slot parked
  before 100 and live from 100.
- Ring eviction, overwrite, invalidation, authority depth 0, shape-enable and occupancy restore,
  atomic refusal, and the collision rule in a stepped world.
- The restore from another world's slot (§13, 2026-10-08).

## §12 The restitution velocity threshold (Chaos parity)

A contact gets its restitution only when the bodies approach faster than a threshold. Jolt's threshold
is PhysicsSettings::mMinVelocityForRestitution, 1 m/s by default (vendored Jolt PhysicsSettings.h),
compared against the normal velocity in the contact solver. Chaos's is an acceleration multiplied by
the step: DefaultRestitutionThreshold 1000 cm/s² (Chaos PBDRigidsEvolutionGBF.h), multiplied by Dt in
the collision solver (PBDCollisionContainerSolver.cpp), so 1000 × 1/60 = 16.67 cm/s at the game's
60 Hz tick, compared the same way (restitution only when the approach is faster than the threshold).
No game ini or console variable overrides either value.

So the world sets mMinVelocityForRestitution from `JoltWorldConfig::minVelocityForRestitutionCmPerS`,
converted once through `joltUnits::centimetresToMetres`, and the configuration's default is
`kChaosParityMinVelocityForRestitutionCmPerS` (1000 cm/s² × 1/60 s). The value is a velocity, not
Chaos's acceleration, because the world's step length is fixed by the host and never seen by the
constructor; a host that steps at another rate (a 120 Hz tick) must pass 1000 × its step.

The setting is not in a snapshot, so like the other world settings it must be the same on every peer;
it comes from the configuration, which every peer builds identically.

Measured (`JoltSuite.TheRestitutionVelocityThresholdIsChaosParity`, 2026-10-07): the brawler capsule
(friction 0, restitution 0) dropped on a default-surface floor (0.7 / 0.3) gets the Average-combined
restitution 0.15. Its rebound at approach speeds of 10 / 50 / 150 cm/s is 0 / 7.5 / 22.5 cm/s with
the parity threshold, and 0 / 0 / 22.5 cm/s with Jolt's default: landings between 16.7 and 100 cm/s
now bounce as they do in Chaos.

Parity row for gate 22:

| Setting | Chaos (today) | Jolt world | Verdict |
|---|---|---|---|
| Restitution velocity threshold | 1000 cm/s² × dt = 16.67 cm/s at 60 Hz | `kChaosParityMinVelocityForRestitutionCmPerS` = 16.67 cm/s | MATCH at 60 Hz |

Note for gate 22: Chaos combines friction and restitution with Average, so the capsule (a 0 / 0
material) against a default static already gets friction 0.35 and restitution 0.15 in Chaos; "0/0
capsule" describes the material, not the contact.

## §13 Restoring from another world's slot: `restoreFromSnapshot`

`restoreFromSnapshot(slot)` restores this world from a `JoltStateSlot` that another world saved. It
exists for a **shadow world**: a second `JoltWorld`, owned by a thread that does not step the
simulation (a host's drawing thread), which never steps itself and is refreshed from the step world's
newest saved slot, so that spatial queries made on that thread run against the step's poses without
touching the step world while it steps. `restoreTick` cannot serve: it restores only ticks held in the
world's own ring, and the byte-level restore it uses is private.

**What it does.** It refuses a slot that holds no snapshot (`valid` false) with a Warning line and
returns false. Otherwise it runs the helper of §6 without the undo save: the body-set check (refusal
with a Warning line naming `restoreFromSnapshot` and the slot's tick, the world unchanged), Jolt's
restore, then the slot's sidecar (occupancy for every slot, then shape enables).

**What it does not do.**
- It writes nothing to the ring: no slot is held, overwritten or invalidated, and the last saved tick
  is unchanged.
- It takes no undo save. The pre-validation is what makes the restore atomic in practice; the undo
  save exists for a restore that fails after it, which no reachable input produces (§6). For a world
  refreshed once per drawn frame the undo would be a full state save per frame for nothing: a shadow
  that a failed restore left half-written is refreshed from the next slot anyway, and the failure
  still fails the `OG_CHECK`. The tests pin this with a shadow whose ring slots are 64 bytes, too
  small for any snapshot: an undo save there would log that the snapshot does not fit and fail its
  `OG_CHECK`.

**What the two worlds must share.** A slot holds Jolt's body state, the saved body ids, the contact
cache and the sidecar (§5). Everything else must already match, because the restore cannot carry it:
- the world configuration (slot template, slot count, layers, settings): the body ids in the slot are
  `slotBodyId` values, which depend on the slot count and the template;
- the bind of every slot body: the body factory's materials, mass properties and user data, and the
  query adapter's shape registrations, are set by the bind, not by a restore. The spatial query
  adapter drops hits on a body without user data, so a shadow that is not bound exactly as the step
  world is answers queries differently although its body states are identical;
- the statics, built by the same calls in the same order.

**The static-coverage limit.** Statics are not in a slot (the save filter skips static bodies) and not
in the body-set check (it counts non-static bodies only), so a shadow whose statics differ restores
without complaint. Building the same statics in a different order is enough to give them different
body ids; the contact cache in the slot then names the step world's static ids, which is harmless
only because the shadow never steps and queries do not read the cache. The one static difference
the restore does detect is a static that holds a recorded slot body's id, because the body-set check
refuses a recorded id that is static in this world. Keeping the statics identical is the host's job.
A test pins that a static-order difference is **not** refused, so the limit is a measured property
rather than an assumption
(`JoltShadowWorld.StaticsInADifferentOrderAreNotRefusedTheStaticCoverageLimit`).

**Configuration of a shadow.** A shadow needs no ring (`ringDepthTicks` 0), and its temp allocator,
which Jolt's step takes its scratch from (§2), can be small. Measured: a shadow with a 64 KiB temp
allocator builds its statics, binds 8 slots, restores and answers 200 queries.

**Threading.** Each world is single-threaded (§10). The shadow reads only the slot it is given, so the
slot must be one the step world is no longer writing: a copy handed over through a mailbox, never a
pointer into the step world's live ring.

**Measured** (`JoltShadowWorldRestoreTest.cpp`, Win64 Development, 2026-10-08): 8 occupied brawler
slots on an arena floor with contacts, a 7,601-byte slot. After the restore, `liveStateHash` equals
the step world's and the slot's stored hash, the occupancy and shape enables equal the step world's,
and 200 random sphere overlaps through each world's `JoltSpatialQueryAdapter` return identical hit
lists (body, root, categories, position), at least 20 of them with slot-body hits. A shadow with one
slot fewer, or one template body fewer, is refused with one Warning line and its hash and full state
unchanged. 200 restores alternating between the two newest ticks: median 0.0041–0.0047 ms over
six runs (p95 at most 0.0078 ms).
