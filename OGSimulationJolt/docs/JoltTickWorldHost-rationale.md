<!-- SPDX-License-Identifier: MPL-2.0 -->
# `JoltTickWorldHost.h` / `JoltTickWorldHost.cpp` — rationale

`JoltTickWorldHost` is the Jolt half of a tick-driver host: it owns the physics worlds a simulation steps,
emplaces the session's three adapters over the step world, binds characters into fixed body slots, keeps a
game-thread copy of the step world for queries (the shadow world), and logs the contacts each step touched.
It is engine-free and game-agnostic: the game supplies the simulatable type and the owner keys, the engine
supplies a clock, a frame number, a log sink and a lock. The sources hold the licence, the docs pointers,
the code, the `static_assert` and `OG_CHECK` messages, the log strings and five guard tags; everything that
explains them is here. Prohibitions are in `JoltTickWorldHost-guards.md`.

**If this file and the sources disagree, the sources are authoritative and this file is stale.** Fix this
file; do not soften the sources to match it.

⚠ **This file ships with og-simulation-jolt under MPL-2.0.** Everything below is true to a reader with no
game engine and no other repository. The code it replaced lived in a game's engine module; §14 names it.

---

## §1 What it is for, and what it is not

A tick driver (og-simulation's `SimulationTickDriver`) runs a `SimulationStepDriver` over a physics world.
It needs a world host that models `TickDriverWorldHost` (`TickDriverConcepts.h`): a `World`, a step lock, the
two step doors and the thread context. This class is that host for Jolt. Besides the concept's members it
gives the session what it needs from the physics side at world build, at a character's join and leave, and
once per frame on the game thread.

It also models og-simulation's `TickBackendWorldHost` and `TickBackendWorldBuild` (`SimulationTickBackend.h`), the
members og-simulation's `SimulationTickBackend` calls (`build`, `bindIntegratedTickSource`, `rootDeclarationIndex`,
`acquire`, `bind`, `release`, `slotOf` and the three visualization doors), so a game can let that backend drive it.
Two `static_assert`s at the end of the header check both, over og-simulation's simulatable archetype, `std::mutex`,
the self-check engine and the three real adapter optionals; that is why the header includes
`SimulationTickBackend.h`. The archetype checks the declarations; the bodies compile where a game instantiates
the backend with its own simulatable.

It does **not** step anything. The driver opens the step scope, calls `runTick` and calls the two step
doors; this class only answers them. It does not read cvars, own actors, import level geometry or convert
engine units: the engine side does those and passes the results in (`JoltTickWorldHostConfig`, the two
static descriptions, the owner keys and names).

## §2 The template parameters, and how the host learns the diagnostics switch

`JoltTickWorldHost<SimulatableT, LockT, EngineT, LineAllocatorT = std::allocator<char>>`:

- **`SimulatableT`** is the game's character type. The host reads only its physics composite: the slot
  template is folded from `getPhysicsComposite()`'s type (§5), and `bind` writes the bindings through
  `editPhysicsComposite()`. Each declaration must model og-simulation's `PhysicsDeclaration`, and the
  composite must be a `SimulationComposite` (the shadow hand-over slots are sized from its declaration
  count at compile time).
- **`LockT`** is the world lock, "J2" in the design that introduced it. It must model og-simulation's
  `BasicLockable` (`lock()`, `unlock()`); an engine mutex with other member names is wrapped first, as the
  test does with the kit's `WrapperOverLockUnlockOnly`.
- **`EngineT`** models `JoltTickHostEngine` (this header): a constant-expression `kHostDiagnostics`,
  `nowSeconds()`, `frameNumber()`, `logEnabled(level)` and `log(level, line)`. It is the subset of
  og-simulation's `TickDriverEngine` this class uses, so the engine type a tick driver is built with
  models it too. The concept's self-check namespace holds one conforming and two non-conforming shapes.
- **`LineAllocatorT`** is the allocator of the host's three line buffers (§12). Production code keeps the
  default; the test passes a counting allocator to show that a step's Verbose lines allocate nothing.

**The diagnostics switch is a template parameter, through `EngineT`.** The step doors are called by the
driver with the arguments `TickDriverWorldHost` fixes (`afterTick_Step(outcome)`,
`afterRunTick_Step(step, outcome)`), and the second one logs. The host therefore has to hold the engine
anyway; the switch comes with it as `EngineT::kHostDiagnostics`, read once into the private
`kDiagnostics`. Every diagnostic is in the header, behind `if constexpr (kDiagnostics)`:
the bind-table comparison and its two lines, the shadow window and its line, the hash check after a
restore, the shadow build's body-count check, the visualization-door counters, and the overwrite's
visitor. With diagnostics off none of them is compiled; the `[SimHost.Shadow] built` line, the
contact lines and every other line stay. The constants the old code declared only in development builds
(`joltTickWorldHost::kShadowWindowFrames`) are plain declarations, so both settings compile in one
build, and the test instantiates both.

The `.cpp` holds no diagnostic code (§3).

## §3 One copy of the thread-context state per library

`TickDriverThreadContext.h`'s masks and owner counts are `inline thread_local` variables. Each shared
library that compiles a read or write of them gets its own copy (`TickDriverThreadContext-rationale.md`
§7 in og-simulation's docs). In a build where og-simulation-jolt is its own shared library, the host's
`.cpp` is compiled there, while the header template is instantiated where the game names the host. So:

- **every use of the thread context is in the header template**: `configure`, the `WorldLockScope`s
  (`lockForStep`, the build's scope, `bind`'s scope), the step-scope check in the contact recorder, the
  predicates and the two counter reads of the shadow window;
- **the `.cpp` touches none of it.** Its functions, all in namespace `joltTickWorldHost`:
  `splitStatics`, `rootDeclarationIndexOf`, `stepWorldConfigOf`, `buildStatics`, `slotRootBodyId`,
  `boundBodyCount`, `prepareShadowStateSlot` and `copySavedState`. None reads or writes the
  `tickDriverThread` state, none takes a lock scope, and none logs.
- The access predicates handed to `JoltSpatialQueryAdapter` are function pointers to the header's
  instantiation, so the adapter (compiled in og-simulation-jolt) calls code that reads the instantiating
  library's copy.

The runtime lease (`JoltTickRuntimeLease`) is also in the header: `JoltRuntime::acquire` passes the
caller's `JPH_VERSION_ID` to the library, which refuses a caller compiled with other Jolt defines. Moved
into the `.cpp`, that check would compare the library with itself.

## §4 Lifetime and member order

- **`JoltTickWorlds`** holds the runtime lease, the contact counter, the step world, the shadow world, the
  shadow world's three adapters and the slot template. Members are destroyed in reverse declaration
  order, and two `static_assert`s keep the order that matters: the lease outlives both worlds (the last
  `JoltRuntime::release` unregisters Jolt's types and deletes its factory); the counter outlives the step
  world (the world holds a raw pointer to its contact listener until it is destroyed); the shadow world
  outlives its adapters, and its body adapter outlives the reader and the query adapter (the reader holds
  the body adapter's bind table).
- **The step world's adapters are the session's**, not the host's: `build` takes references to three
  `std::optional` objects the caller owns and emplaces them. The caller declares them after the host, so
  they are destroyed first, while the step world they reference is still alive. The host keeps pointers
  to the three optionals for `bind`, `release`, the visualization doors and the bind-table comparison.
- **The thread context** is configured once, by `build`, on the calling thread. Destroy the host on that
  thread where practical; destroyed elsewhere, the owner count stays raised on the configuring thread,
  which is safe (`TickDriverThreadContext-rationale.md` §3).
- The host is neither copyable nor movable (the deleted copy operations suppress the moves), and must not
  be: the step world's after-step observer holds `this`.

## §5 `build`

`build(config, queryableStatics, physicsOnlyStatics, bodyAdapter, readerAdapter, queryAdapter)`, on the
thread that will own the world (the game thread in an engine), in this order:

1. **The stepping mode and the thread context, first.** The authority always steps inline; a client steps
   inline only with `config.runInline`. `configure` records the role bit (authority 1, client 2) and the
   mode, marks the calling thread as the role's owner and fixes the step and shadow predicates. Then one
   `[SimHost.Mode] role=… stepping=<inline|worker> viz=<stepWorld|shadowWorld>` Warning. This must precede
   every adapter emplacement: the query adapter is built with the predicate, and an inline host's game
   thread may register a volume between steps without the lock only because it is the owner thread. An
   `OG_CHECK` right before the first emplacement holds the order (it names what it replaced); the
   `WorldLockScope` the build takes before it also fails its own check if the role is not chosen yet.
2. **The runtime lease** (§3).
3. **The slot template**, folded from the simulatable's physics composite: one `SlotBodyTemplate` per
   declaration, in declaration order, with the declaration index as its index. The factory binds in
   template order, so that order must be the composite's. `rootDeclarationIndexOf` `OG_CHECK`s that
   exactly one declaration is `isRoot` (the body a renderer moves) and returns its index, which a
   presenter reads through `rootDeclarationIndex()`.
4. **The step world's configuration** (`stepWorldConfigOf`): `kMaxSimulatableSlots` slots, the slot
   template, the static layer keys of both static parts (queryable first), the gravity, and a ring of
   `config.clientRingDepthTicks` on a client and none on the authority, which never resimulates. The
   default ring depth is og-simulation's `rollbackWindowHardCap` plus 2.
5. **Under the world lock:** the step world; if `config.contactCounts`, the contact counter as its
   listener, the after-step observer and the contact buffer's reservation (§9); the statics
   (`buildStatics`, below); the body adapter, the reader over its bind table, the query adapter with
   `config.mappedCategories` and the step predicate; the physics-only statics excluded from queries; one
   `[SimHost.World]` Warning with the slot count, bodies per slot, ring depth and the two parts' shape
   and body counts. Nothing contends for the lock at build; a worker client's query predicate demands it.
6. **The shadow world**, on a worker client only (§11).
7. **One `[SimHost.Contacts] installed=…` Warning**, and an `OG_CHECK` that the step world's listener is
   set exactly when the counter was installed and that the shadow world has none.

**The statics.** `buildStatics` runs two `JoltStaticWorldBuilder`s on one world, the queryable part first,
then the physics-only part, and returns each builder's body ids. Both worlds are built by this one
function, so they get the same calls in the same order and the same static body ids; a query hit's
static key is the body index, so the order must also be the same on every peer. `splitStatics` splits
an imported description into the two parts by a per-shape flag, the shape order kept; it
`OG_CHECK`s one flag per shape.

**`[SimHost.World]`'s counts** are shapes (`statics=`, `physicsOnlyStatics=`) and bodies (`staticBodies=`,
`physicsOnlyBodies=`). A builder may put several shapes into one body (a chunk), so the two differ.

## §6 Slots

A character owns one of the world's fixed slots from its first acquire until it leaves, and the same slot
on every peer. The bookkeeping is og-simulation's `TickSlotTable` (its rationale has the rule, the waits and
the `[SimHost.Slot]` lines); the host adds the Jolt parts.

- **`acquire(key, name, isAuthority, replicatedSlot, publish)`** asks `slotOf(key)` first, then the table's
  `acquire`, and calls `publish(slot)` only when the peer is the authority, the acquire returned a slot and
  the key held none before. An acquire for a key that already holds a slot returns the same slot, and the
  authority's replicated slot must be written once.
- **`release(key, id, pushBodySlotOccupancy)`** frees the key's slot in the table, then calls
  `pushBodySlotOccupancy(slot, false)`, clears the slot's bind-table entries in the step world, logs
  `[SimHost.Bind] release id=… slot=… boundBodies=… slotsInUse=…` (Warning; `slotsInUse` read after the
  table forgot the key), and mirrors the release into the shadow world (§11). It returns the slot, or
  nothing for a key that held none (then it does nothing else).
- **Occupancy is never applied here.** A slot's bodies stay parked until the step applies the occupancy
  command the caller pushes; `pushBodySlotOccupancy` is the caller's door to that command ring (the tick driver's
  `pushBodySlotOccupancy`). The release clears the bind table on the calling thread without the lock, as the code
  it replaced did (§14). A caller whose step thread can still read a released slot's bind-table entries
  needs the release under the lock.
- `slotOf(key)` and `slotsInUse()` expose the table's two queries.

## §7 `bind`

`bind(slot, id, simulatable, staticData, onWorldLockWait)` binds the slot's bodies to one character, for a
slot `acquire` returned.

1. It takes the world lock, timing the wait with the engine clock, and hands the wait to
   `onWorldLockWait(seconds)` (the tick driver's `noteWorldLockWait`).
2. Under the lock: a `JoltPhysicsFactory` over the session's body adapter, the slot template, the slot and
   `id` as the simulatable id, with the session query adapter's `registerShape` as the shape registrar;
   then, for each declaration in composite order, `createPhysicalObject`, and the bindings written: the
   body id, the parent (the slot's root body), the attachment offset and shape ids, and each declaration
   volume registered with the root as the ignored body. The lock covers all of it: on a worker client the
   step reads the bodies, the bind table and the query adapter's tables, and the query adapter's predicate
   refuses a registration outside it.
3. After the lock: one `[SimHost.Bind] bind id=… slot=… body0=… boundBodies=… slotsInUse=…` Warning. `body0`
   is the Jolt body index of the slot's first template body, so two peers' lines show the same body ids,
   not only the same slot.
4. The bind mirrored into the shadow world (§11).

`id` must be the same on every peer (the game's peer-stable key), because the query adapter orders hits by
it. The lock is not recursive: do not call `bind` with the world lock held.

## §8 The visualization doors

`editVizQuery()` and `getVizReader()` return the step world's query adapter and reader on an inline host
(the game thread steps, so it may query between steps), and the shadow world's on a worker client.
`registerVizVolume(descriptor, key, name, isAuthority, replicatedSlot, publish)` acquires the key's slot as
`acquire` does and registers the volume, with the slot's root body as the ignored body, in the same world
the two doors hand out; while the acquire waits it returns nothing and the caller retries. With diagnostics
on, each door counts which world it handed out, for the shadow window (§11).

## §9 The step doors

- **`lockForStep()`** returns a `WorldLockScope<LockT, JoltWorldAccessScope>`: the lock, the Jolt access
  scope and this role's "lock held" bit. The driver holds it for the whole `runTick` and opens the step
  scope itself.
- **`afterTick_Step(outcome)`**, from the driver's `afterTick` hook, hands the step's saved state to the
  shadow world (§11). It does nothing on an inline host.
- **`afterRunTick_Step(physicsStep, outcome)`**, after `runTick` returns, logs that `runTick`'s contact
  steps when the counter is installed.

**The contact steps.** The step driver runs a resimulation's replayed steps and the forward step inside
one `runTick`, and its hooks fire once per `runTick`, never during the replay. So the per-step record comes
from the step world's after-step observer: after every world step, forward or replayed, it drains the
counter into a buffer with the integrated tick and the world's step count. `afterRunTick_Step` then checks
that it drained exactly `replayedTicks + 1` records, logs every record but the last as replayed, and clears
the buffer. The buffer is reserved at build for the ring depth plus one (a replay never exceeds the ring);
an `OG_CHECK` fires before a push past it.

**The integrated tick** is the manager's: call `bindIntegratedTickSource(manager)` once, before the first
step, with any object that has `currentIntegratedTick()`. The host stores a pointer and a captureless
thunk. An `OG_CHECK` fires if a step records a contact step without one. Nothing else reads it.

**The Verbose line**, one per contact step that touched a contact, only when the engine has Verbose
enabled:

```
[Verbose][SimHost.Contacts] tick=%u step=%llu worldStep=%llu replay=%d manifolds=%u added=%u rootRoot=%u rootChild=%u rootStatic=%u childChild=%u childStatic=%u staticStatic=%u ids=%s idsDropped=%u declarationsDropped=%u
```

`ids` lists each simulatable id that took part, as `id:rootRoot/rootChild/rootStatic/childChild/childStatic:0x<child declarations>`
in the counter's order, or a dash; the field meanings are `JoltContactCounter-rationale.md`'s. The ids are
formatted piece by piece into a reserved buffer and the line into another (§12), so a step at Verbose
allocates nothing while both fit their reservations.

## §10 The game-thread doors

- **`restore_GameThread(frameNumber, onRestore)`**, once per frame before anything reads the shadow world,
  on a worker client (on an inline host it returns at once). It peeks the newest handed-over state and
  restores it only if its sequence is newer than the last one restored: each published state is restored
  at most once, and a frame with nothing newer counts as unchanged. A restored state's timing and whether
  it was refused go to `onRestore(seconds, refused)` (the tick driver's `noteShadowRestore`), timed with the
  engine clock. A refused state (its body set differs; og-simulation-jolt logs the Warning) is not retried.
  It records `frameNumber` for the overwrite's check.
- **`overwriteShadowPoses_GameThread(poses, forEachTargetSlot, visitor)`**, after the restore, writes the
  rendered poses into the shadow world's slot bodies so the visualizations query what is shown.
  `forEachTargetSlot(visit)` calls `visit(id, slot)` once per render target that holds a slot, in the
  caller's target order (it is the spec's slot lookup). For each declaration with a pose in `poses` and a
  bound shadow body, the pose's position and, where it has one, rotation are set. With diagnostics on,
  `visitor(id, declarationIndex, pose, readBack)` is then called, where `readBack()` returns the shadow
  reader's transform of that body, for the caller's tolerance check and its Warning; the read happens only
  if the visitor calls it. An `OG_CHECK` requires the restore to have run in
  the engine's current frame: a restore after the overwrite would put the newest step back.

## §11 The shadow world

On a worker client the step runs on another thread, so the game thread may not query the step world. The
visualizations query a second world instead, which only the game thread touches and which is restored once
per frame from the newest state the step saved. Inline hosts build none.

- **Construction** (end of `build`): the step world's configuration with two fields changed, a ring depth of
  0 (it never saves a tick) and a 64 KiB temp allocator (`kShadowTempAllocatorBytes`; it never steps, where
  the step world's default is 16 MiB); the statics by `buildStatics` from the same two descriptions; its own
  body adapter, reader and query adapter, the physics-only statics excluded as in the step world, and the
  shadow predicate (the owner thread only). An `OG_CHECK` first requires the step world's ring slots to have
  `JoltWorldConfig`'s default size, the size the hand-over slots are pre-reserved to.
- **The build check.** The restore compares only slot bodies (the save filter skips statics), so a shadow
  whose statics differed would restore without complaint. One `[SimHost.Shadow] built bodies step=…
  shadow=… sameStatics=…` Warning on every build, and, with diagnostics on, an `OG_CHECK` that the slot
  plus static body counts and the static body ids agree.
- **Mirroring.** A step-world bind writes things no saved state carries: the bind table, the query
  adapter's shape registrations and each body's defaults and user data. So every bind runs the same factory
  bind over the shadow's adapters (same template, slot, id and options), and every release clears the
  shadow's bind table for the slot. Declaration volumes are not registered in the shadow; the
  visualizations use only the volume `registerVizVolume` registers there.
- **The comparison** (diagnostics on), after each mirrored bind and release: every slot body's id,
  binding and locked-rotation inertia must agree between the two bind tables, and after a bind the body,
  shape and parent ids the two factories returned too. `[SimHost.Shadow] bindCompare … equal=1` at Log, or
  `equal=0` with the differing count at Warning.
- **The hand-over** (`afterTick_Step`, the step's thread): only when the step saved a state
  (`stepAllocatesFrontierSlot`; a Stall step saves none) and only on a worker client. The ring's state for
  the step's tick is copied into a pooled slot of a three-slot `SnapshotChannel` (bytes into storage sized
  at construction, body ids into a vector reserved for every slot body), stamped with a sequence number and
  committed. A tick the ring does not hold counts as missing. The game thread holds at most one slot, so
  the writer always finds a free one; an `OG_CHECK` holds that.
- **The window** (diagnostics on): every `kShadowWindowFrames` (600) frames one `[SimHost.Shadow]
  role=Client …` line at Log with the frames, restores, unchanged frames, refusals, states published and
  missing, hash agreements after a restore, the last restore's occupied slots, tick and the slowest restore,
  the visualization doors' counts, the predicate counters of this thread (shadow-predicate calls, and
  step-predicate calls on the owner thread outside the lock), the bind comparisons so far and the channel's
  drops. The first window counts the thread's predicate calls from zero, the later ones from the previous
  window's end. A restore whose live hash differs from the saved one logs one `[SimHost.Shadow] HASH`
  Warning per window.

## §12 Lines, levels and buffers

| line | level | when |
|---|---|---|
| `[SimHost.Mode]` | Warning | `build`, first |
| `[SimHost.World]` | Warning | `build` |
| `[SimHost.Shadow] built` | Warning | `build`, worker client |
| `[SimHost.Contacts] installed` | Warning | `build`, last |
| `[SimHost.Slot] wait` / `assigned` | Log | `acquire` (the table's) |
| `[SimHost.Bind] bind` / `release` | Warning | `bind` / `release` |
| `[SimHost.Shadow] bindCompare` | Log (`equal=1`), Warning (`equal=0`) | after a mirrored bind or release, diagnostics on |
| `[SimHost.Shadow] HASH` | Warning | after a restore, once per window, diagnostics on |
| `[SimHost.Shadow] role=Client` | Log | every 600 restore frames, diagnostics on |
| `[Verbose][SimHost.Contacts]` | Verbose | `afterRunTick_Step`, counter installed |

- Every format string is the old code's, byte for byte; a line is formatted only when the engine has its
  level enabled.
- The game-thread lines share one buffer and the step thread's contact lines use two others, so the two
  threads never write one buffer. Each is reserved at construction (`kReservedLineChars`,
  `kReservedContactIdsChars`) and formatted with og-simulation's `formatHostLine`, which never truncates and
  grows a buffer only for a line longer than its capacity.

## §13 Tests and poisons

`JoltTickWorldHostTest.cpp` in og-simulation-tests (`[JoltTickWorldHost]`) runs real Jolt worlds with the
tick-driver kit's fake engine and a two-declaration test simulatable:

- `JoltTickWorldHost.Build.InlineHostRegistersAVolumeFromTheConfiguringThreadWithoutTheLock`: an inline
  authority and an inline client register a volume on the step query adapter from the building thread,
  without the lock, right after `build`.
- `JoltTickWorldHost.Build.WorkerClientBuildsAShadowWorldWithTheSameStatics`: the built line with
  `sameStatics=1`, and both worlds' static bodies where a reference world built queryable-first puts them.
- `JoltTickWorldHost.Bind.BindReleaseAndRebindLeaveTheTwoBindTablesEqual` and
  `JoltTickWorldHost.Bind.TheAuthorityPublishesOnlyANewAssignment`.
- `JoltTickWorldHost.Shadow.HandOverPublishesOnlyOnFrontierSavingStepsAndRestoresOncePerSequence`,
  `JoltTickWorldHost.Shadow.InlineHostHasNoHandOverAndNoRestore` and
  `JoltTickWorldHost.Shadow.OverwriteWritesTheRenderPosesIntoTheRestoredShadow` (both diagnostics settings).
- `JoltTickWorldHost.Contacts.OneRunTickWithAFiveTickReplayRecordsSixContactSteps`: a real
  `SimulationStepDriver` with a five-tick replay; six records, five marked replayed, no line-buffer
  allocation at Verbose, nothing formatted at Log.
- `JoltTickWorldHost.Predicates.RefuseANonStepNonLockCallerInWorkerMode`.
- `JoltTickWorldHost.Lines.MatchTheGoldens`: every line above except `HASH` (a consistent pair of worlds
  restores to an equal hash, and the hand-over has no test seam), with both diagnostics settings.
- `JoltTickWorldHost.Statics.SplitByThePhysicsOnlyFlags`.

Poisons seen failing on 2026-10-16 (each applied, built, run and reverted byte-exactly): the thread context
configured after the adapter emplacement (an `OG_CHECK` aborts `build`; with checks compiled out the
inline registration case fails); the physics-only statics built first (the static positions differ from the
reference); a state restored twice (the restore count); the bind mirror or the release mirror deleted, or
the shadow keeping the step's ring depth (the bind-table and line goldens); occupancy applied on the game
thread instead of pushed; the bind outside the lock on a worker client (the query adapter's `OG_CHECK`); the
contact line's buffer released after each `runTick` (the allocation count); the Verbose gate or the replay
flag lost; a hand-over on a Stall step.

## §14 Provenance

Moved from the brawler game's engine module, where the Jolt backend host held the same code in its
JoltBackendUImpl files (historical name; TickDriverBackendUImpl since og-tickdriver task 15b) (og-tickdriver task 6): the runtime lease and the worlds struct with its two
destruction-order asserts, the stepping-mode decision, the world build, the shadow world's build, bind
mirror, release and bind-table comparison, the slot release, the bind, the root-body and bound-body
helpers, the visualization doors, the shadow hand-over and restore with its window, the shadow overwrite,
and the contact recorder and its lines. Its static import split moved here as `splitStatics`. Every
`checkf` became an `OG_CHECK` without printf arguments, and each message names the check it replaced. The
slot table had already moved to og-simulation's `TickSlotTable`. Prohibitions that guarded the moved code
there are now this file's G-01 to G-05 (the guards doc names each predecessor).
