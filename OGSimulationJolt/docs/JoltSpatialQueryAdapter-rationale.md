<!-- SPDX-License-Identifier: MPL-2.0 -->
# `JoltSpatialQueryAdapter.h` / `JoltSpatialQueryAdapter.cpp` — rationale

og-simulation's `SpatialQueryAdapter` on a `JoltWorld`. Query volumes are standalone Jolt shapes, never
bodies: a query places the volume's shape at a pose and asks the world's narrow phase what it overlaps
(`overlap`) or what it hits first along a displacement (`sweep`). Prohibitions are in
`JoltSpatialQueryAdapter-guards.md`.

**If this file and the sources disagree, the sources are authoritative and this file is stale.**

The adapter reproduces the Chaos-backed query adapter of the Unreal host (UE-sim's
ChaosSpatialQueryAdapter.h/.cpp, not distributed with this repository) and the result semantics of
og-simulation's `SpatialQueryResult.h`. Where this file names that adapter, an Unreal Engine 5.6 source
file or a game header, it is provenance; every behaviour stated here is pinned by a test in
og-simulation-tests (`Jolt/JoltSpatialQueryAdapterTest.cpp`).

---

## §1 Surface

| member | what it does |
|---|---|
| `registerVolume(descriptor, ignoredRootBodyId)` | the engine-free form of the Chaos adapter's registration. The Chaos form took UE query params carrying the owner actor as an ignored actor; here the owner is named by its root body id (`BodyId{}` = none). Returns sequential `QueryVolumeId`s, like Chaos. The descriptor's `searchCategories` is the category set (the task text's third argument) |
| `registerShape(body, shapeIndex, rootBodyId)` | the factory's call (`JoltShapeRegistrarFn`): the host passes it as the `JoltPhysicsFactory` registrar. Records the body's root and returns the body's shape id (§3) |
| `excludeStaticBodiesFromQueries(bodies)` | marks static bodies that queries must not find (PhysicsOnly statics, §8) |
| `overlap`, `sweep`, `setVolumeParentTransform`, `enableShape`, `disableShape` | the concept (§4–§7) |
| `callerMayAccessWorld()` | the access predicate every member asserts (§2) |
| `hitKeyOf(bodyId)` | the peer-stable key of a body (§3), for callers and tests that need to compare hits across peers |

`JoltSpatialQueryConfig::mappedCategories` is the host's category table: the categories the Chaos adapter
was constructed with (the brawler maps six). It is required (an `OG_CHECK` rejects an empty mask)
because the object-query and diagnostic semantics depend on it (§5).

The concept is checked by a `static_assert` at the end of the header.

## §2 Threading: every member asserts its caller

In M1 the Jolt world, and with it this adapter, is touched by the step (on a client's worker task or the
server's game thread) and by the game thread under the world mutex (the join-time bind pass, and the
visualization passes if the host keeps them on Jolt). Every member except the constructor and the trivial
accessors (`callerMayAccessWorld`, `volumeCount`, `mappedCategories`) starts with `OG_CHECK(callerMayAccessWorld(), …)`. `OG_CHECK` is compiled out of shipping
builds, so the check costs nothing there.

`callerMayAccessWorld()` calls `JoltSpatialQueryConfig::accessCheck` when the host supplies one, and
otherwise `JoltWorldAccessScope::isOpenOnThisThread()`. `JoltWorldAccessScope` is an engine-free RAII
marker: a thread-local depth counter. The host opens one wherever it may touch the world (around each
step, and wherever it holds the world mutex), which is the "in step / holds the mutex" flag pair the
rewire design asks for. A host with its own flags passes them as `accessCheck` instead.

The adapter takes no lock itself and reads Jolt through the world's no-lock interfaces, like
`JoltWorld` (the world is single-threaded by contract).

## §3 Identity

**Body id.** A hit's `bodyId` is the Jolt body id (index and sequence), as everywhere in this backend
(`JoltPhysicsBodyAdapter-rationale.md`). Slot bodies are slot-local, so peers that assigned a character
to different slots report different ids for it; nothing below orders by them.

**Owner.** The owner simulatable id and declaration index come from the body's **user data**, written by
`JoltPhysicsFactory` at bind (`joltBodyUserData`). That is "the factory's table" the task text names.
The bind table of `JoltPhysicsBodyAdapter` holds the same binding but is game-thread data in M1, so the
step-thread query does not read it (guard G-04).

**Root.** `registerShape` records each body's root, flattening the parent chain at registration exactly
as the Chaos adapter does (the root of a root is itself). `rootBodyId` is that root, or the body itself
when it was never registered. Statics are their own root.

**Shape id.** `JoltPhysicsFactory::defaultShapeIdOf`: the body's id value, for the body's single shape.
The Chaos adapter numbered shapes sequentially, which grows with every join; fixed ids keep the
`ShapeEnableTable` bits (256) bounded and make a rejoin reuse the slot's ids (guard G-03).

**The peer-stable key** (`JoltQueryHitKey`), in order: slot hits before statics; then owner simulatable
id, declaration index, shape index (0: M1 slot bodies have one shape); for statics, the static body's
index and the element (compound child) index. Static bodies come from `JoltStaticWorldBuilder`, whose body
ids and child order are a pure function of the description (its own rationale), so the static part is
peer-stable too. After the key, a deeper penetration and then the raw sub-shape id break ties, so the
order is fully determined. Every report is sorted by it (guard G-06); the Chaos adapter returned the
engine's order, which the hit loop's order dependence made a cross-peer hazard.

## §4 `overlap`

- **Only the last volume of the list is queried** (guard G-01). The Chaos adapter calls the engine's
  multi sweep once per volume into one array, and the engine resets that array at the start of every
  call (UE 5.6 SceneQuery.cpp, TSceneCastCommonImpWithRetryRequest calls ResetOutHits, which is
  OutHits.Reset()). Only the last call's hits survive. Every brawler caller passes one volume except the
  radial attack, which passes its inner (r) and outer (2r) spheres, so Chaos answers with the outer
  sphere's hits. ⚠ This looks like an accident in the Chaos adapter; it is reproduced because M1 is
  equivalence, and it is a decision for the user whether both adapters should change.
- **The pose is the translation of the parent transform times the offset, with the identity rotation**
  (guard G-02), as the Chaos adapter passes. Box and capsule volumes are axis-aligned; a capsule is
  Z-up (a Jolt capsule is Y-up, so it is wrapped in a rotated shape, as `JoltWorld` does for slot
  bodies). Units: cm at the seam, m in Jolt, converted with `joltUnits`.
- **Shapes:** sphere, box (convex radius 0, so the box is exact, as the Chaos query box is), capsule
  (`CapsuleGeometry::halfHeight` is the total half height; a capsule no taller than its diameter becomes a
  sphere).
- **One hit per body, or per static element.** Jolt reports one result per sub-shape pair: one per
  touched triangle of a mesh, one per child of a compound. The Chaos query reports one per shape. Results
  are reduced to one per key (§3), keeping the deepest.
- **Mesh back faces count** for overlaps (inferred to match Chaos's double-sided triangle queries; no
  brawler volume searches the world today).
- **Fields:** `bodyId`, `rootBodyId` (§3); `objectCategories` = the hit body's layer membership; and
  `objectPosition` (guard G-08). The Chaos adapter reports the hit's actor location, and every component
  its factory creates belongs to the character actor, so for a slot body that is the character's location:
  here, the root (capsule) body's position. For a static it is the element's centre of mass; the Chaos
  value (the static actor's origin) has no counterpart in a compound, and no brawler query searches
  statics with `overlap` today.

## §5 Query modes and category filtering

The Chaos adapter folds `searchCategories` into an object-type set through its category table, dropping
unmapped categories. UE then decides the mode (PhysicsInterfaceUtils.cpp, CreateQueryFilterData): a
non-empty object-type set is an **object query**, an empty one a **trace-channel query** on the volume's
trace channel. The adapter mirrors that with `joltQueryRule` (in the header, with `static_assert`s):

- `effectiveObjectMask` = search & mapped. Non-zero: **object query.** A body matches when its layer's
  categories intersect the mask: **any-of over membership**, through task 10's `JoltQueryLayerFilter`,
  which decodes the layer from the layer table and calls the private `joltCollisionRule::queryMatches`.
  A PARKED layer has no categories and matches nothing.
- Zero: **trace-channel query** on `traceCategory` (guard G-05). This is the documented mode of a
  volume with empty `searchCategories`, and also what a volume whose every search bit is unmapped does
  in Chaos. ⚠ The Chaos adapter's own diagnostic says such a volume "matches NOTHING"; by the engine
  code above it runs as a trace query. The Jolt diagnostic says what happens.
- **Trace responses** (`respondsToTraceCategory`, `blocksTraceCategory`): the Chaos factory gives every
  slot body an Overlap response to every channel and Block to its `blockingCategories`; the importer
  records only Block responses for statics (blockingCategories). So for a trace on category T: a slot
  body always responds and blocks only if it lists T; a static responds (and blocks) only if it lists T;
  a PARKED body never responds. `overlap` reports every responding body; `sweep` considers blocking ones
  only, as the Chaos sweep keeps only blocking hits.

**Diagnostics**, at registration, once per volume, through the injected logger at `[Error]`, with the
Chaos adapter's tags: `[SpatialQuery.EmptyObjectQuery]` (non-empty search, none mapped),
`[SpatialQuery.PartialObjectQuery]` (some mapped, the rest dropped) and
`[SpatialQuery.UnmappedTraceCategory]`. The Chaos adapter's other two tags have no counterpart: there is
no channel table to pad (MapGap), and no per-category channel lookup (UnmappedCategory).

## §6 `sweep`

The volume's shape is cast from the translation of the sweep transform times the offset (guard G-02),
along the displacement, with Jolt's narrow-phase shape cast and the same filters as `overlap`. It never
reads or writes the volume's stored parent transform, as the concept requires.

- **Nearest hit:** the smallest fraction; ties break by the peer-stable key (§3). The Chaos adapter keeps
  the first minimum-time hit in engine order.
- **A sweep that starts inside a target** (the S5 case). The Chaos sweep sets bFindInitialOverlaps, and UE
  converts an initial overlap (CollisionConversions.cpp, ConvertOverlappedShapeToImpactHit) to: time 0,
  start-penetrating, impact normal and penetration depth from the minimum translation distance, impact
  point at the deepest point. As every initial overlap has time 0, it always beats a later hit. The Jolt
  cast reproduces that with the deepest-point option on and back faces enabled (guard G-07): the result
  is `blocked`, `fraction` 0, `startPenetrating`, `penetrationDepth` = the push-out distance in cm, and
  `normal` = the push-out direction (away from the target). This is measured, for upward, downward,
  sideways and zero displacement, and against both a static and a slot body.
- **A zero displacement** (length at most 1e-8 cm, UE's nearly-zero test) is answered by an overlap at
  the start pose and reported as a start-penetrating hit, as UE does with a zero-length sweep.
- **Fields:** `fraction` in [0, 1]; `normal` = minus the normalised Jolt penetration axis (the surface
  normal of the target, pointing at the volume), or +Z if Jolt returns no axis (UE's fallback for an initial overlap);
  `impactPoint` = the contact point on the target, in cm; `penetrationDepth` only when start-penetrating,
  else 0; identity and categories as in §3–§4. A miss is the default `SweepHit` (not blocked,
  fraction 1).

## §7 What a query never returns

A body filter, applied by Jolt before the narrow phase:

- **Unbound or parked slot bodies.** A slot body counts only if its user data carries a binding and its
  layer is not PARKED (guard G-04). A vacant slot is PARKED (`JoltWorld`), so a bound but vacant body is
  excluded by its layer; an occupied but not yet bound body by its user data. `keyOf` repeats the binding
  test, so a body without a binding can never be given a key.
- **Disabled shapes**: a registered body whose `ShapeEnableTable` bit is clear (`enableShape` /
  `disableShape`). The bits live in `JoltWorld::shapeEnables`, are saved in the tick sidecar and restored
  by `restoreTick`, so the first query after a restore sees the restored flags.
- **The ignored owner**: a slot body whose root is the volume's `ignoredRootBodyId`. The Chaos form
  ignored the owner actor, which owns every body of the character, so ignoring by root covers the same
  set (the host must pass `registerShape` as the factory's registrar, or children keep themselves as
  root).
- **PhysicsOnly statics** (§8).

## §8 PhysicsOnly statics: excluded in the query filter

Chaos queries cannot find a static element whose collision is PhysicsOnly. The Unreal importer emits such
elements with their categories (it has no "not queryable" field to set, and none may be added to
og-simulation in M1) and only counts them. Of the two options the task text offers, a query-enable bit
needs a new descriptor field, so the adapter **skips them in the query filter**:
`excludeStaticBodiesFromQueries` takes static body ids, and the body filter drops them. They still
collide (measured: a capsule rests on one).

The host builds the PhysicsOnly elements with a second `JoltStaticWorldBuilder` (static bodies are
grouped into compounds, so one body must not mix queryable and PhysicsOnly elements) and passes that
builder's `stats().bodies`. ⚠ That needs the importer to say **which** elements are PhysicsOnly (today
it counts them only); that is host work (task 18 or the importer), and there are none on ThirdPersonMap.

## §9 Parity with the Chaos adapter

| Chaos adapter behaviour | Jolt adapter | |
|---|---|---|
| Volume shapes: sphere, box, capsule (UE collision shapes, cm) | Jolt sphere, box (convex radius 0), Z-up capsule (m) | equal |
| Pose: translation of parent times offset, identity rotation | same (G-02) | equal |
| `overlap` over several volumes: the engine resets the array per call, only the last volume's hits survive | only the last volume is queried (G-01) | equal ⚠ (likely a Chaos accident; user decision) |
| Mode: object query when the mapped search set is non-empty, else trace channel | same (G-05) | equal |
| Object query: the body's object type (its **first** category's channel) is in the set | any-of over the body's **full** membership | equal for single-category shapes (every brawler shape; asserted by task 10) |
| Unmapped search bits dropped | dropped (`effectiveObjectMask`) | equal |
| Trace query: shape response to the channel (slot bodies Overlap all, Block their blocking set) | same for slot bodies | equal |
| Trace query on statics: their full UE response container | Block responses only (the importer records no others) | ⚠ deviation: a static that overlaps a trace channel without blocking it is not found. No brawler volume traces |
| Unmapped trace category traces on channel 0 (WorldStatic) | answers on the category's own bit; diagnostic says so | ⚠ deviation, diagnosed; not reachable with the shipped table |
| Hit order: engine order | sorted by the peer-stable key (G-06) | intended change (S12) |
| One hit per shape | one per body / static element (deduplicated, deepest kept) | equal |
| `objectCategories`: the reverse-mapped first category | the layer's full membership | equal for single-category shapes |
| `objectPosition`: actor location (GT actor transform, read from the physics thread) | root body's position in the step's own world (G-08) | ⚠ timing: Chaos read the game-thread actor, which lags the physics step; Jolt reads the step's state |
| `objectPosition` of a static: the static actor's origin | the element's centre of mass | ⚠ deviation; no brawler `overlap` searches statics |
| Self-ignore: the owner actor (every component of the character) | every body whose root is `ignoredRootBodyId` | equal when the factory registers through the adapter |
| Disabled shapes: physics-thread query-enable flag, post-filtered | `ShapeEnableTable` bit, filtered before the narrow phase, saved and restored with the tick | equal; Jolt also rolls the flag back (S11) |
| Mid-spawn bodies dropped (no physics-thread view yet) | unbound or parked bodies never returned (G-04) | equal intent |
| Sweep: nearest blocking hit by time, first minimum wins | nearest by fraction, ties by key | equal up to tie order |
| Sweep starting inside: time 0, start-penetrating, MTD normal and depth | same (G-07) | equal (measured) |
| Zero-length sweep: initial overlaps only | answered by an overlap at the start pose | equal |
| PhysicsOnly statics: never found | excluded by the host's list (§8) | equal once the host supplies the list |
| `ShapeId`: sequential per registration | the body's id value; a rejoin reuses it and re-enables it (G-03) | ⚠ ids differ; bounded |
| Threading: physics-thread or game-thread context checks | an access assert on every member (§2) | stricter |
| Diagnostics: EmptyObjectQuery, PartialObjectQuery, UnmappedTraceCategory, MapGap, UnmappedCategory | the first three, same tags, through the logger | MapGap and UnmappedCategory have no counterpart |

## §10 Notes for the host (task 18)

- Construct the adapter with the category mask the Chaos adapter maps, and pass `registerShape` as the
  factory's registrar.
- Open a `JoltWorldAccessScope` around each step and wherever the world mutex is held, or pass the
  host's own predicate as `accessCheck`.
- Volumes are never unregistered (the concept has no such member), as in Chaos: every join adds volumes.
