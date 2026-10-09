<!-- SPDX-License-Identifier: MPL-2.0 -->
# `JoltPhysicsFactory.h` / `JoltPhysicsFactory.cpp` — rationale

og-simulation's `PhysicsObjectFactory` on a `JoltWorld`: it **binds** an existing slot body to a
simulatable's declaration and gives it the Chaos-parity configuration. It creates no Jolt body: the
world's body set is fixed at construction (`JoltWorld-rationale.md`). Prohibitions are in
`JoltPhysicsFactory-guards.md`.

**If this file and the sources disagree, the sources are authoritative and this file is stale.**

---

## §1 Shape of the factory

One factory per registration, like the Chaos factory (constructed per character with that character's
owner and root). It is constructed with:

- the body adapter (for the world and the bind table),
- the slot template the world was built from (`JoltWorldConfig::slotTemplate`; copied, six small
  descriptors),
- the slot the host's allocator assigned and the simulatable's id,
- `JoltPhysicsFactoryOptions` and an optional shape registrar (§5).

`createPhysicalObject(descriptor, name)` binds the **next** template body of the slot. The host calls it
once per declaration in the physics composite's declaration order, which is the template order; the
factory checks that each descriptor equals its template body's (`sameDescriptor`, an `OG_CHECK`) so a
reordering fails loud instead of binding a weapon to a guard body. `name` is unused (the Chaos factory
names the UE component with it).

`parentBodyId` is the slot body of the template's single `isRoot` declaration (the character capsule),
which the host stores as every declaration's parent. The constructor checks there is exactly one.

## §2 What a bind writes

For the slot body, inside one body write lock:

- friction and restitution: `kAdoptedRootMaterial` for the root, `kCreatedBodyMaterial` for every other
  body (guard G-01; `JoltBodyDefaults-rationale.md` §2);
- body user data: the binding (`joltBodyUserData::encode`);
- linear and angular damping, max angular and max linear velocity from `JoltBodyDefaults.h`;
- mass and inertia from `joltBodyDefaults::chaosParityMassPropertiesOf`, with the body's allowed
  degrees of freedom (all, or translation only for a `lockRotation` body);
- then the motion quality (§4).

It checks, and does not set, what the world already decided at construction: the allowed DOFs match
the descriptor's `lockRotation`, and the body may not sleep.

It never writes the object layer, gravity factor, position or velocity (guard G-02): the world owns
them through occupancy and the snapshot ring. A body bound while its slot is vacant stays PARKED until
the occupancy that the step driver applies says otherwise.

Everything a bind writes is body **configuration**, which Jolt's snapshot does not save. That is safe
because every slot gets the same values for the same template body, so a bind never makes two peers or
two restored ticks differ (`JoltBodyAdapter.BoundBodyConfigurationSurvivesARestore`). Unbound slot bodies
keep Jolt's defaults until their first bind; they are parked, so no contact or query sees them.

Then the bind table records the binding and the locked-rotation inertia
(`JoltPhysicsBodyAdapter-rationale.md` §5). From the next poll the host's "resolvable?" question answers
true.

## §3 Combine rules

The factory sets no combine rule. The world's friction and restitution combine functions,
`joltBodyDefaults::combineFrictionAverage` / `combineRestitutionAverage` (Chaos's effective rule,
`JoltBodyDefaults-rationale.md` §2), are set once by the `JoltWorld` constructor
(`JoltWorld-rationale.md` §2). This constructor used to set them, idempotently, on every bind; the
rule is per physics system and affects statics too, so it moved to the world, where a world with no
bound body has it as well.

## §4 Motion quality

`JoltPhysicsFactoryOptions::linearCastCharacters` gives the root body (the character) Jolt's
`LinearCast` motion quality; every other body is `Discrete`. The default is off, which is Chaos parity:
the character capsule's body instance does not enable CCD.

**The default stays off, decided by the determinism and cost suite (2026-10-07).** Measured with
`JoltSuite.TunnellingAtTheMaximumKnockbackSpeedWithAndWithoutLinearCast`: the brawler capsule (radius
42 cm) driven at a constant speed into a 1 cm box wall, a 10 cm box wall and a zero-thickness triangle
mesh wall, from 16 starting positions spread over one tick of travel, 20 ticks each:

| Speed (cm/s) | Travel per tick | Discrete: tunnelled / max penetration | LinearCast: tunnelled / max penetration |
|---|---|---|---|
| 2000 (the maximum knockback) | 33.3 cm | 0 of 48 / 31.2 cm | 0 of 48 / 2.0 cm |
| 2500 | 41.7 cm | 0 of 48 / 39.6 cm | 0 of 48 / 2.0 cm |
| 2828 (knockback plus terminal fall, both 2000) | 47.1 cm | 32 of 48 (every 1 cm box and mesh run; the 10 cm box held, 45 cm deep) | 0 of 48 / 2.0 cm |
| 4000 to 16000 | 66.7 to 266.7 cm | 48 of 48 | 0 of 48 / 2.0 cm |

So `Discrete` does not tunnel at the brawler's maximum knockback speed: it tunnels only once a tick's
travel exceeds the capsule radius (above about 2520 cm/s), which the game reaches only when a
2000 cm/s knockback and a 2000 cm/s terminal fall combine against a surface facing that combined
direction. It does sink up to 31 cm into a wall for a tick at 2000 cm/s before the contact pushes the
capsule out.

Today's Chaos build runs the same regime: the capsule's body instance has neither CCD nor
movement-aware collision detection (both off by default), and Chaos looks ahead of a moving body by
its cull distance (3 cm) plus at most 3 cm of velocity expansion (the default
p.Chaos.Solver.Collision.MaxVelocityBoundsExpansion), so it also lets a fast capsule penetrate and,
past about the radius per tick, pass. This is read from the Chaos source, not measured. M1 is an
equivalence milestone, so the factory keeps `Discrete`.

`LinearCast` is ready if gate 22 shows a capsule passing through geometry: it never tunnelled, it
costs nothing measurable with 8 characters (step median 0.011 ms with or without it), and
`JoltSuite.LinearCastCharactersAreBitExactUnderAlwaysResims` shows it is bit-exact under rollback.

## §5 Shape ids

Each declaration's shapes get a `ShapeId`. Without a registrar the id is `defaultShapeIdOf`: the body's
`BodyId` value (one shape per M1 slot body, ids 1 to 48, inside the world's 256-entry shape-enable
table). With a registrar (the spatial query adapter's engine-free registerShape, a later addition) the
factory calls it as the Chaos factory calls its query adapter: root body for every created body,
std::nullopt for the root itself.

## §6 Parity with the Chaos factory

| Chaos factory behaviour (UE-sim ChaosPhysicsFactory.cpp, applyDescriptor / createPhysicalObject) | Jolt factory |
|---|---|
| Creates a UE component per declaration; adopts the actor's capsule for the `isRoot` one | Binds the next fixed slot body; the root is the template's `isRoot` body. No body is created |
| simulatePhysics → simulated or kinematic | The world built the body Dynamic or Kinematic from the same flag |
| enableGravity | The world's gravity factor (1 or 0) applied with occupancy |
| lockRotation → three rotation locks + a SixDOF joint | Allowed DOFs = translation only (exact; no joint drift) |
| Collision: first category bit → object type, Block on `blockingCategories` | The world's layer table (both sides list each other, AND); every brawler shape is single-category, so they agree |
| BodyResimPolicy → Chaos resim type | Ignored: the Jolt world restores the whole world and replays every body (FullResim for all) |
| Child bodies attached to the capsule in the scene graph, not welded (they simulate) | No constraint; `parentBodyId` is data, and the simulation places each child from it every tick |
| Mass, inertia, damping, material, max angular velocity: UE / Chaos defaults | The `JoltBodyDefaults.h` constants and formula, equal to those defaults |

## §7 Guards

- **G-01** — the root takes `kAdoptedRootMaterial`, never the created-body material.
- **G-02** — a bind never writes the layer, gravity factor, position or velocity.
