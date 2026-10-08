<!-- SPDX-License-Identifier: MPL-2.0 -->
# `JoltStaticWorldBuilder.h` / `JoltStaticWorldBuilder.cpp` — rationale

Builds a `JoltWorld`'s static geometry (the arena) from og-simulation's engine-independent
`StaticWorldDescription` (`StaticGeometry.h`). It is the Jolt implementation of the `StaticWorldBuilder`
concept; a `static_assert` at the end of the header checks the signature.

**If this file and the sources disagree, the sources are authoritative and this file is stale.**

---

## §1 Use and lifetime

1. The host imports its level into a `StaticWorldDescription`.
2. The world is built with that description's layer keys:
   `JoltWorldConfig::staticLayers = staticLayerKeysOf(description)` (`JoltLayerTable-rationale.md` §6).
   Layers are allocated once, when the world is built, so the builder cannot add one later.
3. A `JoltStaticWorldBuilder` is made over the world and `build(description)` is called **once**,
   before the first saved tick. A second call fails an `OG_CHECK` and adds nothing (with checks
   compiled out it logs a warning).

The statics are never removed and never saved: `JoltWorld`'s snapshot filter skips static bodies
(`JoltWorld-rationale.md`), and every peer builds the same statics from the same description (§5).

The builder takes its own logger (the same function type as the world's), and an optional
`JoltStaticWorldBuilderOptions`.

## §2 Shape mapping

| `StaticShape` alternative | Jolt shape |
|---|---|
| `StaticBox` | box, with a convex radius (§3) |
| `StaticSphere` | sphere |
| `StaticCapsuleZ` | Jolt capsule (Y axis) inside a rotated-translated shape that turns Y into Z; cylinder half height = `totalHalfHeight` − `radius`. A capsule with `totalHalfHeight` equal to `radius` is a sphere; one with `totalHalfHeight` < `radius` is invalid (§6) |
| `StaticConvexHull` | convex hull of the points, with a convex radius (§3) |
| `StaticTriangleMesh` | mesh shape; the triangles keep the description's winding (guard G-02) |

The shape builder is a visitor over the `StaticShape` variant, so a new alternative does not compile
until it has an arm. A `static_assert` on `kStaticShapeTypeCount` also names the build summary line,
which prints one count per type.

Height fields, cylinders and planes are follow-on 16 (they are not in `StaticShape` yet).

**Units.** Every length (half extents, radii, points, vertices, translations) goes through
`JoltUnits.h` (`joltUnits::centimetresToMetres`). The builder has no conversion factor of its own.
Scale factors are unitless and are not converted.

## §3 Convex radius: the host engine's collision margin

Jolt rounds the edges of boxes and convex hulls by a convex radius: the core shape is shrunk by the
radius and the radius is added back, so faces stay where they are and only edges and corners round
off. The host engine whose levels the first importer reads does the same with a collision margin for
boxes and convexes: by default 5% of the shape's size, capped at 10 cm (DefaultCollisionMarginFraction
= 0.05 and DefaultCollisionMarginMax = 10 in that engine's rigid evolution header). Which size that
engine measures (a full or a half extent) was not checked.

The builder uses the same rule by default (`JoltStaticWorldBuilderOptions` ::
`convexRadiusFractionOfMinExtent` = 0.05, `maxConvexRadiusCm` = 10). The smallest extent is the
smallest full edge of a box, or of the axis-aligned bounds of a hull's points. Jolt can lower the
radius further for a hull it cannot shrink that far. This replaces Jolt's own default (a fixed 5 cm)
so that a capsule sliding over an arena edge meets the same rounding as before. Whether the two engines
then behave the same at an edge is for the parity playtest to judge; nothing here measures it.

## §4 Grouping into bodies, and chunks

Every Jolt body has one object layer, one friction and one restitution. So elements are grouped by:

1. the layer key: (Static, `categories`, `blockingCategories`), one layer per body; and
2. the surface: friction and restitution, compared as bit patterns.

The design named only the first. The second is added because a static compound body has a single
friction and restitution, so two elements with different surfaces in one body would lose one surface.
On ThirdPersonMap every element has the same layer key and the same surface (0.7, 0.3), so its ten
elements become one group, one chunk and one body.

Each group is cut into **chunks** of at most `JoltStaticWorldBuilderOptions` :: `maxShapesPerChunk`
elements (default 256), in sorted order. Each chunk becomes one static compound shape, whose children
carry the elements' placements, and one body at the origin with identity rotation. A chunk of one
element is still built through the static compound settings; Jolt then returns the shape itself, or
the shape inside a rotated-translated shape when it is placed away from the origin.

The chunk limit keeps a compound's children within Jolt's sub-shape id budget (32 bits shared by every
level of nesting; a mesh uses many of them). If Jolt still refuses a chunk, the builder logs it, counts
it in `JoltStaticBuildStats` :: `chunksSplitIntoSingles`, and builds that chunk's elements as one body
each, in the same order.

The design pictured each element wrapped in a rotated-translated shape and then combined. A
compound child already carries a position and a rotation, so the builder places the children directly
and keeps the explicit rotated-translated wrapper only where it is part of the shape (the Z-up
capsule), where a scale must be applied to it (§6).

## §5 Order: the same arena on every peer

The elements are sorted by (layer key, surface, `stableKey`, content hash) (guard G-01), then grouped
and chunked in that order, and the bodies are created in that order through `JoltWorld::createStaticBody`,
which hands out consecutive explicit ids after the slot bodies. Two peers given the same set of
elements in any order therefore build identical chunks, identical child order and identical body ids.

The content hash (FNV-1a 64 over the shape, the transform, both category sets and the surface, bit
exact) only matters when two elements share a `stableKey`. The importer can emit duplicates; the
builder counts them (`duplicateStableKeys`) and the hash orders them by content instead of by their
position in the description. Exact duplicates are interchangeable.

All bodies are then added in **one batch** (Jolt's add-bodies prepare and finalize), not activated
(statics never are), followed by exactly **one** broadphase optimization. One batch and one
optimization give one broadphase layout from one input order. They also make a large arena cheap: Jolt
documents that adding bodies one at a time leaves the broadphase inefficient until it is optimized.

`JoltStaticWorldBuilder.ShuffledDescriptionGivesIdenticalStateAfter300TicksOfRestingCapsules` builds a
20-element arena (all five shape types, two collision pairs, three surfaces, a zero surface, chunks of
three) in its original order, reversed, and in two random shuffles. Each world drops eight brawler
capsules onto it and runs 300 ticks. All four end with the same body ids, the same `stateHash` and
byte-identical full Jolt state.

## §6 Transforms and scale

`localToWorld` is decomposed into a translation (converted to metres), a rotation and a per-axis scale
(the lengths of the matrix's first three columns; a negative determinant puts the sign on X). The
rotation is the normalized quaternion of the matrix with the scale divided out.

The first importer emits rigid transforms: it bakes the host's scale into the shape (the importer's
rationale, transforms section). The builder still accepts a scaled transform. A scale within 1e-4 of
1 on every axis is treated as rigid. Any other scale is applied with Jolt's scale-shape operation, which
picks the nearest scale the shape supports (a sphere only takes a uniform scale). When that differs
from the requested scale the element is counted in `approximatedScales` and logged.

An element is **skipped**, counted in `skippedInvalidShape` and logged when its transform has a zero
or non-finite axis, or when Jolt refuses its shape: a hull Jolt cannot build (for example one with no points), a mesh whose
index count is not a multiple of 3 or which names a vertex out of range, a capsule shorter than its
diameter. Skipping is deterministic, so every peer skips the same elements. Per-element warnings stop
after 16 lines; the counts in the summary line are complete.

## §7 Surface: friction and restitution

Each body takes its group's friction and restitution. How Jolt combines them with a capsule's is set
once for the whole world by the body factory (its parity table), not here.

**Zero surface.** The first importer emits friction = restitution = 0 for a static whose
component has no simple physical material, instead of the host engine's default material. The
description has no field that says "no material", so the builder treats **both values exactly zero** as
"no material". It counts them in `zeroSurfaceElements`, logs one warning line, and by default gives
them `kUnrealDefaultPhysicalMaterialSurface` (friction 0.7, restitution 0.3: the values the host
engine's physical material constructor sets). Setting `JoltStaticWorldBuilderOptions` ::
`surfaceForZeroFrictionAndRestitution` to empty keeps 0/0, and the warning still names them.

A static that really has friction 0 and restitution 0 is changed by the default as well; the warning
line says so. Neither case occurs on ThirdPersonMap, where every static carries the project default
(0.7, 0.3). The clean fix is a material flag in the description, which is follow-on 16 (M1 adds no core
fields).

## §8 Logging and statistics

One summary line per build, without a level prefix: shapes built of shapes given, per type, bodies,
groups, chunks, splits, seconds, and every skip and approximation count. Problems are separate lines
starting with "[Warning] ", as `JoltWorld` does. `JoltStaticBuildStats` keeps the same counts and the
static body ids in creation order, so a host or a test can read them without parsing the log.
`StaticWorldBuildReport` (the concept's return value) carries the per-type counts and the build time.

## §9 Evidence (2026-10-07, Win64 Development)

Catch2 cases, tags `[Jolt][StaticGeometry]`, in `JoltStaticWorldBuilderTest.cpp`:

- `JoltStaticWorldBuilder.EachShapeTypeIsQueryableAtTheExpectedPlace`: one element of each type, placed
  with a translation and (except the sphere and capsule) a rotation. Rays from above hit the expected
  top within 0.05 cm and miss just outside. The capsule's top is at centre + `totalHalfHeight` (so it is
  Z-up), and a ray along X meets its side at centre + radius. The mesh is hit from above and not from below.
- `JoltStaticWorldBuilder.GroupsByCollisionPairAndSurfaceIntoStaticBodiesOnTheirLayer`: three groups and
  four chunks become four static, inactive bodies with consecutive ids, each on a Static-class layer with
  its group's surface; seven children in total.
- `JoltStaticWorldBuilder.ZeroFrictionAndRestitutionGetTheUnrealDefaultSurfaceAndAreLogged`: §7, both
  option settings.
- `JoltStaticWorldBuilder.ScaleIsAppliedOrApproximatedAndInvalidElementsAreSkipped`: §6.
- `JoltStaticWorldBuilder.ShuffledDescriptionGivesIdenticalStateAfter300TicksOfRestingCapsules`: §5.
- `JoltStaticWorldBuilder.ThreeHundredElementsBuildWithinBound`: 300 elements (60 of each type, two
  collision pairs, two surfaces) build in about 2 ms; the test allows 1 s.

## §10 Limits and follow-ons

- The arena is rebuilt from the description on every peer. Cooking it to a binary shape blob is a
  follow-on.
- Elements the host marks query-only or physics-only arrive with their categories like any other; what
  queries may find is the spatial query adapter's decision.
- Height fields, cylinders and planes: follow-on 16, as is a "no material" flag (§7).

## §11 Guards

- **G-01** — the sort order of the elements (§5).
- **G-02** — the triangle winding (§2).
