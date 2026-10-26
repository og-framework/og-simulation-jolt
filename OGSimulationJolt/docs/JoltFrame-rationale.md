<!-- SPDX-License-Identifier: MPL-2.0 -->
# `JoltFrame.h` — rationale

The one place og-simulation-jolt decides how a Jolt capsule's axis meets og-simulation's frame.

**If this file and `JoltFrame.h` disagree, the header is authoritative and this file is stale.**

---

## §1 Why it exists

Jolt's capsule lies along its local Y axis (a cylinder of half height `cylinderHalfHeight` along Y,
capped by two hemispheres). og-simulation's capsules (`CapsuleGeometry`, `StaticCapsule`) stand along
the frame's up, `simFrame::kUp` in og-simulation's `SimulationFrame.h`, which is +Y. So every capsule
the backend builds is a plain Jolt capsule, with no wrapper. The three builders, the slot bodies
(`JoltWorld.cpp`), the query volumes (`JoltSpatialQueryAdapter.cpp`) and the static capsules
(`JoltStaticWorldBuilder.cpp`), all call `joltFrame`, so the axis decision is made in one place.

## §2 What it builds

`joltFrame::capsuleShape(cylinderHalfHeight, radius)` takes og-simulation metres (Jolt's unit too) and
returns a `CapsuleShape(cylinderHalfHeight, radius)`. `joltFrame::capsuleShapeFromSettings` builds the
same shape through a `CapsuleShapeSettings` and returns the `JPH::Shape::ShapeResult` of `Create()`; the
static world builder uses it, because its other shape builders also return a `ShapeResult`.

A `static_assert` in the header requires `simFrame::kUp` to be +Y. Under any other up the plain capsule
would lie across the frame instead of along it.

What stays with the callers: turning a total half height into the cylinder half height
(`halfHeight - radius`), and the sphere they build instead when that cylinder half height is not
positive.

History: until the frame flipped to Y-up the header was a template over the frame and, for Unreal's
+Z up, wrapped the capsule in a rotated-translated shape that turned Y into Z (a quarter turn about X).
Stage C of og-sim-metres-yup deleted that branch and the template with it; at +Y up the shipped frame
always took the plain branch, so every shape and every result built on it is unchanged.

## §3 What the tests show

`JoltFrameCapsuleTest.cpp` in og-simulation-tests (tag `[Jolt]`):

- `JoltFrame.CapsuleIsLongestAlongTheFrameUp`: both functions give a shape whose local bounds are
  longest along `simFrame::kUp`, and the same shape type.
- `JoltFrame.TheCapsuleIsAPlainCapsuleShape`: both give a plain Jolt capsule, not a wrapper.

The capsule is also pinned through the determinism fingerprint (`JoltDeterminismFingerprint-rationale.md`):
the probe's two characters are capsules built here.

## §4 Guards

**None.** The frame restriction is a `static_assert`, and the shapes are pinned by the tests above and
by the fingerprint.
