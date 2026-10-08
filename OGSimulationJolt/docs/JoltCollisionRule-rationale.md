<!-- SPDX-License-Identifier: MPL-2.0 -->
# `JoltCollisionRule.h` — rationale

The pair rule that decides whether two shapes collide, and the match rule for queries, private to
og-simulation-jolt. The header holds the licence, a docs pointer, two `constexpr` functions and their
compile-time truth table; this file carries the why.

**If this file and `JoltCollisionRule.h` disagree, the header is authoritative and this file is
stale.**

---

## §1 What the two functions say

Every shape is **in** a 32-bit set of categories (`ShapeDescriptor::categories` in og-simulation's
`QueryGeometry.h`) and **collides with** a 32-bit set (`ShapeDescriptor::blockingCategories`).

- `joltCollisionRule::shouldCollide(aIn, aWith, bIn, bWith)` is true only when **each** side lists the
  other: `(aWith & bIn) != 0` and `(bWith & aIn) != 0`. This is the **AND** rule.
- `joltCollisionRule::queryMatches(objectIn, queryAnyOf)` is true when the object is in **any** of the
  queried categories.

## §2 Why AND: it is the rule the Chaos build runs today

The Chaos-backed factory of the Unreal host (ChaosPhysicsFactory.cpp, function applyDescriptor, in
the og-simulation-unreal module, which is not part of this repository) sets every channel to overlap,
then sets the channels in `blockingCategories` to block, and gives the shape the object type of its
first category. Chaos's collision filter requires
both shapes to block each other before it creates a contact. So a pair where only one side lists the
other overlaps, and does not collide. Running the same rule in Jolt keeps the Jolt build equivalent to
the Chaos build.

The Chaos side uses only the **first** category bit of a shape; Jolt sees every bit. The two agree as
long as every shape has exactly one category, which every shape of the game this backend was first
built for does. The test `JoltCollisionRule.M1EquivalenceWithChaosForEveryBrawlerShapePair` lists every
shape kind, asserts each is single-category, and compares this rule with a model of the Chaos filter
for every pair; a future multi-category shape fails there instead of diverging silently.

## §3 The planned change, and why the rule lives here

The target model is **OR, one-way**: if only X lists Y, X is blocked and Y is unaffected. Switching to
it changes `shouldCollide` and adds a contact listener that gives the non-listing side infinite mass;
nothing else here or in `JoltLayerTable.h` changes, because the layer table is keyed on the raw sets
and never on the rule's answer.

Until then the rule is private to this repository, so og-simulation core carries no collision
semantics of its own.

## §4 The compile-time truth table

The `static_assert`s in the header pin the AND rule's four cases (both, one side only, neither,
parked) and its symmetry. Changing the body of `shouldCollide` to OR fails the build there with the
message that M1 is the AND rule (witnessed 2026-10-07: MSVC `C2338` at the "one side only" assert).
The test `JoltCollisionRule.ContactsInAWorldFollowTheAndRule` checks the same four cases as real
contacts in a stepped world.

## §5 Guards

**None.** The rule is enforced by the compiler (§4) and by the world-level tests.
