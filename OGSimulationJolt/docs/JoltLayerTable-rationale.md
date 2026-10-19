<!-- SPDX-License-Identifier: MPL-2.0 -->
# `JoltLayerTable.h` / `JoltLayerTable.cpp` — rationale

How og-simulation's two 32-bit category sets become Jolt object layers, broadphase layers and filters.

**If this file and the sources disagree, the sources are authoritative and this file is stale.**

<!-- lint-external-ref: PhysicsSystem::Init -- vendored Jolt v5.6.0 under ThirdParty/, which the lint does not index -->

---

## §1 Why a table, not a mask

Jolt gives each body one object layer of `JPH_OBJECT_LAYER_BITS` bits (16 in this library). Jolt's
own mask-based filter splits those bits into a "group" half and a "mask" half (8 + 8 bits), so it
cannot hold two 32-bit sets. Instead each distinct key gets an object layer number from a table:

`JoltLayerKey` = (`JoltBroadPhaseClass`, `categories`, `blockingCategories`).

The table is **lossless** (the full key is stored and looked up per layer) and **rule-agnostic**: it
never stores whether two layers collide. The answer is computed by `joltCollisionRule::shouldCollide`
on the two keys (`JoltCollisionRule-rationale.md`), so switching the rule changes no layer number.

## §2 The broadphase class is part of the key

Jolt derives a body's broadphase layer from its object layer alone. Two bodies with the same category
sets but different motion types must therefore get different object layers, so the class is part of
the key. There are two classes, and the class is the body's motion type:

- `JoltBroadPhaseClass::Static` — static geometry;
- `JoltBroadPhaseClass::Moving` — every slot body, dynamic or kinematic.

There is no query-only class. Under the planned OR one-way rule, a body that collides with nothing can
still be hit by others, so "collides with nothing" is not a property of the body alone and cannot pick
a broadphase class.

## §3 Allocation is deterministic and happens once

`JoltLayerTable`'s constructor takes every key the world will ever use, adds the PARKED key, sorts
them (`JoltLayerKey` compares class, then categories, then blocking set), removes duplicates, and
numbers them in that order. The keys come from the slot template and from the static description
(`staticLayerKeysOf`) when the world is built (`JoltWorld-rationale.md`); none are added later. Two
peers that build from the same template and description therefore get identical layer numbers
whatever order the keys arrive in (`JoltLayerTable.AllocatesDeterministicallyAndIncludesParked`
shuffles them). Layers are configuration, not simulated state, and are never saved in a snapshot.

`layerOf` on a key the table was not built with fails an `OG_CHECK`; with checks compiled out it
returns the PARKED layer, so the body collides with nothing rather than with the wrong things.

## §4 PARKED

`kParkedLayerKey` = (Moving, 0, 0). It is in no category and collides with nothing, so under AND (and
under OR) it makes no contact, and `queryMatches` never matches it. A vacant slot's bodies sit on this
layer.

## §5 The three Jolt interfaces

The table owns three small adapters that Jolt's `PhysicsSystem::Init` takes by reference:

- `JoltBroadPhaseLayers` maps a layer to its class (two broadphase layers);
- `JoltObjectLayerPairFilter` calls `JoltLayerTable::shouldCollide`, which calls the rule;
- `JoltObjectVsBroadPhaseFilter` answers whether a layer can collide with **any** layer of a class.
  The answer is precomputed per layer from the rule at construction (`mayCollideWithClass`), so it is
  exact for the rule in force and lets the broadphase skip a whole class (for PARKED, both classes).

The adapters hold a reference to the table, so the table is neither copyable nor movable; `JoltWorld`
keeps it as a member that outlives the `PhysicsSystem`.

`JoltQueryLayerFilter` is the object-layer filter for queries: it accepts a layer when
`joltCollisionRule::queryMatches` says the layer's categories match the query's any-of set. The
spatial query adapter is its main user; `JoltWorld.VacantSlotIsInvisibleToAWorldCollideShape` uses it
to show that a vacant slot is invisible to a query.

## §6 Statics

`staticLayerKeysOf` turns each static shape descriptor into a Static-class key. Static chunks are later
grouped by (categories, blockingCategories), so every static body has exactly one key.

## §7 Guards

**None.** Determinism of the numbering and the PARKED layer are tested; the filters are tested against
the rule.
