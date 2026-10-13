<!-- SPDX-License-Identifier: MPL-2.0 -->
# `JoltBodyUserData.h` — rationale

The binding a slot body carries (`JoltBodyBinding`) and how it, plus the root mark, is packed into
Jolt's 64-bit body user data (`joltBodyUserData`). The header has no guard tag, so there is no guards
file.

**If this file and `JoltBodyUserData.h` disagree, the header is authoritative and this file is stale.**

---

## §1 Why it is its own header

The helpers depend on nothing but `<cstdint>` and `<optional>`: no Jolt header, no world, no glm. They
used to live in `JoltPhysicsBodyAdapter.h`, so `JoltBodyDefaults.h`, which needs only `isRoot` for its
friction combine (`JoltBodyDefaults-rationale.md` §5), pulled in the adapter, the world and glm with it.
Both now include this header instead. Moving them changed no byte of the helpers: the user data bits,
the friction combine and the determinism fingerprint are the same.

## §2 The binding

`JoltBodyBinding` is the simulatable id and the declaration index a slot body is bound to. It is what
the bind table of `JoltPhysicsBodyAdapter` stores per slot body (`JoltPhysicsBodyAdapter-rationale.md`
§5), and what `decode` returns.

## §3 Body user data

`joltBodyUserData::encode` packs a binding into Jolt's 64-bit body user data: bit 63 = bound, bit 62 =
root (`kRootBit`), bits 8-39 = simulatable id, bits 0-7 = declaration index. `decode` returns nothing
when bit 63 is clear (Jolt's default user data is 0) and ignores bit 62. `encodeUnbound` is the root
bit alone, which the world writes into every slot body at creation; `isRoot` reads it for the friction
combine (`JoltBodyDefaults-rationale.md` §5). `encode` takes `isRoot` as a required argument, so a bind
cannot drop the mark by leaving it out. `static_assert`s pin the round trip at the field limits, with
and without the root bit. User data is body
configuration, not simulation state: Jolt's snapshot does not save it, so it is the same in every
restored tick.

## §4 Who writes and who reads it

- **Writers.** `JoltWorld` writes `encodeUnbound` of the template's `BodyDescriptor::isRoot` when it
  creates each slot body (`JoltWorld-rationale.md`, the root mark). `JoltPhysicsFactory`'s bind writes
  `encode` of the binding and the template's `isRoot`, after an `OG_CHECK` that the world wrote the same
  root mark (`JoltPhysicsFactory-rationale.md` §2).
- **Readers.** `joltBodyDefaults::combineFrictionAverageExceptRoots` reads `isRoot` on both bodies of a
  contact. `JoltSpatialQueryAdapter` reads `decode` for a hit's owner and to skip a slot body that is not
  bound yet (`JoltSpatialQueryAdapter-rationale.md`).
