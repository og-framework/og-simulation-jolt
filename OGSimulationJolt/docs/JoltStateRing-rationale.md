<!-- SPDX-License-Identifier: MPL-2.0 -->
# `JoltStateRing.h` / `JoltStateRing.cpp` — rationale

The preallocated per-tick snapshot store behind `JoltWorld`'s `saveTick` / `restoreTick`, and the
fixed-buffer state recorder that writes into it.

**If this file and the sources disagree, the sources are authoritative and this file is stale.**

---

## §1 `JoltFixedStateRecorder`

Jolt saves and restores through a state recorder (a byte stream). Jolt's bundled implementation is
backed by a growing string stream, which allocates on the heap on every save. `JoltFixedStateRecorder`
reads and writes a caller-owned byte span instead:

- `beginWrite(buffer, capacity)` then Jolt's save: bytes are appended; a write that does not fit sets
  the failed flag and writes nothing more. `bytesWritten` is the snapshot size.
- `beginRead(buffer, byteCount)` then Jolt's restore: a read past the end sets the failed and
  end-of-file flags and returns zeros. `fullyRead` says whether the restore consumed every byte.

It has no validating mode (Jolt's own validating recorder compares each read with live memory; it is
a debugging aid that only works on unfiltered saves, so it is not used here).

## §2 `JoltStateSlot`

One slot is `{tick, valid, byteCount, sidecar, stateHash, bodyIds, bytes}`:

- `bytes` is preallocated to `JoltWorldConfig::ringSlotBytes` (16 KiB by default) and never resized;
- `bodyIds` is the list of bodies the snapshot holds, recorded during the save, reserved at
  construction for every slot body. `JoltWorld::restoreTick` checks it against the live body set before
  restoring (`JoltWorld-rationale.md`, the atomic restore section);
- `sidecar` is the `ConfigSidecar` (`JoltConfigSidecar-rationale.md`);
- `stateHash` is the canonical hash of the world when the slot was saved.

## §3 `JoltStateRing`: the ring, the scratch slot and the restore-undo slot

The ring holds `depthTicks` slots, all allocated in the constructor, plus two extra slots that are not
in the ring:

- the **scratch** slot (`scratch`), written by `JoltWorld::saveScratch` and copied into the ring by
  `commit` (`JoltWorld::commitScratch`). The step driver takes it before every normal step and commits
  it only on a Skip, under the skipped tick;
- the **restore-undo** slot (`restoreUndo`), written by `JoltWorld::restoreTick` just before it calls
  Jolt's restore, so a restore that fails half way can put the world back. It is separate from the
  scratch slot so that a restore never overwrites a scratch snapshot the step driver still means to
  commit, whatever order the driver calls them in.

`slotForSave(tick)` implements og-simulation's ring contract (`PhysicsWorldAdapter-rationale.md` in
og-simulation core):

1. a held `tick` is returned for an in-place overwrite (nothing is evicted);
2. otherwise a free slot, if any;
3. otherwise the slot of the **smallest** held tick is evicted and returned — unless `tick` is itself
   smaller than every held tick, in which case nothing is returned and nothing is held (saving a tick
   older than everything in a full ring holds nothing).

A depth of 0 (the authority) returns nothing for every tick, so the authority holds no snapshot.

`find`, `oldestHeldTick` and `heldTickCount` scan the slots (depths are tens of slots).
`invalidateAll` clears every `valid` flag (the HardResync wipe); the bytes stay allocated.

`reservedBytes` reports the memory the ring holds: per slot the slot struct, `ringSlotBytes` and the
reserved body-id list, for `depthTicks + 2` slots.

## §4 Measured sizes (2026-10-07, Jolt 5.6.0, Win64 Development)

With the 6-body brawler slot template, 8 occupied slots (48 non-static bodies), a static floor and
scripted forces, the largest snapshot over ticks 89–120 was **7,313 bytes**
(`JoltWorld.BytesPerSavedTickWithEightOccupiedSlots`), under half of the default 16 KiB slot. A ring
of depth 32 reserved **567,392 bytes** (34 slots). The ring depth a predicting client needs is
`TimeConfig::rollbackWindowHardCap` + 2 (22 with the default cap of 20), about 393 KiB of slot bytes.

A snapshot that does not fit its slot is a configuration error: `JoltWorld` logs it, fails an
`OG_CHECK`, and leaves the tick unheld.

## §5 Guards

**None.** The ring rules are covered by `JoltWorld.RingHoldsEvictsOverwritesAndInvalidates`,
`JoltWorld.AuthorityDepthZeroHoldsNothing` and `JoltWorld.CommitScratchRestoresThePreStepWorld`.
