// SPDX-License-Identifier: MPL-2.0
// docs/JoltStateRing-rationale.md

#include "OGSimulationJolt/JoltStateRing.h"

#include <algorithm>
#include <cstring>

#include "OGSimulation/OGAssert.h"

void JoltFixedStateRecorder::beginWrite(uint8_t* buffer, size_t capacity)
{
	m_writeBuffer = buffer;
	m_readBuffer = nullptr;
	m_size = capacity;
	m_cursor = 0;
	m_eof = false;
	m_failed = false;
}

void JoltFixedStateRecorder::beginRead(const uint8_t* buffer, size_t byteCount)
{
	m_writeBuffer = nullptr;
	m_readBuffer = buffer;
	m_size = byteCount;
	m_cursor = 0;
	m_eof = false;
	m_failed = false;
}

void JoltFixedStateRecorder::WriteBytes(const void* data, size_t byteCount)
{
	if (m_failed || m_writeBuffer == nullptr || byteCount > m_size - m_cursor)
	{
		m_failed = true;
		return;
	}
	std::memcpy(m_writeBuffer + m_cursor, data, byteCount);
	m_cursor += byteCount;
}

void JoltFixedStateRecorder::ReadBytes(void* data, size_t byteCount)
{
	if (m_failed || m_readBuffer == nullptr || byteCount > m_size - m_cursor)
	{
		m_eof = m_readBuffer != nullptr;
		m_failed = true;
		std::memset(data, 0, byteCount);
		return;
	}
	std::memcpy(data, m_readBuffer + m_cursor, byteCount);
	m_cursor += byteCount;
}

bool JoltFixedStateRecorder::IsEOF() const
{
	return m_eof;
}

bool JoltFixedStateRecorder::IsFailed() const
{
	return m_failed;
}

JoltStateRing::JoltStateRing(uint32_t depthTicks, uint32_t slotBytes, uint32_t expectedSavedBodies)
	: m_slotBytes(slotBytes)
	, m_slots(depthTicks)
{
	for (JoltStateSlot& slot : m_slots)
	{
		prepare(slot, slotBytes, expectedSavedBodies);
	}
	prepare(m_scratch, slotBytes, expectedSavedBodies);
	prepare(m_restoreUndo, slotBytes, expectedSavedBodies);
}

void JoltStateRing::prepare(JoltStateSlot& slot, uint32_t slotBytes, uint32_t expectedSavedBodies)
{
	slot.bytes.assign(slotBytes, 0u);
	slot.bodyIds.reserve(expectedSavedBodies);
}

JoltStateSlot* JoltStateRing::slotForSave(SimTick tick)
{
	if (m_slots.empty())
	{
		return nullptr;
	}

	JoltStateSlot* free = nullptr;
	JoltStateSlot* oldest = nullptr;
	for (JoltStateSlot& slot : m_slots)
	{
		if (!slot.valid)
		{
			if (free == nullptr)
			{
				free = &slot;
			}
			continue;
		}
		if (slot.tick == tick)
		{
			return &slot;
		}
		if (oldest == nullptr || slot.tick < oldest->tick)
		{
			oldest = &slot;
		}
	}

	if (free != nullptr)
	{
		return free;
	}
	if (tick < oldest->tick)
	{
		return nullptr;
	}
	oldest->valid = false;
	return oldest;
}

const JoltStateSlot* JoltStateRing::find(SimTick tick) const
{
	for (const JoltStateSlot& slot : m_slots)
	{
		if (slot.valid && slot.tick == tick)
		{
			return &slot;
		}
	}
	return nullptr;
}

std::optional<SimTick> JoltStateRing::oldestHeldTick() const
{
	std::optional<SimTick> oldest;
	for (const JoltStateSlot& slot : m_slots)
	{
		if (slot.valid && (!oldest.has_value() || slot.tick < *oldest))
		{
			oldest = slot.tick;
		}
	}
	return oldest;
}

void JoltStateRing::invalidateAll()
{
	for (JoltStateSlot& slot : m_slots)
	{
		slot.valid = false;
	}
}

void JoltStateRing::commit(const JoltStateSlot& source, SimTick tick)
{
	OG_CHECK(source.valid, "JoltStateRing::commit - the source snapshot holds no state (saveScratch never ran, or it failed)");
	if (!source.valid)
	{
		return;
	}
	JoltStateSlot* target = slotForSave(tick);
	if (target == nullptr)
	{
		return;
	}
	std::copy_n(source.bytes.begin(), source.byteCount, target->bytes.begin());
	target->byteCount = source.byteCount;
	target->bodyIds = source.bodyIds;
	target->sidecar = source.sidecar;
	target->stateHash = source.stateHash;
	target->tick = tick;
	target->valid = true;
}

uint32_t JoltStateRing::heldTickCount() const
{
	return static_cast<uint32_t>(std::count_if(m_slots.begin(), m_slots.end(), [](const JoltStateSlot& slot) { return slot.valid; }));
}

size_t JoltStateRing::reservedBytes() const
{
	const size_t perSlot = sizeof(JoltStateSlot) + m_slotBytes + m_scratch.bodyIds.capacity() * sizeof(JPH::BodyID);
	return perSlot * (m_slots.size() + 2u);
}
