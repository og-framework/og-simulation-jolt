#pragma once
// SPDX-License-Identifier: MPL-2.0
// docs/JoltStateRing-rationale.md

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include "OGSimulationJolt/JoltConfigSidecar.h"
#include "OGSimulationJolt/JoltDefineChecks.h"
#include "OGSimulationJolt/OGJoltExport.h"

#include <Jolt/Physics/Body/BodyID.h>
#include <Jolt/Physics/StateRecorder.h>

#include "OGSimulation/PhysicsWorldAdapter.h"

class JoltFixedStateRecorder final : public JPH::StateRecorder
{
public:
	OGSIMULATIONJOLT_API void beginWrite(uint8_t* buffer, size_t capacity);
	OGSIMULATIONJOLT_API void beginRead(const uint8_t* buffer, size_t byteCount);

	OGSIMULATIONJOLT_API void WriteBytes(const void* data, size_t byteCount) override;
	OGSIMULATIONJOLT_API void ReadBytes(void* data, size_t byteCount) override;
	OGSIMULATIONJOLT_API bool IsEOF() const override;
	OGSIMULATIONJOLT_API bool IsFailed() const override;

	[[nodiscard]] size_t bytesWritten() const { return m_cursor; }
	[[nodiscard]] bool fullyRead() const { return m_cursor == m_size; }

private:
	uint8_t* m_writeBuffer = nullptr;
	const uint8_t* m_readBuffer = nullptr;
	size_t m_size = 0;
	size_t m_cursor = 0;
	bool m_eof = false;
	bool m_failed = false;
};

struct JoltStateSlot
{
	SimTick tick = 0;
	bool valid = false;
	uint32_t byteCount = 0;
	ConfigSidecar sidecar;
	uint64_t stateHash = 0;
	std::vector<JPH::BodyID> bodyIds;
	std::vector<uint8_t> bytes;
};

class JoltStateRing
{
public:
	OGSIMULATIONJOLT_API JoltStateRing(uint32_t depthTicks, uint32_t slotBytes, uint32_t expectedSavedBodies);

	JoltStateRing(const JoltStateRing&) = delete;
	JoltStateRing& operator=(const JoltStateRing&) = delete;

	[[nodiscard]] OGSIMULATIONJOLT_API JoltStateSlot* slotForSave(SimTick tick);
	[[nodiscard]] OGSIMULATIONJOLT_API const JoltStateSlot* find(SimTick tick) const;
	[[nodiscard]] OGSIMULATIONJOLT_API std::optional<SimTick> oldestHeldTick() const;
	OGSIMULATIONJOLT_API void invalidateAll();
	OGSIMULATIONJOLT_API void commit(const JoltStateSlot& source, SimTick tick);

	[[nodiscard]] JoltStateSlot& scratch() { return m_scratch; }
	[[nodiscard]] const JoltStateSlot& scratch() const { return m_scratch; }
	[[nodiscard]] JoltStateSlot& restoreUndo() { return m_restoreUndo; }

	[[nodiscard]] uint32_t depthTicks() const { return static_cast<uint32_t>(m_slots.size()); }
	[[nodiscard]] uint32_t slotBytes() const { return m_slotBytes; }
	[[nodiscard]] OGSIMULATIONJOLT_API uint32_t heldTickCount() const;
	[[nodiscard]] OGSIMULATIONJOLT_API size_t reservedBytes() const;

private:
	static void prepare(JoltStateSlot& slot, uint32_t slotBytes, uint32_t expectedSavedBodies);

	uint32_t m_slotBytes = 0;
	std::vector<JoltStateSlot> m_slots;
	JoltStateSlot m_scratch;
	JoltStateSlot m_restoreUndo;
};
