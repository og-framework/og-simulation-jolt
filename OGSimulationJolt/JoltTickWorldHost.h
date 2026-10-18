#pragma once
// SPDX-License-Identifier: MPL-2.0
// docs/JoltTickWorldHost-rationale.md · docs/JoltTickWorldHost-guards.md

#include <algorithm>
#include <atomic>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <optional>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include "OGSimulationJolt/JoltContactCounter.h"
#include "OGSimulationJolt/JoltDefineChecks.h"
#include "OGSimulationJolt/JoltLayerTable.h"
#include "OGSimulationJolt/JoltPhysicsBodyAdapter.h"
#include "OGSimulationJolt/JoltPhysicsBodyReaderAdapter.h"
#include "OGSimulationJolt/JoltPhysicsFactory.h"
#include "OGSimulationJolt/JoltRuntime.h"
#include "OGSimulationJolt/JoltSpatialQueryAdapter.h"
#include "OGSimulationJolt/JoltStateRing.h"
#include "OGSimulationJolt/JoltWorld.h"
#include "OGSimulationJolt/OGJoltExport.h"

#include <Jolt/Physics/Body/BodyID.h>

#include "glm/gtc/quaternion.hpp"
#include "glm/mat4x4.hpp"
#include "glm/vec3.hpp"
#include "OGSimulation/BodyId.h"
#include "OGSimulation/BodySlotOccupancy.h"
#include "OGSimulation/Mailbox.h"
#include "OGSimulation/OGAssert.h"
#include "OGSimulation/PCTimeManagement/TimeConfig.h"
#include "OGSimulation/QueryGeometry.h"
#include "OGSimulation/RenderInterpolation.h"
#include "OGSimulation/SimulationComposite.h"
#include "OGSimulation/SimulationTickBackend.h"
#include "OGSimulation/SimulationTimeContext.h"
#include "OGSimulation/StaticGeometry.h"
#include "OGSimulation/StepHooks.h"
#include "OGSimulation/TickDriverLog.h"
#include "OGSimulation/TickDriverThreadContext.h"
#include "OGSimulation/TickSlotTable.h"

template <typename E>
concept JoltTickHostEngine = TickSlotLogTarget<E> && requires(const E& engine)
{
	{ E::kHostDiagnostics } -> std::convertible_to<bool>;
	typename std::bool_constant<static_cast<bool>(E::kHostDiagnostics)>;
	{ engine.nowSeconds() }  -> std::same_as<double>;
	{ engine.frameNumber() } -> std::same_as<uint64_t>;
};

namespace joltTickHostEngineSelfCheck
{
struct Conforming
{
	static constexpr bool kHostDiagnostics = true;
	double nowSeconds() const;
	uint64_t frameNumber() const;
	bool logEnabled(HostLogLevel) const;
	void log(HostLogLevel, const char*);
};

struct RuntimeDiagnosticsSwitch : Conforming
{
	static bool kHostDiagnostics;
};

struct MissingClock
{
	static constexpr bool kHostDiagnostics = true;
	uint64_t frameNumber() const;
	bool logEnabled(HostLogLevel) const;
	void log(HostLogLevel, const char*);
};

static_assert(JoltTickHostEngine<Conforming>,
	"JoltTickHostEngine self-check: the reference shape (TickDriverEngine's clock, frame number, log and diagnostics switch) "
	"must satisfy the concept");
static_assert(!JoltTickHostEngine<RuntimeDiagnosticsSwitch>,
	"JoltTickHostEngine self-check: kHostDiagnostics must be a constant expression, because every diagnostic of the world host "
	"is behind if constexpr on it");
static_assert(!JoltTickHostEngine<MissingClock>,
	"JoltTickHostEngine self-check: nowSeconds is required (the bind's lock wait and the shadow restore are timed with it)");
} // namespace joltTickHostEngineSelfCheck

struct JoltTickWorldHostConfig
{
	bool isAuthority = false;
	bool runInline = false;
	uint32_t clientRingDepthTicks = static_cast<uint32_t>(TimeConfig{}.rollbackWindowHardCap) + 2u;
	glm::vec3 gravity = JoltWorldConfig{}.gravityCmPerS2;
	bool contactCounts = false;
	uint32_t mappedCategories = 0u;
	JoltRuntime::LogFn log = nullptr;
};

struct JoltTickStatics
{
	StaticWorldDescription queryable;
	StaticWorldDescription physicsOnly;
};

struct JoltTickStaticBodies
{
	std::vector<JPH::BodyID> queryable;
	std::vector<JPH::BodyID> physicsOnly;

	std::vector<JPH::BodyID> all() const
	{
		std::vector<JPH::BodyID> bodies = queryable;
		bodies.insert(bodies.end(), physicsOnly.begin(), physicsOnly.end());
		return bodies;
	}
};

namespace joltTickWorldHost
{
inline constexpr uint32_t kShadowTempAllocatorBytes = 64u * 1024u;
inline constexpr uint32_t kShadowWindowFrames       = 600u;
inline constexpr size_t   kReservedLineChars        = 1024u;
inline constexpr size_t   kReservedContactIdsChars  = 512u;

[[nodiscard]] OGSIMULATIONJOLT_API JoltTickStatics splitStatics(const StaticWorldDescription& description,
	const std::vector<uint8_t>& physicsOnly);
[[nodiscard]] OGSIMULATIONJOLT_API uint8_t rootDeclarationIndexOf(const std::vector<SlotBodyTemplate>& slotTemplate);
[[nodiscard]] OGSIMULATIONJOLT_API JoltWorldConfig stepWorldConfigOf(const std::vector<SlotBodyTemplate>& slotTemplate,
	const StaticWorldDescription& queryableStatics, const StaticWorldDescription& physicsOnlyStatics, uint32_t ringDepthTicks,
	const glm::vec3& gravity);
[[nodiscard]] OGSIMULATIONJOLT_API JoltTickStaticBodies buildStatics(JoltWorld& world,
	const StaticWorldDescription& queryableStatics, const StaticWorldDescription& physicsOnlyStatics, JoltRuntime::LogFn log);
[[nodiscard]] OGSIMULATIONJOLT_API BodyId slotRootBodyId(const JoltWorld& world, const std::vector<SlotBodyTemplate>& slotTemplate,
	uint32_t slot);
[[nodiscard]] OGSIMULATIONJOLT_API uint32_t boundBodyCount(const JoltWorld& world, const JoltBodyBindTable& bindTable);
OGSIMULATIONJOLT_API void prepareShadowStateSlot(JoltStateSlot& state, size_t bodyCapacity);
OGSIMULATIONJOLT_API void copySavedState(const JoltStateSlot& saved, JoltStateSlot& target);

template <typename Composite>
struct declarationCountOf;

template <typename... Declarations>
struct declarationCountOf<SimulationComposite<Declarations...>>
	: std::integral_constant<size_t, sizeof...(Declarations)>
{
};

template <typename SimulatableT>
using PhysicsCompositeOf = std::remove_cvref_t<decltype(std::declval<const SimulatableT&>().getPhysicsComposite())>;

template <typename SimulatableT>
std::vector<SlotBodyTemplate> slotTemplateOf()
{
	std::vector<SlotBodyTemplate> slotTemplate;
	const PhysicsCompositeOf<SimulatableT> composite{};
	composite.forEach([&slotTemplate](const auto& declaration)
	{
		using D = std::decay_t<decltype(declaration)>;
		slotTemplate.push_back(SlotBodyTemplate{ D::descriptor(), static_cast<uint8_t>(slotTemplate.size()) });
	});
	return slotTemplate;
}

struct NoShadowRestoreNote
{
	void operator()(double, bool) const {}
};
} // namespace joltTickWorldHost

struct JoltTickRuntimeLease
{
	explicit JoltTickRuntimeLease(JoltRuntime::LogFn logger) : runtime(JoltRuntime::acquire(logger)) {}
	~JoltTickRuntimeLease() { JoltRuntime::release(); }

	JoltTickRuntimeLease(const JoltTickRuntimeLease&) = delete;
	JoltTickRuntimeLease& operator=(const JoltTickRuntimeLease&) = delete;

	JoltRuntime& runtime;
};

struct JoltTickWorlds
{
	std::optional<JoltTickRuntimeLease>          runtime;
	std::optional<JoltContactCounter>            contactCounter;
	std::optional<JoltWorld>                     stepWorld;
	std::optional<JoltWorld>                     shadowWorld;
	std::optional<JoltPhysicsBodyAdapter>        shadowBodyAdapter;
	std::optional<JoltPhysicsBodyReaderAdapter>  shadowReader;
	std::optional<JoltSpatialQueryAdapter>       shadowQuery;
	std::vector<SlotBodyTemplate>                slotTemplate;
};

static_assert(__builtin_offsetof(JoltTickWorlds, runtime) < __builtin_offsetof(JoltTickWorlds, contactCounter)
		&& __builtin_offsetof(JoltTickWorlds, contactCounter) < __builtin_offsetof(JoltTickWorlds, stepWorld),
	"JoltTickWorlds members destruct in REVERSE declaration order. The JoltRuntime lease must outlive the world (the last "
	"JoltRuntime::release unregisters Jolt's types and deletes its Factory), and the step world holds a raw pointer to the "
	"contact counter (its Jolt contact listener) until it is destroyed, so the counter must outlive the world. Keep runtime < "
	"contactCounter < stepWorld; the step world's adapters are the session's, declared after the world host. Was the first "
	"destruction-order static_assert of the brawler's JoltBackendWorldsUImpl.");

static_assert(__builtin_offsetof(JoltTickWorlds, runtime) < __builtin_offsetof(JoltTickWorlds, shadowWorld)
		&& __builtin_offsetof(JoltTickWorlds, shadowWorld) < __builtin_offsetof(JoltTickWorlds, shadowBodyAdapter)
		&& __builtin_offsetof(JoltTickWorlds, shadowBodyAdapter) < __builtin_offsetof(JoltTickWorlds, shadowReader)
		&& __builtin_offsetof(JoltTickWorlds, shadowBodyAdapter) < __builtin_offsetof(JoltTickWorlds, shadowQuery),
	"JoltTickWorlds members destruct in REVERSE declaration order. The game-thread shadow world must outlive its own adapters "
	"(the reader holds the shadow body adapter's bind table), and the JoltRuntime lease must outlive the shadow world as it "
	"outlives the step world. Keep runtime < shadowWorld < shadowBodyAdapter < {shadowReader, shadowQuery}. Was the second "
	"destruction-order static_assert of the brawler's JoltBackendWorldsUImpl.");

template <typename SimulatableT, typename LockT, JoltTickHostEngine EngineT, typename LineAllocatorT = std::allocator<char>>
	requires BasicLockable<LockT>
class JoltTickWorldHost
{
public:
	using World     = JoltWorld;
	using Config    = JoltTickWorldHostConfig;
	using WorldLock = WorldLockScope<LockT, JoltWorldAccessScope>;
	using Line      = BasicHostLine<LineAllocatorT>;

	struct ShadowWindow
	{
		uint32_t frames               = 0u;
		uint32_t restores             = 0u;
		uint32_t unchanged            = 0u;
		uint32_t refused              = 0u;
		uint32_t hashEqual            = 0u;
		uint32_t hashDiffer           = 0u;
		uint32_t occupiedSlots        = 0u;
		uint64_t vizQueryShadow       = 0u;
		uint64_t vizQueryStep         = 0u;
		uint64_t vizReaderShadow      = 0u;
		uint64_t vizReaderStep        = 0u;
		uint64_t shadowQueryAccessesAtStart         = 0u;
		uint64_t stepQueryOwnerThreadAccessesAtStart = 0u;
		uint64_t publishedAtStart     = 0u;
		uint64_t missingAtStart       = 0u;
		double   maxRestoreMs         = 0.0;
		uint32_t lastRestoredTick     = 0u;
	};

	explicit JoltTickWorldHost(EngineT& engine, const LineAllocatorT& lineAllocator = LineAllocatorT())
		: m_engine(engine)
		, m_slots(engine)
		, m_line(lineAllocator)
		, m_contactLine(lineAllocator)
		, m_contactIds(lineAllocator)
	{
		m_line.reserve(joltTickWorldHost::kReservedLineChars);
		m_contactLine.reserve(joltTickWorldHost::kReservedLineChars);
		m_contactIds.reserve(joltTickWorldHost::kReservedContactIdsChars);
	}

	JoltTickWorldHost(const JoltTickWorldHost&) = delete;
	JoltTickWorldHost& operator=(const JoltTickWorldHost&) = delete;

	void build(const Config& config, const StaticWorldDescription& queryableStatics,
		const StaticWorldDescription& physicsOnlyStatics, std::optional<JoltPhysicsBodyAdapter>& bodyAdapter,
		std::optional<JoltPhysicsBodyReaderAdapter>& readerAdapter, std::optional<JoltSpatialQueryAdapter>& queryAdapter)
	{
		const bool stepsInline = config.isAuthority || config.runInline;
		m_threadContext.template configure<kDiagnostics>(
			config.isAuthority ? tickDriverThread::kAuthorityRoleBit : tickDriverThread::kClientRoleBit, stepsInline);

		if (m_engine.logEnabled(HostLogLevel::Warning))
			m_engine.log(HostLogLevel::Warning, formatHostLine(m_line, "[SimHost.Mode] role=%s stepping=%s viz=%s",
				config.isAuthority ? "Authority" : "Client",
				stepsInline ? "inline" : "worker",
				stepsInline ? "stepWorld" : "shadowWorld"));

		m_log = config.log;
		m_mappedCategories = config.mappedCategories;
		m_worlds.runtime.emplace(config.log);

		m_worlds.slotTemplate = joltTickWorldHost::slotTemplateOf<SimulatableT>();
		m_rootDeclarationIndex = joltTickWorldHost::rootDeclarationIndexOf(m_worlds.slotTemplate);

		const JoltWorldConfig worldConfig = joltTickWorldHost::stepWorldConfigOf(m_worlds.slotTemplate, queryableStatics,
			physicsOnlyStatics, config.isAuthority ? 0u : config.clientRingDepthTicks, config.gravity);

		std::vector<JPH::BodyID> stepStaticBodies;
		{
			WorldLock worldLock(m_worldLock, m_threadContext);

			m_worlds.stepWorld.emplace(m_worlds.runtime->runtime, worldConfig, config.log);
			if (config.contactCounts)
			{
				m_worlds.contactCounter.emplace();
				m_contactSteps_Step.reserve(worldConfig.ringDepthTicks + 1u);
				m_worlds.stepWorld->physics().SetContactListener(&*m_worlds.contactCounter);
				m_worlds.stepWorld->setAfterStepObserver(&JoltTickWorldHost::onStepWorldStepped_Step, this);
			}

			const JoltTickStaticBodies statics =
				joltTickWorldHost::buildStatics(*m_worlds.stepWorld, queryableStatics, physicsOnlyStatics, config.log);

			OG_CHECK(m_threadContext.isOwnerThreadForRole() && m_threadContext.stepWorldPredicate() != nullptr,
				"JoltTickWorldHost::build: the thread context must be configured before the session's adapters are emplaced, so "
				"the query adapter gets this role's access predicate and an inline host's game thread may register without the "
				"lock. Was the order of the brawler's decideJoltSteppingMode before buildJoltWorld (BL1).");
			bodyAdapter.emplace(*m_worlds.stepWorld);
			readerAdapter.emplace(*m_worlds.stepWorld, bodyAdapter->bindTable());
			queryAdapter.emplace(*m_worlds.stepWorld, config.log, JoltSpatialQueryConfig{
				config.mappedCategories,
				m_threadContext.stepWorldPredicate() });
			queryAdapter->excludeStaticBodiesFromQueries(statics.physicsOnly);

			stepStaticBodies = statics.all();

			if (m_engine.logEnabled(HostLogLevel::Warning))
				m_engine.log(HostLogLevel::Warning, formatHostLine(m_line,
					"[SimHost.World] slots=%u bodiesPerSlot=%u ringDepthTicks=%u statics=%d physicsOnlyStatics=%d staticBodies=%d physicsOnlyBodies=%d",
					m_worlds.stepWorld->simulatableSlots(), m_worlds.stepWorld->bodiesPerSlot(), worldConfig.ringDepthTicks,
					static_cast<int32_t>(queryableStatics.shapes.size()), static_cast<int32_t>(physicsOnlyStatics.shapes.size()),
					static_cast<int32_t>(statics.queryable.size()),
					static_cast<int32_t>(statics.physicsOnly.size())));
		}

		m_bodyAdapter   = &bodyAdapter;
		m_readerAdapter = &readerAdapter;
		m_queryAdapter  = &queryAdapter;

		if (!stepsInline)
			buildShadowWorld(worldConfig, queryableStatics, physicsOnlyStatics, stepStaticBodies);

		const bool stepListener   = m_worlds.stepWorld->physics().GetContactListener() != nullptr;
		const bool shadowListener = m_worlds.shadowWorld.has_value() && m_worlds.shadowWorld->physics().GetContactListener() != nullptr;
		if (m_engine.logEnabled(HostLogLevel::Warning))
			m_engine.log(HostLogLevel::Warning, formatHostLine(m_line, "[SimHost.Contacts] installed=%d stepListener=%s shadowListener=%s reserved=%u",
				m_worlds.contactCounter.has_value() ? 1 : 0, stepListener ? "set" : "null",
				!m_worlds.shadowWorld.has_value() ? "none" : shadowListener ? "set" : "null",
				static_cast<uint32_t>(m_contactSteps_Step.capacity())));
		OG_CHECK(stepListener == m_worlds.contactCounter.has_value() && !shadowListener,
			"JoltTickWorldHost::build: the contact counter must be the step world's listener exactly when the contact counts were "
			"on at world build, and the game-thread shadow world never has one. Was the contact-listener checkf of the brawler's "
			"buildJoltWorld.");
	}

	template <typename ManagerT>
	void bindIntegratedTickSource(const ManagerT& manager)
	{
		m_tickSource = &manager;
		m_integratedTick = [](const void* source)
		{
			return static_cast<uint32_t>(static_cast<const ManagerT*>(source)->currentIntegratedTick());
		};
	}

	template <typename PublishF>
	std::optional<uint32_t> acquire(uint64_t key, const char* name, bool isAuthority, std::optional<uint32_t> replicatedSlot,
		PublishF&& publish)
	{
		const bool alreadyHeld = m_slots.slotOf(key).has_value();
		const std::optional<uint32_t> slot = m_slots.acquire(key, name, isAuthority, replicatedSlot);
		if (slot && isAuthority && !alreadyHeld)
			publish(*slot);
		return slot;
	}

	template <typename PushOccupancyF>
	std::optional<uint32_t> release(uint64_t key, unsigned int id, PushOccupancyF&& pushOccupancy)
	{
		const std::optional<uint32_t> slot = m_slots.release(key);
		if (!slot.has_value())
			return std::nullopt;

		// ⛔G-02  docs/JoltTickWorldHost-guards.md
		pushOccupancy(*slot, false);
		(*m_bodyAdapter)->bindTable().releaseSlot(*slot);

		if (m_engine.logEnabled(HostLogLevel::Warning))
			m_engine.log(HostLogLevel::Warning, formatHostLine(m_line, "[SimHost.Bind] release id=%u slot=%u boundBodies=%u slotsInUse=%d",
				id, *slot, joltTickWorldHost::boundBodyCount(*m_worlds.stepWorld, (*m_bodyAdapter)->bindTable()),
				static_cast<int32_t>(m_slots.size())));

		// ⛔G-05  docs/JoltTickWorldHost-guards.md
		if (m_worlds.shadowWorld.has_value())
			releaseShadowSlot(*slot);
		return slot;
	}

	std::optional<uint32_t> slotOf(uint64_t key) const { return m_slots.slotOf(key); }
	size_t slotsInUse() const { return m_slots.size(); }

	template <typename StaticDataT, typename OnWorldLockWaitF>
	void bind(uint32_t slot, unsigned int id, SimulatableT& simulatable, const StaticDataT& staticData,
		OnWorldLockWaitF&& onWorldLockWait)
	{
		BodyId parentBodyId;
		{
			const double worldLockWaitStartSeconds = m_engine.nowSeconds();
			// ⛔G-01  docs/JoltTickWorldHost-guards.md
			WorldLock worldLock(m_worldLock, m_threadContext);
			onWorldLockWait(m_engine.nowSeconds() - worldLockWaitStartSeconds);

			JoltPhysicsFactory factory(**m_bodyAdapter, m_worlds.slotTemplate, slot, id, JoltPhysicsFactoryOptions{},
				[this](BodyId body, uint32_t shapeIndex, std::optional<BodyId> rootBodyId)
				{
					return (*m_queryAdapter)->registerShape(body, shapeIndex, rootBodyId);
				});
			parentBodyId = factory.parentBodyId();

			simulatable.editPhysicsComposite().forEach([&](auto& decl)
			{
				using D = std::decay_t<decltype(decl)>;
				const auto& subStaticData = D::staticDataOf(staticData);
				auto r = factory.createPhysicalObject(D::descriptor(), D::name);
				decl.bindings.ownBodyId        = r.bodyId;
				decl.bindings.parentBodyId     = parentBodyId;
				decl.bindings.attachmentOffset = D::attachmentOffset(subStaticData);
				decl.bindings.shapeIds         = std::move(r.shapeIds);
				for (const auto& volDesc : D::queryVolumes(subStaticData))
					decl.bindings.queryVolumeIds.push_back((*m_queryAdapter)->registerVolume(volDesc, parentBodyId));
			});
		}

		if (m_engine.logEnabled(HostLogLevel::Warning))
			m_engine.log(HostLogLevel::Warning, formatHostLine(m_line, "[SimHost.Bind] bind id=%u slot=%u body0=%u boundBodies=%u slotsInUse=%d",
				id, slot, m_worlds.stepWorld->slotBodyId(slot, 0u).GetIndex(),
				joltTickWorldHost::boundBodyCount(*m_worlds.stepWorld, (*m_bodyAdapter)->bindTable()),
				static_cast<int32_t>(m_slots.size())));

		// ⛔G-04  docs/JoltTickWorldHost-guards.md
		if (m_worlds.shadowWorld.has_value())
			bindShadowSlot(slot, id, simulatable);
	}

	JoltSpatialQueryAdapter& editVizQuery()
	{
		if (m_threadContext.stepsInline())
		{
			if constexpr (kDiagnostics)
				++m_shadowWindow.vizQueryStep;
			return m_queryAdapter->value();
		}
		if constexpr (kDiagnostics)
			++m_shadowWindow.vizQueryShadow;
		return m_worlds.shadowQuery.value();
	}

	const JoltPhysicsBodyReaderAdapter& getVizReader() const
	{
		if (m_threadContext.stepsInline())
		{
			if constexpr (kDiagnostics)
				++m_shadowWindow.vizReaderStep;
			return m_readerAdapter->value();
		}
		if constexpr (kDiagnostics)
			++m_shadowWindow.vizReaderShadow;
		return m_worlds.shadowReader.value();
	}

	template <typename PublishF>
	std::optional<QueryVolumeId> registerVizVolume(const QueryVolumeDescriptor& descriptor, uint64_t key, const char* name,
		bool isAuthority, std::optional<uint32_t> replicatedSlot, PublishF&& publish)
	{
		const std::optional<uint32_t> slot = acquire(key, name, isAuthority, replicatedSlot, std::forward<PublishF>(publish));
		if (!slot.has_value())
			return std::nullopt;

		if (!m_threadContext.stepsInline())
			return m_worlds.shadowQuery->registerVolume(descriptor,
				joltTickWorldHost::slotRootBodyId(*m_worlds.shadowWorld, m_worlds.slotTemplate, *slot));

		return (*m_queryAdapter)->registerVolume(descriptor,
			joltTickWorldHost::slotRootBodyId(*m_worlds.stepWorld, m_worlds.slotTemplate, *slot));
	}

	World& stepWorld() { return *m_worlds.stepWorld; }
	const World& stepWorld() const { return *m_worlds.stepWorld; }
	bool hasShadowWorld() const { return m_worlds.shadowWorld.has_value(); }
	uint8_t rootDeclarationIndex() const { return m_rootDeclarationIndex; }
	const TickDriverThreadContext& threadContext() const { return m_threadContext; }

	WorldLock lockForStep() { return WorldLock(m_worldLock, m_threadContext); }

	void afterTick_Step(const TickOutcome& outcome)
	{
		if (!m_worlds.shadowWorld.has_value() || !stepAllocatesFrontierSlot(outcome.kind))
			return;

		const JoltStateSlot* const saved = m_worlds.stepWorld->ring().find(outcome.tick);
		if (saved == nullptr)
		{
			m_shadowSlotsMissing.fetch_add(1u, std::memory_order_relaxed);
			return;
		}

		ShadowStateSlot* const target = m_shadowSlots.beginWrite();
		OG_CHECK(target != nullptr,
			"JoltTickWorldHost::afterTick_Step: the shadow hand-over channel has no free slot. The game thread holds at most one "
			"slot (the newest, for one restore), and a 3-slot channel keeps a free slot for one. Was the checkf of the brawler's "
			"publishShadowSlot_Step.");
		if (target == nullptr)
			return;

		joltTickWorldHost::copySavedState(*saved, target->state);

		target->sequence = m_shadowSlotsPublished.load(std::memory_order_relaxed) + 1u;
		m_shadowSlotsPublished.store(target->sequence, std::memory_order_relaxed);
		m_shadowSlots.commit();
	}

	void afterRunTick_Step(uint64_t physicsStep, const TickOutcome& outcome)
	{
		if (m_worlds.contactCounter.has_value())
			logContactSteps_Step(physicsStep, outcome);
	}

	template <typename OnRestoreF = joltTickWorldHost::NoShadowRestoreNote>
	void restore_GameThread(uint64_t frameNumber, OnRestoreF&& onRestore = {})
	{
		if (!m_worlds.shadowWorld.has_value())
			return;

		ShadowWindow& window = m_shadowWindow;
		++window.frames;
		m_shadowRestoredFrame = frameNumber;

		if (const ShadowStateSlot* const newest = m_shadowSlots.peekNewest(0))
		{
			if (newest->sequence > m_shadowRestoredSequence)
			{
				const double startSeconds = m_engine.nowSeconds();
				const bool restored = m_worlds.shadowWorld->restoreFromSnapshot(newest->state);
				const double restoreSeconds = m_engine.nowSeconds() - startSeconds;
				const double restoreMs = restoreSeconds * 1000.0;
				m_shadowRestoredSequence = newest->sequence;
				onRestore(restoreSeconds, !restored);
				if (restored)
				{
					++window.restores;
					window.lastRestoredTick = static_cast<uint32_t>(newest->state.tick);
					window.maxRestoreMs = std::max(window.maxRestoreMs, restoreMs);
					if constexpr (kDiagnostics)
					{
						window.occupiedSlots = static_cast<uint32_t>(newest->state.sidecar.occupancy.occupied.count());
						if (m_worlds.shadowWorld->liveStateHash() == newest->state.stateHash)
						{
							++window.hashEqual;
						}
						else if (window.hashDiffer++ == 0u)
						{
							if (m_engine.logEnabled(HostLogLevel::Warning))
								m_engine.log(HostLogLevel::Warning, formatHostLine(m_line,
									"[SimHost.Shadow] HASH role=Client tick=%u: the restored shadow world's state hash differs from "
									"the step world's saved tick",
									static_cast<uint32_t>(newest->state.tick)));
						}
					}
				}
				else
				{
					++window.refused;
				}
			}
			else
			{
				++window.unchanged;
			}
			m_shadowSlots.release(newest);
		}
		else
		{
			++window.unchanged;
		}

		if constexpr (kDiagnostics)
		{
			if (window.frames >= joltTickWorldHost::kShadowWindowFrames)
				logShadowWindow_GameThread();
		}
	}

	template <typename PosesT, typename ForEachTargetSlotF, typename VisitorF>
	void overwriteShadowPoses_GameThread(const PosesT& poses, ForEachTargetSlotF&& forEachTargetSlot,
		[[maybe_unused]] VisitorF&& visitor)
	{
		if (!m_worlds.shadowWorld.has_value())
			return;
		OG_CHECK(m_shadowRestoredFrame == m_engine.frameNumber(),
			"JoltTickWorldHost::overwriteShadowPoses_GameThread: the shadow world was not restored this frame before the render "
			"poses were written into it. The restore loads the newest saved tick into every slot body, so a restore after this "
			"overwrite would leave the visualizations querying the newest tick instead of the rendered pose. Was the checkf of "
			"the brawler's overwriteShadowPoses_GameThread.");

		const JoltBodyBindTable& shadowBindTable = m_worlds.shadowBodyAdapter->bindTable();
		forEachTargetSlot([&](unsigned int id, uint32_t slot)
		{
			for (uint32_t declarationIndex = 0u; declarationIndex < m_worlds.shadowWorld->bodiesPerSlot(); ++declarationIndex)
			{
				const RenderBody* const pose = findRenderBody(poses, id, static_cast<uint8_t>(declarationIndex));
				if (pose == nullptr)
					continue;
				const BodyId body = JoltPhysicsBodyAdapter::bodyIdOf(m_worlds.shadowWorld->slotBodyId(slot, declarationIndex));
				if (!shadowBindTable.isBound(body))
					continue;

				glm::mat4 transform = pose->hasRotation != 0u ? glm::mat4_cast(pose->rotation)
				                                              : m_worlds.shadowBodyAdapter->getBodyTransform(body);
				transform[3] = glm::vec4(pose->positionCm, 1.f);
				m_worlds.shadowBodyAdapter->setBodyTransform(body, transform);

				if constexpr (kDiagnostics)
					visitor(id, declarationIndex, *pose, [this, body]() { return m_worlds.shadowReader->getBodyTransform(body); });
			}
		});
	}

	class Diagnostics
	{
	public:
		explicit Diagnostics(const JoltTickWorldHost& host) : m_host(host) {}

		const ShadowWindow& shadowWindow() const { return m_host.m_shadowWindow; }
		uint64_t shadowSlotsPublished() const { return m_host.m_shadowSlotsPublished.load(std::memory_order_relaxed); }
		uint64_t shadowSlotsMissing() const { return m_host.m_shadowSlotsMissing.load(std::memory_order_relaxed); }
		uint64_t shadowRestoredSequence() const { return m_host.m_shadowRestoredSequence; }
		uint32_t shadowBindCompares() const { return m_host.m_shadowBindCompares; }
		uint32_t shadowBindMismatches() const { return m_host.m_shadowBindMismatches; }
		size_t contactStepsReserved() const { return m_host.m_contactSteps_Step.capacity(); }
		const JoltWorld* shadowWorld() const { return m_host.m_worlds.shadowWorld ? &*m_host.m_worlds.shadowWorld : nullptr; }
		const JoltPhysicsBodyAdapter* shadowBodyAdapter() const
		{
			return m_host.m_worlds.shadowBodyAdapter ? &*m_host.m_worlds.shadowBodyAdapter : nullptr;
		}
		const std::vector<SlotBodyTemplate>& slotTemplate() const { return m_host.m_worlds.slotTemplate; }

	private:
		const JoltTickWorldHost& m_host;
	};

	Diagnostics getDiagnostics() const { return Diagnostics(*this); }

private:
	static constexpr bool kDiagnostics = static_cast<bool>(EngineT::kHostDiagnostics);

	struct ShadowStateSlot
	{
		ShadowStateSlot()
		{
			joltTickWorldHost::prepareShadowStateSlot(state,
				kMaxSimulatableSlots * joltTickWorldHost::declarationCountOf<joltTickWorldHost::PhysicsCompositeOf<SimulatableT>>::value);
		}

		JoltStateSlot state;
		uint64_t      sequence = 0u;
	};

	struct ContactStep
	{
		uint32_t              tick      = 0u;
		uint64_t              worldStep = 0u;
		JoltContactStepCounts counts;
	};

	void buildShadowWorld(const JoltWorldConfig& stepConfig, const StaticWorldDescription& queryableStatics,
		const StaticWorldDescription& physicsOnlyStatics, const std::vector<JPH::BodyID>& stepStaticBodies)
	{
		OG_CHECK(m_worlds.stepWorld->ring().slotBytes() == JoltWorldConfig{}.ringSlotBytes,
			"JoltTickWorldHost::buildShadowWorld: the step world's ring slots must hold JoltWorldConfig's default ringSlotBytes, "
			"the size the shadow hand-over's pooled slots are pre-reserved to. Keep the two equal, or a hand-over allocates. Was "
			"the ring-slot checkf of the brawler's buildShadowWorld.");

		JoltWorldConfig shadowConfig = stepConfig;
		shadowConfig.ringDepthTicks     = 0u;
		shadowConfig.tempAllocatorBytes = joltTickWorldHost::kShadowTempAllocatorBytes;

		// ⛔G-03  docs/JoltTickWorldHost-guards.md
		m_worlds.shadowWorld.emplace(m_worlds.runtime->runtime, shadowConfig, m_log);

		const JoltTickStaticBodies statics =
			joltTickWorldHost::buildStatics(*m_worlds.shadowWorld, queryableStatics, physicsOnlyStatics, m_log);

		m_worlds.shadowBodyAdapter.emplace(*m_worlds.shadowWorld);
		m_worlds.shadowReader.emplace(*m_worlds.shadowWorld, m_worlds.shadowBodyAdapter->bindTable());
		m_worlds.shadowQuery.emplace(*m_worlds.shadowWorld, m_log, JoltSpatialQueryConfig{
			m_mappedCategories,
			m_threadContext.shadowPredicate() });
		m_worlds.shadowQuery->excludeStaticBodiesFromQueries(statics.physicsOnly);

		const std::vector<JPH::BodyID> shadowStaticBodies = statics.all();

		const uint32_t stepBodies = m_worlds.stepWorld->simulatableSlots() * m_worlds.stepWorld->bodiesPerSlot()
			+ static_cast<uint32_t>(stepStaticBodies.size());
		const uint32_t shadowBodies = m_worlds.shadowWorld->simulatableSlots() * m_worlds.shadowWorld->bodiesPerSlot()
			+ static_cast<uint32_t>(shadowStaticBodies.size());
		const bool sameStatics = shadowStaticBodies == stepStaticBodies;
		if (m_engine.logEnabled(HostLogLevel::Warning))
			m_engine.log(HostLogLevel::Warning, formatHostLine(m_line,
				"[SimHost.Shadow] built bodies step=%u shadow=%u sameStatics=%d slots=%u bodiesPerSlot=%u ringDepthTicks=%u tempAllocatorBytes=%u",
				stepBodies, shadowBodies, sameStatics ? 1 : 0, m_worlds.shadowWorld->simulatableSlots(),
				m_worlds.shadowWorld->bodiesPerSlot(), shadowConfig.ringDepthTicks, shadowConfig.tempAllocatorBytes));
		if constexpr (kDiagnostics)
		{
			OG_CHECK(stepBodies == shadowBodies && sameStatics,
				"JoltTickWorldHost::buildShadowWorld: the shadow world must hold the step world's body count and static body ids. "
				"It must be built by the same calls, in the same order, from the same configuration except the ring depth and the "
				"temp allocator; its restore checks only the slot bodies, so a missing or extra static would go unnoticed there. "
				"Was the non-shipping body-count checkf of the brawler's buildShadowWorld.");
		}
	}

	void bindShadowSlot(uint32_t slot, unsigned int id, const SimulatableT& simulatable)
	{
		JoltPhysicsFactory factory(*m_worlds.shadowBodyAdapter, m_worlds.slotTemplate, slot, id, JoltPhysicsFactoryOptions{},
			[this](BodyId body, uint32_t shapeIndex, std::optional<BodyId> rootBodyId)
			{
				return m_worlds.shadowQuery->registerShape(body, shapeIndex, rootBodyId);
			});

		bool sameResults = true;
		simulatable.getPhysicsComposite().forEach([&](const auto& decl)
		{
			using D = std::decay_t<decltype(decl)>;
			const JoltPhysicsFactory::PhysicalObjectResult result = factory.createPhysicalObject(D::descriptor(), D::name);
			sameResults = sameResults && result.bodyId == decl.bindings.ownBodyId && result.shapeIds == decl.bindings.shapeIds
				&& factory.parentBodyId() == decl.bindings.parentBodyId;
		});

		compareShadowBindTables_GameThread("bind", slot, id, sameResults);
	}

	void releaseShadowSlot(uint32_t slot)
	{
		m_worlds.shadowBodyAdapter->bindTable().releaseSlot(slot);
		compareShadowBindTables_GameThread("release", slot, 0u, true);
	}

	void compareShadowBindTables_GameThread([[maybe_unused]] const char* after, [[maybe_unused]] uint32_t slot,
		[[maybe_unused]] unsigned int id, [[maybe_unused]] bool sameResults)
	{
		if constexpr (kDiagnostics)
		{
			const JoltBodyBindTable& stepTable   = (*m_bodyAdapter)->bindTable();
			const JoltBodyBindTable& shadowTable = m_worlds.shadowBodyAdapter->bindTable();
			const bool sameShape = stepTable.simulatableSlots() == shadowTable.simulatableSlots()
				&& stepTable.bodiesPerSlot() == shadowTable.bodiesPerSlot();
			const uint32_t slots = std::min(stepTable.simulatableSlots(), shadowTable.simulatableSlots());
			const uint32_t bodiesPerSlot = std::min(stepTable.bodiesPerSlot(), shadowTable.bodiesPerSlot());

			uint32_t bound = 0u;
			uint32_t differing = 0u;
			for (uint32_t slotIndex = 0u; slotIndex < slots; ++slotIndex)
			{
				for (uint32_t templateIndex = 0u; templateIndex < bodiesPerSlot; ++templateIndex)
				{
					const BodyId stepBody   = JoltPhysicsBodyAdapter::bodyIdOf(m_worlds.stepWorld->slotBodyId(slotIndex, templateIndex));
					const BodyId shadowBody = JoltPhysicsBodyAdapter::bodyIdOf(m_worlds.shadowWorld->slotBodyId(slotIndex, templateIndex));
					const std::optional<JoltBodyBinding> stepBinding   = stepTable.bindingOf(stepBody);
					const std::optional<JoltBodyBinding> shadowBinding = shadowTable.bindingOf(shadowBody);
					bound += stepBinding.has_value() ? 1u : 0u;
					const bool same = stepBody == shadowBody && stepBinding == shadowBinding
						&& (!stepBinding.has_value()
							|| stepTable.lockedRotationInertiaOf(stepBody) == shadowTable.lockedRotationInertiaOf(shadowBody));
					differing += same ? 0u : 1u;
				}
			}

			const bool equal = sameShape && differing == 0u && sameResults;
			++m_shadowBindCompares;
			if (equal)
			{
				if (m_engine.logEnabled(HostLogLevel::Log))
					m_engine.log(HostLogLevel::Log, formatHostLine(m_line,
						"[SimHost.Shadow] bindCompare after=%s id=%u slot=%u equal=1 boundBodies=%u compares=%u mismatches=%u",
						after, id, slot, bound, m_shadowBindCompares, m_shadowBindMismatches));
			}
			else
			{
				++m_shadowBindMismatches;
				if (m_engine.logEnabled(HostLogLevel::Warning))
					m_engine.log(HostLogLevel::Warning, formatHostLine(m_line,
						"[SimHost.Shadow] bindCompare after=%s id=%u slot=%u equal=0 boundBodies=%u differing=%u sameShape=%d "
						"sameResults=%d compares=%u mismatches=%u: the shadow world's bind table differs from the step world's",
						after, id, slot, bound, differing, sameShape ? 1 : 0, sameResults ? 1 : 0, m_shadowBindCompares,
						m_shadowBindMismatches));
			}
		}
	}

	void logShadowWindow_GameThread()
	{
		if constexpr (kDiagnostics)
		{
			const ShadowWindow& window = m_shadowWindow;
			const uint64_t published = m_shadowSlotsPublished.load(std::memory_order_relaxed);
			const uint64_t missing   = m_shadowSlotsMissing.load(std::memory_order_relaxed);
			const uint64_t shadowQueryAccesses = TickDriverThreadContext::shadowQueryAccessesOnThisThread();
			const uint64_t stepQueryOwnerThreadAccesses = TickDriverThreadContext::stepQueryOwnerThreadAccessesOnThisThread();
			if (m_engine.logEnabled(HostLogLevel::Log))
				m_engine.log(HostLogLevel::Log, formatHostLine(m_line,
					"[SimHost.Shadow] role=Client frames=%u restores=%u unchanged=%u refused=%u published=%llu missing=%llu "
					"hashEqual=%u hashDiffer=%u occupiedSlots=%u lastTick=%u maxRestoreMs=%.4f vizQuery=[shadow:%llu step:%llu] "
					"vizReader=[shadow:%llu step:%llu] shadowQueryAccesses=%llu stepQueryGameThreadAccesses=%llu bindCompares=%u "
					"bindMismatches=%u drops=%llu",
					window.frames, window.restores, window.unchanged, window.refused,
					static_cast<unsigned long long>(published - window.publishedAtStart),
					static_cast<unsigned long long>(missing - window.missingAtStart),
					window.hashEqual, window.hashDiffer, window.occupiedSlots,
					window.lastRestoredTick, window.maxRestoreMs,
					static_cast<unsigned long long>(window.vizQueryShadow), static_cast<unsigned long long>(window.vizQueryStep),
					static_cast<unsigned long long>(window.vizReaderShadow), static_cast<unsigned long long>(window.vizReaderStep),
					static_cast<unsigned long long>(shadowQueryAccesses - window.shadowQueryAccessesAtStart),
					static_cast<unsigned long long>(stepQueryOwnerThreadAccesses - window.stepQueryOwnerThreadAccessesAtStart),
					m_shadowBindCompares, m_shadowBindMismatches, static_cast<unsigned long long>(m_shadowSlots.drops())));

			ShadowWindow next;
			next.publishedAtStart                    = published;
			next.missingAtStart                      = missing;
			next.shadowQueryAccessesAtStart          = shadowQueryAccesses;
			next.stepQueryOwnerThreadAccessesAtStart = stepQueryOwnerThreadAccesses;
			m_shadowWindow = next;
		}
	}

	static void onStepWorldStepped_Step(void* host)
	{
		static_cast<JoltTickWorldHost*>(host)->recordContactStep_Step();
	}

	void recordContactStep_Step()
	{
		OG_CHECK(m_threadContext.isStepRunningOnThisThread(),
			"JoltTickWorldHost::recordContactStep_Step: the step world stepped outside a step scope. The contact lines take each "
			"step's replay flag from its place in runTick's batch, so every step world step must run inside it. Was the first "
			"checkf of the brawler's recordContactStep_Step.");
		OG_CHECK(m_contactSteps_Step.size() < m_contactSteps_Step.capacity(),
			"JoltTickWorldHost::recordContactStep_Step: one runTick stepped the world more often than reserved at world build "
			"(the ring depth's replays plus the forward step). Was the second checkf of the brawler's recordContactStep_Step.");
		OG_CHECK(m_integratedTick != nullptr,
			"JoltTickWorldHost::recordContactStep_Step: bindIntegratedTickSource was not called before the first step with the "
			"contact counts on. Was the brawler host's read of its manager's currentIntegratedTick.");
		m_contactSteps_Step.push_back(ContactStep{ m_integratedTick(m_tickSource),
			m_worlds.stepWorld->physicsStepCount(), m_worlds.contactCounter->drain() });
	}

	void logContactSteps_Step(uint64_t physicsStep, [[maybe_unused]] const TickOutcome& outcome)
	{
		const uint32_t steps = static_cast<uint32_t>(m_contactSteps_Step.size());
		OG_CHECK(steps == outcome.replayedTicks + 1u,
			"JoltTickWorldHost::logContactSteps_Step: one runTick must drain exactly the step driver's replayed ticks plus one "
			"forward step. Every step but the batch's last is logged as replayed. Was the checkf of the brawler's "
			"logContactSteps_Step.");
		if (m_engine.logEnabled(HostLogLevel::Verbose))
		{
			for (uint32_t index = 0u; index < steps; ++index)
			{
				const ContactStep&           step   = m_contactSteps_Step[index];
				const JoltContactStepCounts& counts = step.counts;
				if (counts.manifolds() == 0u && counts.unrecordedIds == 0u && counts.unrecordedDeclarations == 0u)
					continue;
				m_contactIds.clear();
				for (uint32_t idIndex = 0u; idIndex < counts.idCount; ++idIndex)
				{
					const JoltContactIdCounts& id = counts.ids[idIndex];
					char piece[128];
					const int length = std::snprintf(piece, sizeof(piece), "%s%u:%u/%u/%u/%u/%u:0x%x", idIndex == 0u ? "" : ",",
						id.simulatableId, id.pairCount(JoltContactPairClass::RootRoot), id.pairCount(JoltContactPairClass::RootChild),
						id.pairCount(JoltContactPairClass::RootStatic), id.pairCount(JoltContactPairClass::ChildChild),
						id.pairCount(JoltContactPairClass::ChildStatic), id.childDeclarations);
					if (length > 0)
						m_contactIds.append(piece, std::min(static_cast<size_t>(length), sizeof(piece) - 1u));
				}
				m_engine.log(HostLogLevel::Verbose, formatHostLine(m_contactLine,
					"[Verbose][SimHost.Contacts] tick=%u step=%llu worldStep=%llu replay=%d manifolds=%u added=%u "
					"rootRoot=%u rootChild=%u rootStatic=%u childChild=%u childStatic=%u staticStatic=%u ids=%s "
					"idsDropped=%u declarationsDropped=%u",
					step.tick, static_cast<unsigned long long>(physicsStep), static_cast<unsigned long long>(step.worldStep),
					index + 1u < steps ? 1 : 0, counts.manifolds(), counts.added,
					counts.pairCount(JoltContactPairClass::RootRoot), counts.pairCount(JoltContactPairClass::RootChild),
					counts.pairCount(JoltContactPairClass::RootStatic), counts.pairCount(JoltContactPairClass::ChildChild),
					counts.pairCount(JoltContactPairClass::ChildStatic), counts.pairCount(JoltContactPairClass::StaticStatic),
					counts.idCount == 0u ? "-" : m_contactIds.c_str(), counts.unrecordedIds, counts.unrecordedDeclarations));
			}
		}
		m_contactSteps_Step.clear();
	}

	EngineT&                                        m_engine;
	JoltTickWorlds                                  m_worlds;
	TickSlotTable                                   m_slots;
	LockT                                           m_worldLock;
	TickDriverThreadContext                         m_threadContext;
	JoltRuntime::LogFn                              m_log = nullptr;
	uint32_t                                        m_mappedCategories = 0u;
	uint8_t                                         m_rootDeclarationIndex = 0u;
	std::optional<JoltPhysicsBodyAdapter>*          m_bodyAdapter   = nullptr;
	std::optional<JoltPhysicsBodyReaderAdapter>*    m_readerAdapter = nullptr;
	std::optional<JoltSpatialQueryAdapter>*         m_queryAdapter  = nullptr;
	const void*                                     m_tickSource = nullptr;
	uint32_t                                        (*m_integratedTick)(const void*) = nullptr;
	Line                                            m_line;

	SnapshotChannel<ShadowStateSlot, 3>             m_shadowSlots;
	std::atomic<uint64_t>                           m_shadowSlotsPublished{0u};
	std::atomic<uint64_t>                           m_shadowSlotsMissing{0u};
	uint64_t                                        m_shadowRestoredSequence = 0u;
	uint64_t                                        m_shadowRestoredFrame    = 0u;
	uint32_t                                        m_shadowBindCompares     = 0u;
	uint32_t                                        m_shadowBindMismatches   = 0u;
	mutable ShadowWindow                            m_shadowWindow;

	std::vector<ContactStep>                        m_contactSteps_Step;
	Line                                            m_contactLine;
	Line                                            m_contactIds;
};

static_assert(TickBackendWorldHost<JoltTickWorldHost<tickBackendWorldHost::SimulatableArchetype, std::mutex,
		joltTickHostEngineSelfCheck::Conforming>, tickBackendWorldHost::SimulatableArchetype>,
	"JoltTickWorldHost must model og-simulation's TickBackendWorldHost (the world host members SimulationTickBackend calls), "
	"so a SimulationTickBackend can drive it");
static_assert(TickBackendWorldBuild<JoltTickWorldHost<tickBackendWorldHost::SimulatableArchetype, std::mutex,
		joltTickHostEngineSelfCheck::Conforming>, std::optional<JoltPhysicsBodyAdapter>, std::optional<JoltPhysicsBodyReaderAdapter>,
		std::optional<JoltSpatialQueryAdapter>>,
	"JoltTickWorldHost's build must model og-simulation's TickBackendWorldBuild over the session's three Jolt adapter "
	"optionals, so SimulationTickBackend::build can hand them through");
