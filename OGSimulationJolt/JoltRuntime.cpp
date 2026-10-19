// SPDX-License-Identifier: MPL-2.0
// docs/JoltRuntime-rationale.md

#include "OGSimulationJolt/JoltRuntime.h"

#include <atomic>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <mutex>

#include <Jolt/Jolt.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/IssueReporting.h>
#include <Jolt/Core/Memory.h>
#include <Jolt/RegisterTypes.h>

#include "OGSimulation/OGAssert.h"

namespace
{
	std::mutex g_lifecycleMutex;
	uint32_t g_referenceCount = 0;
	std::atomic<JoltRuntime::LogFn> g_logger{ nullptr };
	JPH::TraceFunction g_previousTrace = nullptr;
#ifdef JPH_ENABLE_ASSERTS
	JPH::AssertFailedFunction g_previousAssertFailed = nullptr;
#endif

	void logLine(const char* message)
	{
		if (const JoltRuntime::LogFn logger = g_logger.load())
		{
			logger(message);
		}
	}

	void traceToLogger(const char* format, ...)
	{
		char buffer[1024];
		va_list args;
		va_start(args, format);
		std::vsnprintf(buffer, sizeof(buffer), format, args);
		va_end(args);
		logLine(buffer);
	}

#ifdef JPH_ENABLE_ASSERTS
	bool assertToLogger(const char* expression, const char* message, const char* file, JPH::uint line)
	{
		char buffer[1024];
		std::snprintf(buffer, sizeof(buffer), "[Warning] Jolt assert failed: %s (%s) at %s:%u",
			expression, message != nullptr ? message : "", file, static_cast<unsigned>(line));
		logLine(buffer);
		return true;
	}
#endif
}

JoltRuntime& JoltRuntime::acquireForVersion(uint64_t callerVersionId, LogFn logger)
{
	std::lock_guard<std::mutex> lock(g_lifecycleMutex);

	if (!JPH::VerifyJoltVersionIDInternal(callerVersionId))
	{
		char buffer[256];
		std::snprintf(buffer, sizeof(buffer),
			"[Warning] JoltRuntime: JPH_VERSION_ID mismatch: caller 0x%016llx, library 0x%016llx",
			static_cast<unsigned long long>(callerVersionId), static_cast<unsigned long long>(libraryVersionId()));
		if (logger != nullptr)
		{
			logger(buffer);
		}
		OG_CHECK(false, "JoltRuntime::acquire - the including module sees a different JPH_VERSION_ID than the library; every JPH_* define must be Public");
		std::abort();
	}

	if (g_referenceCount == 0)
	{
		OG_CHECK(JPH::Factory::sInstance == nullptr, "JoltRuntime::acquire - JPH::Factory::sInstance is already set by code outside JoltRuntime");

		g_logger.store(logger);
		g_previousTrace = JPH::Trace;
		JPH::Trace = traceToLogger;
#ifdef JPH_ENABLE_ASSERTS
		g_previousAssertFailed = JPH::AssertFailed;
		JPH::AssertFailed = assertToLogger;
#endif

		JPH::RegisterDefaultAllocator();
		JPH::Factory::sInstance = new JPH::Factory();
		JPH::RegisterTypes();

		char buffer[128];
		std::snprintf(buffer, sizeof(buffer), "JoltRuntime: Jolt %d.%d.%d initialised (JPH_VERSION_ID 0x%016llx)",
			JPH_VERSION_MAJOR, JPH_VERSION_MINOR, JPH_VERSION_PATCH, static_cast<unsigned long long>(libraryVersionId()));
		logLine(buffer);
	}

	++g_referenceCount;

	static JoltRuntime instance;
	return instance;
}

void JoltRuntime::release()
{
	std::lock_guard<std::mutex> lock(g_lifecycleMutex);

	OG_CHECK(g_referenceCount > 0, "JoltRuntime::release - called more often than acquire");
	if (g_referenceCount == 0)
	{
		return;
	}

	--g_referenceCount;
	if (g_referenceCount > 0)
	{
		return;
	}

	JPH::UnregisterTypes();
	delete JPH::Factory::sInstance;
	JPH::Factory::sInstance = nullptr;

	logLine("JoltRuntime: Jolt shut down");

	JPH::Trace = g_previousTrace;
#ifdef JPH_ENABLE_ASSERTS
	JPH::AssertFailed = g_previousAssertFailed;
#endif
	g_logger.store(nullptr);
}

uint32_t JoltRuntime::referenceCount()
{
	std::lock_guard<std::mutex> lock(g_lifecycleMutex);
	return g_referenceCount;
}

uint64_t JoltRuntime::libraryVersionId()
{
	using JPH::uint64;
	return JPH_VERSION_ID;
}
