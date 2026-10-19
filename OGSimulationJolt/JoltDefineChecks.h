#pragma once
// SPDX-License-Identifier: MPL-2.0
// docs/JoltDefineChecks-rationale.md

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Collision/ObjectLayer.h>

#ifdef JPH_DEBUG_RENDERER
#error "og-simulation-jolt: JPH_DEBUG_RENDERER must not be defined in any configuration (Jolt tests it with #ifdef, so =0 turns it ON). It adds virtuals to JPH::Shape and flips a JPH_VERSION_ID feature bit."
#endif

#ifdef JPH_PROFILE_ENABLED
#error "og-simulation-jolt: JPH_PROFILE_ENABLED must not be defined (Jolt tests it with #ifdef, so =0 turns it ON). It flips a JPH_VERSION_ID feature bit."
#endif

#ifdef JPH_DOUBLE_PRECISION
#error "og-simulation-jolt: JPH_DOUBLE_PRECISION must not be defined. The library is single precision, matching the float og-simulation core."
#endif

#ifdef JPH_SHARED_LIBRARY
#error "og-simulation-jolt: JPH_SHARED_LIBRARY must not be defined. Jolt is compiled into the OGSimulationJolt module or the og_jolt_physics static library."
#endif

#if defined(JPH_USE_DX12) || defined(JPH_USE_VK) || defined(JPH_USE_MTL) || defined(JPH_USE_CPU_COMPUTE)
#error "og-simulation-jolt: the Jolt GPU-compute backends (JPH_USE_DX12/VK/MTL/CPU_COMPUTE) are not vendored and must stay unset."
#endif

static_assert(sizeof(JPH::ObjectLayer) == 2, "og-simulation-jolt: JPH_OBJECT_LAYER_BITS must be 16 in every translation unit.");
