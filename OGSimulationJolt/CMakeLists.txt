# SPDX-License-Identifier: MPL-2.0
# docs/OGSimulationJolt-rationale.md

set(OGJOLT_JOLT_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../ThirdParty/JoltPhysics")

file(GLOB_RECURSE OGJOLT_JOLT_SOURCES CONFIGURE_DEPENDS
    "${OGJOLT_JOLT_ROOT}/Jolt/*.cpp"
    "${OGJOLT_JOLT_ROOT}/Jolt/*.h"
    "${OGJOLT_JOLT_ROOT}/Jolt/*.inl"
)

add_library(og_jolt_physics STATIC ${OGJOLT_JOLT_SOURCES})

target_include_directories(og_jolt_physics PUBLIC ${OGJOLT_JOLT_ROOT})

target_compile_definitions(og_jolt_physics PUBLIC JPH_OBJECT_LAYER_BITS=16)

option(OGJOLT_ENABLE_ASSERTS "Define JPH_ENABLE_ASSERTS (changes JPH_VERSION_ID)" OFF)
if(OGJOLT_ENABLE_ASSERTS)
    target_compile_definitions(og_jolt_physics PUBLIC JPH_ENABLE_ASSERTS)
endif()

if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(AMD64|x86_64|amd64|X86_64)$")
    target_compile_definitions(og_jolt_physics PUBLIC JPH_USE_SSE4_1 JPH_USE_SSE4_2)
    if(NOT MSVC)
        target_compile_options(og_jolt_physics PUBLIC -msse4.1 -msse4.2 -mpopcnt)
    endif()
endif()

if(MSVC)
    target_compile_options(og_jolt_physics PRIVATE /GR- /EHs-c- /wd4577)
else()
    target_compile_options(og_jolt_physics PRIVATE -fno-rtti -fno-exceptions)
endif()

file(GLOB_RECURSE OGJOLT_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/*.h"
)

list(REMOVE_ITEM OGJOLT_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/OGSimulationJolt.cpp"
)

add_library(og_simulation_jolt STATIC ${OGJOLT_SOURCES})

target_include_directories(og_simulation_jolt PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/..
)

target_link_libraries(og_simulation_jolt PUBLIC og_simulation og_jolt_physics)
