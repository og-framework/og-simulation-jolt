# SPDX-License-Identifier: MPL-2.0
# OGSimulationJolt/docs/OGSimulationJolt-rationale.md

cmake_minimum_required(VERSION 3.20)
project(OGSimulationJolt CXX)

if(PROJECT_IS_TOP_LEVEL)
    set(CMAKE_CXX_STANDARD 20)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    if(MSVC)
        add_compile_options(/Zc:preprocessor)
    endif()
    add_compile_definitions(OG_STANDALONE_BUILD)
endif()

if(NOT TARGET og_simulation)
    set(OG_SIMULATION_DIR "" CACHE PATH "Root of an og-simulation checkout (the directory that holds OGSimulation/ and glm/)")
    if(NOT EXISTS "${OG_SIMULATION_DIR}/OGSimulation/CMakeLists.txt")
        message(FATAL_ERROR "og-simulation-jolt: set OG_SIMULATION_DIR to the root of an og-simulation checkout")
    endif()
    if(NOT TARGET glm::glm-header-only)
        add_subdirectory("${OG_SIMULATION_DIR}/glm" "${CMAKE_CURRENT_BINARY_DIR}/glm")
    endif()
    add_subdirectory("${OG_SIMULATION_DIR}/OGSimulation" "${CMAKE_CURRENT_BINARY_DIR}/og_simulation")
endif()

add_subdirectory(OGSimulationJolt)
