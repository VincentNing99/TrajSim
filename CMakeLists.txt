cmake_minimum_required(VERSION 3.16)
project(TrajSim VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang|AppleClang")
    add_compile_options(-Wall -Wextra -Wpedantic -Wconversion -Wshadow)
endif()

# Include directories
include_directories(${CMAKE_SOURCE_DIR}/include)


# =============================================================================
# Dependencies
# =============================================================================
include(FetchContent)

FetchContent_Declare(
    nlohmann_json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG v3.11.3
)
FetchContent_MakeAvailable(nlohmann_json)

FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.14.0
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

# =============================================================================
# Library
# =============================================================================
add_library(trajsim_lib
    src/models/aerodynamics.cpp
    src/models/atmosphere.cpp
    src/models/gravity.cpp
    src/models/liquid_engine.cpp
    src/models/guidance.cpp
    src/models/dynamics.cpp
    src/models/integrator.cpp
    src/config/config_loader.cpp
)
target_link_libraries(trajsim_lib PUBLIC nlohmann_json::nlohmann_json)

# =============================================================================
# Tests
# =============================================================================
enable_testing()

add_executable(trajsim_tests
    tests/test_vec3.cpp
    tests/test_mat3.cpp
    tests/test_aerodynamics.cpp
    tests/test_atmosphere.cpp
    tests/test_gravity.cpp
    tests/test_liquid_engine.cpp
    tests/test_guidance.cpp
    tests/test_integrator.cpp
    tests/test_config_loader.cpp
    tests/test_vehicle.cpp
    tests/test_dynamics.cpp
)

target_link_libraries(trajsim_tests
    trajsim_lib
    GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(trajsim_tests)