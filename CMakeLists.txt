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


# =============================================================================
# Library
# =============================================================================
add_library(trajsim_lib
    src/models/aerodynamics.cpp
    src/models/atmosphere.cpp
    src/models/gravity.cpp
    src/models/liquid_engine.cpp
    src/models/guidance/algorithm/open_loop_guidance.cpp
    src/models/guidance/algorithm/iterative_guidance.cpp
    src/models/guidance/guidance.cpp
    src/models/dynamics.cpp
    src/models/integrator.cpp
    src/config/config_loader.cpp
)
target_link_libraries(trajsim_lib PUBLIC nlohmann_json::nlohmann_json)

# =============================================================================
# Executable
# =============================================================================
add_executable(trajsim src/main.cpp)
target_link_libraries(trajsim trajsim_lib)

