cmake_minimum_required(VERSION 3.15)
project(FlightSim VERSION 1.0 LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Performance optimization flags
# Set default build type to Release if not specified
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
    message(STATUS "Build type not specified, defaulting to Release")
endif()

if(CMAKE_BUILD_TYPE STREQUAL "Release")
    message(STATUS "Building with optimizations enabled")
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        # GCC/Clang optimization flags:
        # -O3: Maximum optimization level
        # -march=native: Optimize for current CPU (enables AVX, SSE, etc.)
        # -ffast-math: Aggressive floating-point optimizations
        add_compile_options(-O3 -march=native -ffast-math)
        message(STATUS "Added optimization flags: -O3 -march=native -ffast-math")
    elseif(MSVC)
        # MSVC optimization flags:
        # /O2: Maximum optimization (speed)
        # /fp:fast: Fast floating-point model
        # /arch:AVX2: Use AVX2 instructions
        add_compile_options(/O2 /fp:fast /arch:AVX2)
        message(STATUS "Added optimization flags: /O2 /fp:fast /arch:AVX2")
    endif()
elseif(CMAKE_BUILD_TYPE STREQUAL "Debug")
    message(STATUS "Building in Debug mode (optimizations disabled)")
endif()

# Find required packages
find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)

# ImGui source files
set(IMGUI_DIR ${CMAKE_CURRENT_SOURCE_DIR}/imgui)
set(IMGUI_SOURCES
    ${IMGUI_DIR}/imgui.cpp
    ${IMGUI_DIR}/imgui_draw.cpp
    ${IMGUI_DIR}/imgui_tables.cpp
    ${IMGUI_DIR}/imgui_widgets.cpp
    ${IMGUI_DIR}/imgui_demo.cpp
    ${IMGUI_DIR}/backends/imgui_impl_glfw.cpp
    ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp
)

# Simulation source files
set(SIM_SOURCES
    src/rocket.cpp
    src/gravity.cpp
    src/aerodynamics.cpp
    src/Atmosphere_properties.cpp
    src/engine_model.cpp
    src/utilities.cpp
    src/math.cpp
    src/GNC.cpp
    src/sim_runner.cpp
)

# Create executable
add_executable(FlightSimGUI
    src/gui_main.cpp
    ${IMGUI_SOURCES}
    ${SIM_SOURCES}
)

# Include directories
target_include_directories(FlightSimGUI PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${IMGUI_DIR}
    ${IMGUI_DIR}/backends
)

# Link libraries
target_link_libraries(FlightSimGUI PRIVATE
    OpenGL::GL
    glfw
)

# Platform-specific settings
if(APPLE)
    target_compile_definitions(FlightSimGUI PRIVATE GL_SILENCE_DEPRECATION)
    # No need for Cocoa framework anymore - we'll use cross-platform file dialogs
endif()

if(UNIX AND NOT APPLE)
    # Linux-specific settings
    find_package(Threads REQUIRED)
    target_link_libraries(FlightSimGUI PRIVATE Threads::Threads ${CMAKE_DL_LIBS})
endif()

if(WIN32)
    # Windows-specific settings
    # GLFW on Windows might need additional libraries
    target_link_libraries(FlightSimGUI PRIVATE gdi32)
endif()

# Set output directory
set_target_properties(FlightSimGUI PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
)

# Installation rules (optional)
install(TARGETS FlightSimGUI
    RUNTIME DESTINATION bin
)

# Print build information
message(STATUS "Building FlightSim GUI")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "System: ${CMAKE_SYSTEM_NAME}")
