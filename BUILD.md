# FlightSim Build Instructions

This project is now **cross-platform** and can be built on macOS, Linux, and Windows!

## Prerequisites

### All Platforms
- C++20 compatible compiler
- CMake 3.15 or higher
- GLFW3 library
- OpenGL

### Platform-Specific Requirements

#### macOS
```bash
# Install dependencies using Homebrew
brew install cmake glfw
```

#### Linux (Ubuntu/Debian)
```bash
# Install dependencies
sudo apt-get update
sudo apt-get install build-essential cmake libglfw3-dev libgl1-mesa-dev zenity
```

**Note**: `zenity` is recommended for native file dialogs on Linux.

#### Linux (Fedora/RHEL)
```bash
sudo dnf install cmake glfw-devel mesa-libGL-devel zenity
```

#### Windows
- Install Visual Studio 2019 or later with C++ support
- Install CMake: https://cmake.org/download/
- Install GLFW: Use vcpkg or download from https://www.glfw.org/

Using vcpkg (recommended):
```powershell
vcpkg install glfw3:x64-windows
```

## Building with CMake (Recommended - All Platforms)

### macOS / Linux
```bash
# Create build directory
mkdir build
cd build

# Configure
cmake ..

# Build
cmake --build .

# Run
./FlightSimGUI
```

### Windows (Visual Studio)
```powershell
# Create build directory
mkdir build
cd build

# Configure (if using vcpkg)
cmake .. -DCMAKE_TOOLCHAIN_FILE=[path to vcpkg]/scripts/buildsystems/vcpkg.cmake

# Build
cmake --build . --config Release

# Run
.\Release\FlightSimGUI.exe
```

## Building with Makefile (macOS only)

For quick builds on macOS, you can still use the traditional Makefile:

```bash
# Build
make -f Makefile.gui

# Run
./FlightSimGUI

# Clean
make -f Makefile.gui clean
```

## File Dialogs

The application uses **portable-file-dialogs** for cross-platform native file dialog support:

- **macOS**: Uses AppleScript (osascript)
- **Windows**: Uses native Windows file dialogs
- **Linux**: Uses zenity, kdialog, or matedialog (install at least one)

If no native dialog is available, you can still manually enter file paths in the GUI.

## Project Structure

```
FlightSim/
├── CMakeLists.txt          # Cross-platform build configuration
├── Makefile.gui            # macOS-specific Makefile (legacy)
├── portable-file-dialogs.h # Cross-platform file dialog library
├── gui_main.cpp            # GUI application entry point
├── sim_runner.cpp/hpp      # Simulation runner
├── GNC.cpp/hpp             # Guidance, Navigation & Control
├── rocket.cpp/hpp          # Rocket dynamics
├── aerodynamics.cpp/hpp    # Aerodynamics model
├── gravity.cpp/hpp         # Gravity model
├── engine_model.cpp/hpp    # Engine thrust model
├── math.cpp/hpp            # Math utilities
├── utilities.cpp/hpp       # General utilities
├── imgui/                  # Dear ImGui library
└── files/                  # CSV data files
```

## Troubleshooting

### Linux: File dialogs not working
Install one of the following dialog tools:
```bash
sudo apt-get install zenity    # GTK-based (recommended)
# OR
sudo apt-get install kdialog   # KDE-based
```

### Windows: GLFW not found
Make sure to specify the vcpkg toolchain file when running CMake:
```powershell
cmake .. -DCMAKE_TOOLCHAIN_FILE=C:/path/to/vcpkg/scripts/buildsystems/vcpkg.cmake
```

### Font not loading
The application will automatically fall back to ImGui's default font if system fonts are not found. This is normal and the application will work fine.

### macOS: OpenGL deprecation warnings
These warnings are expected on macOS (Apple deprecated OpenGL in favor of Metal). The application will still work correctly. The warnings are suppressed with the `-DGL_SILENCE_DEPRECATION` flag.

## Notes

- The CMake build system is recommended for all platforms
- The Makefile is provided for legacy macOS builds
- All platform-specific code has been abstracted to work cross-platform
- The GUI uses Dear ImGui with OpenGL 3.3+ backend
