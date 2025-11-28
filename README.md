# FlightSim - Rocket Trajectory Simulator

A high-fidelity rocket trajectory simulation and guidance system with real-time visualization. Uses 4th-order Runge-Kutta (RK4) integration to model multi-stage rocket flight with IGM (Iterative Guidance Mode) for orbital insertion.

![Platform](https://img.shields.io/badge/platform-macOS%20%7C%20Linux%20%7C%20Windows-blue)
![C++](https://img.shields.io/badge/C%2B%2B-20-00599C?logo=c%2B%2B)
![License](https://img.shields.io/badge/license-MIT-green)

## Features

- **High-Fidelity Physics**: RK4 numerical integration with realistic atmospheric, aerodynamic, and gravitational effects (J2 perturbations)
- **Multi-Stage Support**: Configurable first, second, and third stage simulations with stage-specific guidance algorithms (HTTW/LTTW)
- **Advanced Guidance**: Iterative Guidance Mode (IGM) for precision orbital insertion with real-time trajectory correction
- **Real-Time GUI**: Interactive Dear ImGui interface with live telemetry, attitude plots, and orbital element tracking
- **Cross-Platform**: Native support for macOS, Linux, and Windows

## Quick Start

### Requirements

- C++20 compatible compiler (GCC 10+, Clang 12+, or MSVC 2019+)
- CMake 3.15+ or Make
- GLFW 3.x
- OpenGL 3.3+

### Installation

```bash
# macOS
brew install cmake glfw

# Linux (Ubuntu/Debian)
sudo apt-get install build-essential cmake libglfw3-dev libgl1-mesa-dev

# Clone repository
git clone https://github.com/VincentNing99/TrajSim.git
cd TrajSim
```

### Building

```bash
# Using CMake (recommended)
mkdir build && cd build
cmake ..
cmake --build .

# Or using Makefile (macOS/Linux)
make -f Makefile.gui
```

### Running

```bash
# If built with CMake
./build/FlightSimGUI

# If built with Makefile
./FlightSimGUI
```

## Usage

### Simulation Stages

The simulator supports three configurable launch stages:

- **Launch Site (First Stage)**: Ground launch to first stage separation (~150s)
- **Second Stage**: Second stage burn using HTTW (High Thrust-to-Weight) guidance
- **Third Stage**: Third stage burn using LTTW (Low Thrust-to-Weight) guidance

### Basic Workflow

1. **Select Stage**: Choose simulation starting point (Launch Site, Second Stage, or Third Stage)
2. **Configure Parameters**: Set initial conditions, terminal targets, and orbital elements
3. **Load CSV Files**: Provide trajectory data files (euler angles, velocity, position, aerodynamics)
4. **Run Simulation**: Monitor real-time telemetry and attitude plots
5. **Analyze Results**: Review orbital elements, navigation errors, and saved CSV output

### Key Parameters

- **Launch Site**: Longitude, latitude, altitude, azimuth angles (A0, B0)
- **Initial State**: Position (x,y,z), velocity (vx,vy,vz), mass, steering angle
- **Terminal Targets**: Position and velocity at engine cutoff
- **Orbital Elements**: Semi-major axis, eccentricity, inclination, RAAN, argument of perigee, true anomaly

### Coordinate Frames

- **ECSF** (Earth-Centered Space-Fixed): Inertial frame for trajectory propagation
- **Launch Frame**: Local frame defined by launch site parameters
- **Terminal Frame**: Rotating frame aligned with target orbit

## Technical Details

### Numerical Integration

- **Method**: 4th-order Runge-Kutta (RK4)
- **State Vector**: Position (3), velocity (3), mass (1) = 7 states
- **Forces**: Thrust, drag, lift, gravity (with J2 oblateness effects)

### Guidance Algorithms

- **HTTW** (High Thrust-to-Weight): Used for second stage with high acceleration
- **LTTW** (Low Thrust-to-Weight): Used for first and third stages with lower acceleration
- **IGM**: Iterative guidance mode with time-to-go convergence and steering angle calculation

### Key Features

- **Earth Rotation Correction**: Accounts for Earth rotation during flight using ω_e × total_flight_time
- **Dynamic Coordinate Transformations**: Real-time calculation of rotation matrices (G_matrix, C_ea) based on orbital elements
- **Range Angle**: β_e + β_t for second stage, β_e - β_t for first/third stages
- **Engine Cutoff Logic**: Multi-criteria cutoff based on orbital parameters (semi-major axis, eccentricity, inclination)

## Project Structure

```
FlightSim/
├── src/                        # Source files
│   ├── gui_main.cpp           # GUI application entry point
│   ├── sim_runner.cpp/hpp     # Simulation wrapper (threading, telemetry)
│   ├── GNC.cpp/hpp            # Guidance, Navigation, Control
│   ├── rocket.cpp/hpp         # Vehicle dynamics
│   ├── gravity.cpp/hpp        # Gravity model (J2 perturbations)
│   ├── aerodynamics.cpp/hpp   # Aerodynamic forces
│   ├── engine_model.cpp/hpp   # Propulsion system
│   ├── math.cpp/hpp           # RK4 integration
│   ├── utilities.cpp/hpp      # Helper functions
│   ├── constants.h            # Physical constants
│   ├── trajectory.hpp         # Stage-specific parameters
│   └── sim.hpp                # Flight states enum
├── imgui/                     # Dear ImGui library
├── CMakeLists.txt             # CMake build config
├── Makefile.gui               # Makefile for macOS/Linux
└── files/                     # CSV data files
```

## CSV File Formats

### Euler Angles
```csv
time,phi,psi,gamma
0.0,90.0,0.0,0.0
```

### Inertial Velocity
```csv
time,vx,vy,vz
0.0,100.0,0.0,50.0
```

### Inertial Position
```csv
time,x,y,z
0.0,6378137.0,0.0,0.0
```

### Output
Comprehensive CSV with time-series data including position, velocity, mass, attitude, orbital elements, and navigation errors.

## Performance

- **Simulation Speed**: 10-100x real-time (depends on time step)
- **GUI Frame Rate**: 60 FPS
- **Memory Usage**: ~50-100 MB (depends on trajectory history)

## Troubleshooting

### Build Issues
- **GLFW not found**: Install via package manager (`brew install glfw`, `apt-get install libglfw3-dev`)
- **OpenGL headers missing**: Install Mesa dev libraries on Linux
- **C++20 errors**: Ensure compiler version is GCC 10+, Clang 12+, or MSVC 19.29+

### Runtime Issues
- **NaN values in simulation**: Check CSV file formats and initial conditions
- **GUI lag**: Reduce telemetry history size, close unused windows
- **File dialogs don't work**: Install zenity on Linux (`apt-get install zenity`)

## Recent Updates

### Stage Selection Feature
- Added GUI stage selection (LaunchSite, SecondStage, ThirdStage)
- Implemented stage-specific guidance algorithm selection (HTTW/LTTW)
- Dynamic parameter loading based on selected stage
- Fixed hardcoded trajectory parameters to use GUI inputs

### Bug Fixes
- ✅ Fixed hardcoded C_ea launch site rotation matrix
- ✅ Fixed hardcoded launch longitude in G_matrix calculation
- ✅ Fixed Earth rotation correction using stage-specific flight times
- ✅ Fixed time_to_go not updating for third stage
- ✅ Fixed steering angles using wrong flight state
- ✅ Renamed flight states from Chinese pinyin to English
- ✅ Renamed SECO() to Cutoff() for generic engine cutoff

## License

MIT License - see LICENSE file for details.

## Acknowledgments

- **Dear ImGui**: Omar Cornut (@ocornut)
- **GLFW**: Marcus Geelnard, Camilla Löwy
- **Portable File Dialogs**: Sam Hocevar

## Contact

**Vincent Ning**
- GitHub: [@VincentNing99](https://github.com/VincentNing99)
- Repository: [TrajSim](https://github.com/VincentNing99/TrajSim)

---

**Built with precision for spaceflight simulation** 🚀
