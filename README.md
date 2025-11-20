# FlightSim - Rocket Trajectory Simulator

A high-fidelity rocket trajectory simulation and guidance system with real-time visualization. This simulator uses 4th-order Runge-Kutta (RK4) integration to model rocket flight dynamics, including gravity, atmospheric effects, aerodynamics, and propulsion.

![Platform](https://img.shields.io/badge/platform-macOS%20%7C%20Linux%20%7C%20Windows-blue)
![C++](https://img.shields.io/badge/C%2B%2B-20-00599C?logo=c%2B%2B)
![License](https://img.shields.io/badge/license-MIT-green)

## Features

### Core Simulation
- **High-Fidelity Physics**: RK4 numerical integration for accurate trajectory propagation
- **Multi-Stage Support**: Configurable staging with different propulsion models
- **Atmospheric Modeling**: Realistic atmospheric density, pressure, and temperature profiles
- **Aerodynamics**: Drag and lift calculations based on Mach number and altitude
- **Gravity Model**: Earth gravity with J2 perturbation effects
- **Guidance System**: Iterative Guidance Mode (IGM) for orbital insertion

### Real-Time Visualization
- **Interactive GUI**: Built with Dear ImGui for responsive, immediate-mode interface
- **Live Telemetry**: Real-time display of position, velocity, attitude, and orbital elements
- **Attitude Plots**: Time-history graphs for pitch, yaw, and roll angles
- **Navigation Errors**: Delta-V and position errors from target orbit
- **Orbital Elements**: Six classical Keplerian elements with target deltas
- **Window Management**: Customizable panel visibility and layout

### Cross-Platform Support
- **macOS**: Native support with OpenGL and GLFW
- **Linux**: Full compatibility with system libraries
- **Windows**: Windows 10/11 support
- **Portable File Dialogs**: Cross-platform native file selection

## Screenshots

### Main Control Panel
The simulation control panel provides complete configuration of launch parameters, initial conditions, and file inputs.

### Real-Time Telemetry
Live updates of all vehicle state data including position, velocity, mass, thrust, and performance metrics.

### Orbital Elements Display
Six classical Keplerian orbital elements with real-time updates and target deltas.

## Table of Contents

- [Requirements](#requirements)
- [Installation](#installation)
- [Building](#building)
- [Usage](#usage)
- [Configuration](#configuration)
- [Project Structure](#project-structure)
- [Technical Details](#technical-details)
- [Contributing](#contributing)
- [License](#license)

## Requirements

### All Platforms
- C++20 compatible compiler
- CMake 3.15+ (recommended) or Make
- GLFW 3.x
- OpenGL 3.3+

### macOS
```bash
brew install cmake glfw
```
- Xcode Command Line Tools: `xcode-select --install`
- Clang 12+ (included with Xcode)

### Linux (Ubuntu/Debian)
```bash
sudo apt-get update
sudo apt-get install build-essential cmake
sudo apt-get install libglfw3-dev libgl1-mesa-dev
```

### Linux (Fedora/RHEL)
```bash
sudo dnf install cmake gcc-c++
sudo dnf install glfw-devel mesa-libGL-devel
```

### Windows
- Visual Studio 2019+ with C++ Desktop Development
- CMake 3.15+
- GLFW (install via vcpkg or manual download)

```powershell
# Using vcpkg
vcpkg install glfw3:x64-windows
```

## Installation

### Clone the Repository
```bash
git clone https://github.com/VincentNing99/TrajSim.git
cd TrajSim
```

### Initialize Submodules
The project includes Dear ImGui as a submodule (if applicable):
```bash
git submodule update --init --recursive
```

## Building

### Option 1: CMake (Recommended - Cross-Platform)

#### macOS/Linux
```bash
mkdir build
cd build
cmake ..
cmake --build .
```

#### Windows (Visual Studio)
```bash
mkdir build
cd build
cmake .. -G "Visual Studio 16 2019" -A x64
cmake --build . --config Release
```

### Option 2: Makefile (macOS/Linux Only)

```bash
make -f Makefile.gui
```

### Building and Running
```bash
# Build and run in one command
make -f Makefile.gui run
```

### Cleaning Build Files
```bash
# CMake build
cd build && make clean

# Makefile build
make -f Makefile.gui clean
```

## Usage

### Starting the Simulator
```bash
# If built with CMake
./build/FlightSimGUI

# If built with Makefile
./FlightSimGUI
```

### Basic Workflow

1. **Configure CSV File Paths**
   - Click "Browse" buttons to select CSV files containing:
     - Euler angles (attitude data)
     - Inertial velocity profile
     - Inertial position profile
     - High-speed aerodynamic coefficients
     - Low-speed aerodynamic coefficients

2. **Set Launch Site Parameters**
   - **Azimuth A0**: Launch azimuth angle to North (degrees)
   - **Azimuth B0**: Launch site geocentric latitude (degrees)
   - **Longitude**: Launch site longitude (degrees)
   - **Latitude**: Launch site geodetic latitude (degrees)
   - **Site Height**: Launch site elevation above sea level (meters)

3. **Initialize Simulation State**
   - Select starting stage (Launch Site, Second Stage, Third Stage)
   - Set initial time and end time
   - Configure initial mass
   - Set steering angle
   - Input initial position vector (x, y, z) in inertial frame
   - Input initial velocity vector (vx, vy, vz) in inertial frame

4. **Run Simulation**
   - Click "Run Simulation" button
   - Monitor real-time telemetry in separate windows
   - Observe attitude plots, orbital elements, and navigation errors
   - Simulation data is saved to the specified output CSV file

5. **Analyze Results**
   - **Real-Time Telemetry**: View current state data
   - **Navigation Errors**: Check delta-V and position errors
   - **Orbital Elements**: Compare achieved orbit with target
   - **Attitude Plots**: Review pitch/yaw/roll history

### Window Management

Use the **Window Manager** panel to show/hide:
- Simulation Control Panel
- Real-Time Telemetry
- Navigation Errors
- Orbital Elements

## Configuration

### CSV File Formats

#### Euler Angles CSV
```
time,phi,psi,gamma
0.0,0.0,0.0,0.0
1.0,5.2,0.1,0.0
...
```
- **time**: Mission elapsed time (seconds)
- **phi**: Pitch angle (degrees)
- **psi**: Yaw angle (degrees)
- **gamma**: Roll angle (degrees)

#### Inertial Velocity CSV
```
time,vx,vy,vz
0.0,100.0,0.0,50.0
...
```

#### Inertial Position CSV
```
time,x,y,z
0.0,6378137.0,0.0,0.0
...
```

#### Aerodynamic Coefficients CSV
```
mach,altitude,Cd,Cl
0.5,1000.0,0.3,0.1
...
```

### Default File Locations
The simulator looks for CSV files in the `files/` directory by default:
- `files/euler angles.csv`
- `files/V_inertial.csv`
- `files/P_inertial.csv`
- `files/highspeed.csv`
- `files/lowspeed.csv`

Output is saved to:
- `files/rk4_IGM_sim_MSC.csv`

### Coordinate Frames

#### Earth-Centered Inertial (ECI) Frame
- **Origin**: Earth's center of mass
- **X-axis**: Vernal equinox direction
- **Z-axis**: Earth's rotation axis (North)
- **Y-axis**: Completes right-handed system

#### Launch Site Frame
- Defined by launch site coordinates and azimuth angles
- Transforms to inertial frame via rotation matrices

## Project Structure

```
FlightSim/
├── README.md                     # This file
├── BUILD.md                      # Detailed build instructions
├── CROSS_PLATFORM_CHANGES.md    # Cross-platform migration notes
├── CMakeLists.txt               # CMake build configuration
├── Makefile.gui                 # Makefile for macOS/Linux
├── FlightSim.xcodeproj/         # Xcode project (macOS)
│
├── gui_main.cpp                 # Main GUI application
├── sim_runner.hpp               # Simulation wrapper class (header)
├── sim_runner.cpp               # Simulation wrapper implementation
│
├── Core Simulation Files
├── rocket.cpp/hpp               # Rocket vehicle dynamics
├── gravity.cpp/hpp              # Gravity model (J2 perturbations)
├── aerodynamics.cpp/hpp         # Aerodynamic forces
├── Atmosphere_properties.cpp/hpp # Atmospheric model
├── engine_model.cpp/hpp         # Propulsion system
├── GNC.cpp/hpp                  # Guidance, Navigation & Control
├── utilities.cpp/hpp            # Utility functions
├── math.cpp/hpp                 # Mathematical operations
│
├── Configuration Files
├── constants.h                  # Physical constants
├── trajectory.hpp               # Target orbital parameters
├── sim.hpp                      # Simulation configuration
│
├── Third-Party Libraries
├── imgui/                       # Dear ImGui library
├── portable-file-dialogs.h     # Cross-platform file dialogs
│
└── files/                       # CSV data files
    ├── euler angles.csv
    ├── V_inertial.csv
    ├── P_inertial.csv
    ├── highspeed.csv
    ├── lowspeed.csv
    └── rk4_IGM_sim_MSC.csv     # Output file
```

## Technical Details

### Numerical Integration
- **Method**: 4th-order Runge-Kutta (RK4)
- **State Vector**: [position (3), velocity (3), mass (1)] = 7 states
- **Time Step**: Adaptive or fixed (configurable)

### Equations of Motion
```
dr/dt = v
dv/dt = (T + D + L) / m + g
dm/dt = -ṁ
```
Where:
- **T**: Thrust vector
- **D**: Aerodynamic drag
- **L**: Aerodynamic lift
- **m**: Vehicle mass
- **g**: Gravitational acceleration (with J2)
- **ṁ**: Mass flow rate

### Orbital Elements Calculation
From Cartesian state vectors (r, v), the simulator computes:
1. **Semi-major axis (a)**: Orbit size
2. **Eccentricity (e)**: Orbit shape
3. **Inclination (i)**: Orbital plane tilt
4. **Right Ascension (Ω)**: Ascending node orientation
5. **Argument of Perigee (ω)**: Orbit orientation in plane
6. **True Anomaly (ν)**: Position along orbit

### Coordinate Transformations
The simulator handles multiple coordinate frame transformations:
- **ECEF ↔ ECI**: Earth-fixed to inertial
- **Launch Frame → ECI**: Site-specific transformations
- **Orbital Frame → ECI**: Keplerian element conversions

### Threading Model
- **Main Thread**: GUI rendering and user input
- **Simulation Thread**: Physics propagation (background)
- **Thread-Safe Telemetry**: Mutex-protected data sharing

## GUI Features

### Tabbed Telemetry Display
- **Summary**: Key performance metrics
- **Position & Velocity**: State vectors and magnitudes
- **Attitude**: Euler angles with time-history plots
- **Mass & Fuel**: Propellant tracking and consumption rates
- **Performance**: Mach number, dynamic pressure, Q-alpha
- **Engine**: Thrust, exhaust velocity, specific impulse

### Real-Time Plots
- **Pitch Angle**: Vertical trajectory profile
- **Yaw Angle**: Out-of-plane deviations
- **Roll Angle**: Vehicle rotation about velocity vector
- **Auto-scaling**: Dynamic Y-axis range adjustment
- **Grid Lines**: Enhanced readability

### Navigation Error Monitoring
- **Position Errors**: Δx, Δy, Δz from SECO target
- **Velocity Errors**: Δvx, Δvy, Δvz from terminal velocity
- **Orbital Element Errors**: Deltas from target orbit
- **Range Angle**: β angle for trajectory optimization

## File Formats

### Output CSV Format
The simulation outputs a comprehensive CSV file with the following columns:
```
time, px, py, pz, vx, vy, vz, mass, thrust, altitude,
velocity_mag, mach, q_pressure, phi, psi, gamma,
semi_major_axis, eccentricity, inclination, raan,
arg_perigee, true_anomaly, apogee, perigee, ...
```

### Input File Validation
- CSV files must have headers matching expected column names
- Numeric values are validated during parsing
- Missing or malformed files generate error messages in GUI

## Performance

### Typical Simulation Speed
- **Real-time factor**: 10-100x (depends on time step and system)
- **Frame rate**: 60 FPS GUI update
- **Telemetry update**: Every integration step

### Memory Usage
- **Baseline**: ~50 MB (GUI + libraries)
- **Telemetry history**: ~10 MB per 10,000 data points
- **CSV data**: Depends on file sizes

## Troubleshooting

### Build Issues

**Problem**: CMake cannot find GLFW
```bash
# macOS
brew install glfw

# Linux
sudo apt-get install libglfw3-dev

# Windows (vcpkg)
vcpkg install glfw3:x64-windows
```

**Problem**: OpenGL headers not found
```bash
# Linux
sudo apt-get install libgl1-mesa-dev
```

**Problem**: C++20 features not supported
- Ensure your compiler supports C++20:
  - GCC 10+
  - Clang 12+
  - MSVC 19.29+ (Visual Studio 2019 16.11+)

### Runtime Issues

**Problem**: Font not loading (squares instead of text)
- The application tries multiple system font paths
- If fonts fail, it falls back to Dear ImGui's default embedded font
- On Linux, install DejaVu fonts: `sudo apt-get install fonts-dejavu`

**Problem**: File dialogs don't appear
- Ensure `portable-file-dialogs.h` is in the project root
- On Linux, install dialog backend: `sudo apt-get install zenity`

**Problem**: Simulation runs but produces NaN values
- Check CSV file formats match expected structure
- Verify initial conditions are physically reasonable
- Ensure mass is positive and non-zero

### Performance Issues

**Problem**: GUI is laggy or slow
- Reduce telemetry history size (modify `max_history_points` in gui_main.cpp)
- Close unused telemetry windows
- Reduce integration time step

## Development

### Adding New Features

1. **New Telemetry Fields**
   - Add fields to `TelemetryData` struct in [sim_runner.hpp](sim_runner.hpp)
   - Update telemetry population in `run_simulation()` in [sim_runner.cpp](sim_runner.cpp)
   - Add GUI display in [gui_main.cpp](gui_main.cpp)

2. **New Physics Models**
   - Create new .cpp/.hpp file pair
   - Add to `SIM_SOURCES` in [CMakeLists.txt](CMakeLists.txt) and [Makefile.gui](Makefile.gui)
   - Integrate into `run_simulation()` physics loop

3. **Custom Guidance Algorithms**
   - Modify [GNC.cpp](GNC.cpp) for guidance logic
   - Update control law calculations
   - Add tuning parameters to GUI

### Coding Standards
- **C++ Version**: C++20
- **Style**: Follow existing code style (4-space indentation)
- **Comments**: Document complex physics calculations
- **Units**: SI units throughout (meters, seconds, kilograms)

### Git Workflow
```bash
# Create feature branch
git checkout -b feature-name

# Make changes and commit
git add .
git commit -m "Description of changes"

# Push to GitHub
git push -u origin feature-name

# Create pull request on GitHub
```

## Contributing

Contributions are welcome! Please follow these steps:

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

### Areas for Contribution
- Additional atmospheric models (custom planets)
- More sophisticated aerodynamic models
- Trajectory optimization algorithms
- 3D visualization of trajectory
- Additional export formats (JSON, HDF5)
- Unit tests and validation cases
- Documentation improvements

## Acknowledgments

- **Dear ImGui**: Omar Cornut (@ocornut) - https://github.com/ocornut/imgui
- **GLFW**: Marcus Geelnard, Camilla Löwy - https://www.glfw.org/
- **Portable File Dialogs**: Sam Hocevar - https://github.com/samhocevar/portable-file-dialogs

## References

### Orbital Mechanics
- Vallado, D. A. (2013). *Fundamentals of Astrodynamics and Applications*
- Curtis, H. D. (2013). *Orbital Mechanics for Engineering Students*

### Guidance Algorithms
- Powered Explicit Guidance (PEG)
- Iterative Guidance Mode (IGM)
- Apollo Guidance Computer documentation

### Atmospheric Models
- U.S. Standard Atmosphere, 1976
- NRLMSISE-00 atmospheric model

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Contact

**Project Maintainer**: Vincent Ning
- GitHub: [@VincentNing99](https://github.com/VincentNing99)
- Repository: https://github.com/VincentNing99/TrajSim

## Roadmap

### Planned Features
- [ ] 3D trajectory visualization using OpenGL
- [ ] Real-time orbital map overlay
- [ ] Mission replay functionality
- [ ] Multi-vehicle formation flying simulation
- [ ] Monte Carlo dispersion analysis
- [ ] Optimization engine for trajectory design
- [ ] Web-based telemetry streaming
- [ ] Docker containerization
- [ ] Python bindings for automation

### Version History
- **v1.0.0** (Current): Initial cross-platform release
  - Dear ImGui GUI
  - RK4 integration
  - Multi-stage support
  - Real-time telemetry
  - Cross-platform builds (macOS, Linux, Windows)

---

**Built with passion for spaceflight and orbital mechanics** 🚀

*For detailed build instructions, see [BUILD.md](BUILD.md)*
