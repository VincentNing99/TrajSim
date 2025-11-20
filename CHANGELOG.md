# Changelog

All notable changes to the FlightSim project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Planned
- 3D trajectory visualization
- Real-time orbital map overlay
- Mission replay functionality
- Monte Carlo dispersion analysis
- Trajectory optimization engine
- Python bindings for automation
- Unit testing framework
- Automated CI/CD pipeline

## [1.0.0] - 2025-01-20

### Added - Cross-Platform Release
- **Cross-platform support**: macOS, Linux, and Windows builds
- **CMake build system**: Modern, cross-platform build configuration
- **Dear ImGui GUI**: Complete rewrite with immediate-mode GUI
  - Simulation control panel with file browsing
  - Real-time telemetry display
  - Navigation errors window
  - Orbital elements display
  - Window manager for show/hide panels
  - Tabbed telemetry interface
  - Real-time attitude plots (pitch, yaw, roll)
- **Portable file dialogs**: Cross-platform native file selection
- **Launch site initialization**: GUI-configurable launch parameters
  - Azimuth angles (A0, B0)
  - Geographic coordinates (longitude, latitude)
  - Site elevation
  - Initial altitude
- **Multi-threaded simulation**: Background physics computation
  - Thread-safe telemetry updates
  - Non-blocking GUI rendering
  - Mutex-protected data sharing
- **Comprehensive telemetry**: 50+ real-time parameters
  - Position and velocity vectors
  - Euler angles with time-history plots
  - Mass and fuel tracking
  - Orbital elements (6 Keplerian elements)
  - Engine performance (thrust, Isp, exhaust velocity)
  - Aerodynamic data (Mach, dynamic pressure)
- **Documentation**: Complete project documentation
  - README.md with setup instructions
  - BUILD.md for detailed build steps
  - CONTRIBUTING.md for contributor guidelines
  - CROSS_PLATFORM_CHANGES.md for migration notes
  - LICENSE (MIT)

### Changed
- **Removed macOS-only dependencies**
  - Replaced Cocoa framework with portable alternatives
  - Removed Objective-C++ compilation requirements
  - Replaced NSOpenPanel with portable-file-dialogs
- **Launch parameters**: Moved from hard-coded constants to GUI inputs
  - Removed dependency on trajectory.hpp constants
  - All initialization now configurable via GUI
- **Font loading**: Cross-platform system font detection
  - macOS: Arial Unicode, Helvetica
  - Windows: Arial, Segoe UI
  - Linux: DejaVu Sans, Liberation Sans
  - Graceful fallback to embedded font

### Fixed
- Build warnings on multiple platforms
- File dialog crashes on non-macOS systems
- Thread safety issues in telemetry updates
- Memory leaks in simulation loop

### Technical Details

#### Physics Models
- **Integration**: 4th-order Runge-Kutta (RK4)
- **Gravity**: Earth gravity with J2 perturbations
- **Atmosphere**: U.S. Standard Atmosphere 1976
- **Aerodynamics**: Mach-dependent drag and lift
- **Propulsion**: Variable thrust and specific impulse

#### Coordinate Frames
- Earth-Centered Inertial (ECI)
- Earth-Centered Earth-Fixed (ECEF)
- Launch site frame
- Orbital frame

#### Numerical Methods
- State vector: [position(3), velocity(3), mass(1)]
- Adaptive time stepping capability
- Keplerian orbital element computation

## [0.9.0] - Pre-release (Internal)

### Added
- Initial RK4 integration engine
- Basic rocket dynamics
- Gravity model implementation
- Atmosphere model
- Aerodynamics calculations
- Engine model
- GNC (Guidance, Navigation, Control) framework
- CSV file input/output
- Command-line interface

### Platforms
- macOS only (Xcode project)

---

## Version Numbering

This project uses [Semantic Versioning](https://semver.org/):
- **MAJOR** version: Incompatible API changes
- **MINOR** version: Backwards-compatible functionality additions
- **PATCH** version: Backwards-compatible bug fixes

## Release Types

- **[Unreleased]**: Changes in development, not yet released
- **[X.Y.Z]**: Released versions with date

## Categories

- **Added**: New features
- **Changed**: Changes in existing functionality
- **Deprecated**: Soon-to-be removed features
- **Removed**: Removed features
- **Fixed**: Bug fixes
- **Security**: Security fixes

---

**Note**: For detailed commit history, see the [GitHub repository](https://github.com/VincentNing99/TrajSim/commits/main).
