# FlightSim Quick Start Guide

Get up and running with FlightSim in under 5 minutes!

## 🚀 Super Quick Start

### macOS
```bash
# Install dependencies
brew install cmake glfw

# Clone and build
git clone https://github.com/VincentNing99/TrajSim.git
cd TrajSim
make -f Makefile.gui

# Run
./FlightSimGUI
```

### Linux (Ubuntu/Debian)
```bash
# Install dependencies
sudo apt-get update
sudo apt-get install build-essential cmake libglfw3-dev libgl1-mesa-dev

# Clone and build
git clone https://github.com/VincentNing99/TrajSim.git
cd TrajSim
mkdir build && cd build
cmake ..
cmake --build .

# Run
./FlightSimGUI
```

### Windows
```powershell
# Install vcpkg if not already installed
git clone https://github.com/Microsoft/vcpkg.git
.\vcpkg\bootstrap-vcpkg.bat

# Install GLFW
.\vcpkg\vcpkg install glfw3:x64-windows

# Clone and build
git clone https://github.com/VincentNing99/TrajSim.git
cd TrajSim
mkdir build
cd build
cmake .. -G "Visual Studio 16 2019" -DCMAKE_TOOLCHAIN_FILE=path\to\vcpkg\scripts\buildsystems\vcpkg.cmake
cmake --build . --config Release

# Run
.\Release\FlightSimGUI.exe
```

## 📋 First Run Checklist

When you first open FlightSimGUI:

### 1. Load CSV Files (30 seconds)
The example trejectory is in the files folder and are loaded by default. To simulate your own trejectory you must upload the following:

- ✅ steering angles: `files/euler angles.csv`
- ✅ Veloctiy_inertial: `files/V_inertial.csv`
- ✅ P_inertial: `files/P_inertial.csv`
- ✅ Aerodynamics table(Highspeed): `files/highspeed.csv`
- ✅ Aerodynamics table(Lowspeed): `files/lowspeed.csv`

2. Set Launch Site (30 seconds)
Default values are pre-filled, but you can adjust:
- Azimuth A0: `191.47506°` (Launch azimuth to North)
- Azimuth B0: `40.80768°` (Launch site geocentric latitude)
- Longitude: `100.13805°` (East positive)
- Latitude: `40.80768°` (North positive)
- Site Height: `1000 m` (Above sea level)

### 3. Configure Initial State (1 minute)
The defaults are set for a specific mission scenario:

**Time Parameters:**
- Initial Time: `3140.3405 s`
- End Time: `3344.5194 s`

**Initial Position (Inertial Frame):**
- Px: `-912205.4 m`
- Py: `-13212262.5 m`
- Pz: `149163.6 m`

**Initial Velocity (Inertial Frame):**
- Vx: `-7440.6404 m/s`
- Vy: `953.8138 m/s`
- Vz: `294.5883 m/s`

**Vehicle Parameters:**
- Initial Mass: `6731.1 kg`
- Steering Angle: `0.0°`

### 4. Run Simulation (10 seconds)
Click **"Run Simulation"** and watch the magic happen! ✨

## 🎯 What to Watch

### Real-Time Telemetry Window
Shows live updates every frame:
- **Position & Velocity**: Inertial frame state vectors
- **Attitude**: Pitch, yaw, roll with live plots
- **Mass & Fuel**: Propellant consumption tracking
- **Performance**: Mach number, dynamic pressure
- **Engine**: Thrust, specific impulse, exhaust velocity

### Navigation Errors Window
Displays errors from target orbit:
- **Position Errors**: Δx, Δy, Δz (meters)
- **Velocity Errors**: Δvx, Δvy, Δvz (m/s)
- **Orbital Element Errors**: All 6 Keplerian elements

### Orbital Elements Window
Six classical orbital elements:
1. **Semi-major axis (a)**: Orbit size
2. **Eccentricity (e)**: Orbit shape
3. **Inclination (i)**: Orbital plane tilt
4. **Right Ascension (Ω)**: Node orientation
5. **Argument of Perigee (ω)**: Orbit rotation
6. **True Anomaly (ν)**: Position in orbit

## 💡 Pro Tips

### Tip 1: Window Management
Use the **Window Manager** panel to show/hide windows:
- Toggle telemetry windows on/off
- Customize your workspace
- Focus on relevant data

### Tip 2: CSV Output
Results are automatically saved to:
```
files/rk4_IGM_sim_MSC.csv
```
You can change this in the "Output File" field.

### Tip 3: Attitude Plots
The attitude plots show pitch/yaw/roll over time:
- **Blue line**: Pitch angle (vertical trajectory)
- **Green line**: Yaw angle (out-of-plane deviation)
- **Red line**: Roll angle (vehicle rotation)
- Auto-scaling Y-axis for best view

### Tip 4: Simulation Stages
Select starting stage based on your scenario:
- **Launch Site**: Full launch from ground
- **Second Stage**: Start from second stage ignition
- **Third Stage**: Start from third stage ignition

### Tip 5: Real-Time vs Recorded
This simulator runs **faster than real-time**:
- Typical speed: 10-100x real-time
- 200+ second mission completes in ~2-20 seconds
- GUI updates at 60 FPS regardless of simulation speed

## 📊 Understanding Results

### Successful Orbital Insertion
Look for these indicators:
- ✅ Semi-major axis ≈ Target value
- ✅ Eccentricity ≈ 0 (circular orbit)
- ✅ Small position errors (< 1 km)
- ✅ Small velocity errors (< 10 m/s)

### Common Issues

**Problem**: Simulation completes instantly
- **Cause**: End time ≤ Initial time
- **Fix**: Ensure end time > initial time

**Problem**: Large orbital errors
- **Cause**: Incorrect initial conditions or steering angle
- **Fix**: Verify initial state matches your scenario

**Problem**: NaN values appear
- **Cause**: Invalid CSV data or numerical instability
- **Fix**: Check CSV file formats, ensure positive mass

## 🎓 Learning More

### Next Steps
1. **Modify Parameters**: Try different steering angles
2. **Custom Trajectories**: Load your own CSV files
3. **Analyze Results**: Plot output CSV in Excel/Python
4. **Understand Physics**: Read the technical documentation

### Documentation
- **Full README**: [README.md](README.md) - Complete feature documentation
- **Build Guide**: [BUILD.md](BUILD.md) - Detailed build instructions
- **Contributing**: [CONTRIBUTING.md](CONTRIBUTING.md) - How to contribute
- **Changes**: [CROSS_PLATFORM_CHANGES.md](CROSS_PLATFORM_CHANGES.md) - Migration notes

### Example Missions

#### Low Earth Orbit (LEO)
- Altitude: 200-2000 km
- Inclination: 28.5° (Cape Canaveral) or 51.6° (ISS)
- Velocity: ~7.8 km/s

#### Sun-Synchronous Orbit (SSO)
- Altitude: ~600-800 km
- Inclination: ~97-98° (retrograde)
- Velocity: ~7.5 km/s

#### Geostationary Transfer Orbit (GTO)
- Perigee: 200-300 km
- Apogee: 35,786 km
- Inclination: Variable

## 🐛 Troubleshooting

### Build Issues
```bash
# macOS: GLFW not found
brew install glfw

# Linux: Missing libraries
sudo apt-get install libglfw3-dev libgl1-mesa-dev

# Windows: CMake can't find GLFW
# Use vcpkg or specify GLFW path manually
cmake .. -DGLFW3_DIR="path/to/glfw"
```

### Runtime Issues
```bash
# Fonts not loading (Linux)
sudo apt-get install fonts-dejavu

# File dialogs not working (Linux)
sudo apt-get install zenity
```

## ⌨️ Keyboard Shortcuts

- **Esc**: Close focused window
- **Tab**: Navigate between input fields
- **Enter**: Confirm input field changes
- **Space**: Toggle checkboxes

## 🎨 Customization

### Change Window Positions
- Drag window title bars to reposition
- Windows remember positions in `imgui.ini`
- Delete `imgui.ini` to reset layout

### Adjust Time Step
Edit [sim_runner.cpp](sim_runner.cpp):
```cpp
const double dt = 0.1; // Integration time step (seconds)
```

### Modify Plots
Edit [gui_main.cpp](gui_main.cpp):
```cpp
const int max_history_points = 10000; // Increase for longer history
```

## 📞 Getting Help

- **GitHub Issues**: https://github.com/VincentNing99/TrajSim/issues
- **Discussions**: https://github.com/VincentNing99/TrajSim/discussions
- **Email**: Check GitHub profile for contact

## 🎉 Success!

If you see telemetry updating and orbital elements displayed, congratulations! You've successfully:
- ✅ Built FlightSim from source
- ✅ Configured a mission scenario
- ✅ Run a trajectory simulation
- ✅ Analyzed orbital mechanics results

Welcome to the world of rocket trajectory simulation! 🚀

---

**Ready for more?** Check out the [full README](README.md) for advanced features and detailed documentation.
