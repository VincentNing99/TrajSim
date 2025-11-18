# Flight Simulation GUI Setup

## What's Been Done ✓

1. **ImGui Files Copied** - All necessary ImGui files from `~/Downloads/imgui-master/` have been copied to your project:
   - Core ImGui files in `imgui/` directory
   - GLFW + OpenGL3 backend files in `imgui/backends/` directory
   - Demo file included for reference

2. **GUI Main File Created** - `gui_main.cpp` with:
   - Basic window setup using GLFW + OpenGL3
   - Control panel for CSV file inputs
   - Input fields for simulation parameters (time, position, velocity, mass, steering angle)
   - Run simulation button
   - ImGui demo window (toggle on/off to learn ImGui features)

3. **Makefile Created** - `Makefile.gui` for building the GUI application

## What You Need to Do Next

### 1. Install GLFW (Required)
```bash
brew install glfw
```

### 2. Download ImPlot (Optional but Recommended for Plotting)
ImPlot is needed for real-time trajectory plotting. Download it from:
- https://github.com/epezent/implot

Then extract and copy these files to your project:
```bash
# After downloading implot-master.zip
mkdir -p "implot"
cp ~/Downloads/implot-master/implot.cpp implot/
cp ~/Downloads/implot-master/implot.h implot/
cp ~/Downloads/implot-master/implot_items.cpp implot/
cp ~/Downloads/implot-master/implot_internal.h implot/
```

### 3. Build the GUI Application

**Option A: Using Command Line (Makefile)**
```bash
make -f Makefile.gui
./FlightSimGUI
```

**Option B: Using Xcode**
1. Create new target "FlightSimGUI" in Xcode
2. Add these files to the target:
   - `gui_main.cpp`
   - All files in `imgui/` folder
   - All files in `imgui/backends/` folder
   - All your existing simulation .cpp files (except sim.cpp)

3. Configure Build Settings:
   - **Header Search Paths**: Add `imgui`, `imgui/backends`
   - **Library Search Paths**: Add `/opt/homebrew/lib`
   - **Other Linker Flags**: Add `-lglfw`
   - **Framework**: Link OpenGL, Cocoa, IOKit, CoreVideo

4. Build and Run

## Current File Structure

```
FlightSim rk4 MSC cleaned/
├── imgui/                          ← ImGui core files
│   ├── backends/                   ← GLFW + OpenGL3 backends
│   ├── imgui.cpp
│   ├── imgui.h
│   └── ... (other imgui files)
├── gui_main.cpp                    ← NEW: GUI entry point
├── Makefile.gui                    ← NEW: Build script
├── sim.cpp                         ← Your original CLI simulation
└── [all your other .cpp/.hpp files]
```

## Testing the Setup

Once built, you should see:
- A window titled "Flight Simulation Control Panel"
- Input fields for CSV file paths
- Parameter controls
- A "Run Simulation" button
- Optional ImGui demo window (great for learning ImGui features)

## Next Steps After GUI Works

1. **Connect simulation logic** - Wire the "Run Simulation" button to actually run your simulation code
2. **Add threading** - Run simulation in background thread to keep GUI responsive
3. **Add ImPlot** - Create real-time plots for trajectory, angles, velocity, etc.
4. **Add file browser** - Use ImGuiFileDialog for better file selection
5. **Add progress bar** - Show simulation progress in real-time

## Troubleshooting

**If build fails with "GLFW not found":**
- Make sure GLFW is installed: `brew install glfw`
- Check library path in Makefile matches your Homebrew installation

**If build fails with linking errors:**
- Ensure all simulation .cpp files are included in the build
- Check that OpenGL framework is linked

**If window doesn't open:**
- Check console for error messages
- Verify OpenGL 3.3+ support on your system
