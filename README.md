# TrajSim

A 3-DOF launch vehicle trajectory simulation written in C++20.

Models the ascent phase from liftoff to orbital insertion, using a closed-loop Iterative Guidance Mode (IGM) algorithm derived from Apollo/Space Shuttle guidance heritage. The simulation integrates aerodynamics, atmosphere, gravity, and propulsion physics at each timestep and writes telemetry to CSV.

## Features

- **Iterative Guidance Mode (IGM)** — closed-loop terminal targeting with time-to-go convergence and position corrections
- **Strategy-pattern guidance** — per-stage coordinator, auto-advances through open-loop and closed-loop algorithms on cutoff criteria
- **Physics models** — US Standard Atmosphere 1976, J2 gravity perturbations, aerodynamic drag and lift, liquid engine thrust/mass flow
- **Mission configurations** — JSON-driven, supports multiple missions (sun-synchronous orbits)

## Requirements

- CMake ≥ 3.16
- C++20 compiler (GCC 11+, Clang 13+, AppleClang 14+)
- Internet access on first build (FetchContent downloads nlohmann/json)

## Build & Run

```bash
# Build and run simulation
./run.sh

# Clean rebuild
./run.sh --clean
```

Or manually:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
./build/trajsim
```

> **Note:** The active mission config is hardcoded in `src/main.cpp`. Edit the `loadConfig(...)` path to switch missions.

Output telemetry is written to `output/telemetry.csv`.

## Project Structure

```
TrajSim/
├── include/
│   ├── core/               # Vec3, Mat3, constants, types, utils
│   ├── config/             # Config loader
│   └── models/
│       ├── atmosphere.hpp
│       ├── dynamics.hpp
│       ├── gravity.hpp
│       ├── integrator.hpp
│       ├── reference_mission.hpp
│       ├── vehicle/
│       │   ├── vehicle.hpp
│       │   ├── aerodynamics.hpp
│       │   └── engine_models/
│       │       ├── engine.hpp
│       │       └── liquid_engine.hpp
│       └── guidance/
│           ├── guidance.hpp
│           ├── exit_criteria.hpp
│           └── algorithm/
│               ├── guidance_algorithm.hpp
│               ├── open_loop_guidance.hpp
│               └── iterative_guidance.hpp
├── src/                    # Implementations
├── config/                 # Mission JSON configs
├── ui/                     # Trajectory visualizer (HTML/JS)
├── files/                  # Reference telemetry data
├── CMakeLists.txt
└── run.sh
```

## Configuration

Mission parameters are defined in `config/config_mission_N.json`. Key fields:

```json
{
  "mission": {
    "semiMajorAxis": 6903085.0,
    "inclination": 97.496919,
    "eccentricity": 8.851e-6,
    ...
  },
  "guidance": [
    {
      "stage": 1,
      "algorithms": [
        { "type": "OpenLoopGuidance", ... },
        { "type": "IterativeGuidance", ... }
      ]
    }
  ]
}
```
To run your own guidance simulation, please download one of the example json file and edit it according to your vehicle specifications and mission requirements.

## Notes
Normally, if you are not doing a upper stage correction burn, the delta V gap should be easily closed by accleration from your engine. However, if that is not the case, and you find your time-to-go varible is diverging, try setting timeToGoMethod in the json setting file to 1 which uses a different method for time-to-go.
## License

MIT — see [LICENSE](LICENSE).
