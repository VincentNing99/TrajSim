# TrajSim

A 3-DOF launch vehicle trajectory simulation written in C++20.

Models the ascent phase from liftoff to orbital insertion, using a closed-loop Iterative Guidance Mode (IGM) algorithm derived from the Apollo/Space Shuttle guidance heritage. The simulation integrates aerodynamics, atmosphere, gravity, and propulsion physics at each timestep and writes telemetry to CSV.

## Features

- **Iterative Guidance Mode (IGM)** — closed-loop terminal targeting with time-to-go convergence and position corrections
- **Strategy-pattern guidance** — per-stage coordinator, auto-advances through open-loop and closed-loop algorithms on cutoff criteria
- **Physics models** — US Standard Atmosphere 1976, J2 gravity perturbations, aerodynamic drag and lift, liquid engine thrust/mass flow
- **Mission configurations** — JSON-driven, supports multiple missions (sun-synchronous orbits)
- **370+ unit tests** via GoogleTest

## Requirements

- CMake ≥ 3.16
- C++20 compiler (GCC 11+, Clang 13+, AppleClang 14+)
- Internet access on first build (FetchContent downloads GoogleTest and nlohmann/json)

## Build & Run

```bash
# Build and run simulation (mission 1 by default)
./run.sh

# Build and run with mission 2
./run.sh --mission 2

# Build and run tests
./run.sh test

# Clean rebuild
./run.sh --clean
```

Or manually:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
./build/trajsim
./build/trajsim_tests
```

Output telemetry is written to `output/telemetry.csv`.

## Project Structure

```
TrajSim/
├── include/
│   ├── core/               # Vec3, Mat3, constants, types, utils
│   ├── config/             # Config loader
│   └── models/
│       ├── aerodynamics.hpp
│       ├── atmosphere.hpp
│       ├── dynamics.hpp
│       ├── gravity.hpp
│       ├── integrator.hpp
│       ├── liquid_engine.hpp
│       ├── reference_mission.hpp
│       └── guidance/
│           ├── guidance.hpp
│           ├── exit_criteria.hpp
│           └── algorithm/
│               ├── guidance_algorithm.hpp
│               ├── open_loop_guidance.hpp
│               └── iterative_guidance.hpp
├── src/                    # Implementations
├── tests/                  # GoogleTest unit tests
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

## License

MIT — see [LICENSE](LICENSE).
