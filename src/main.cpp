// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Vincent Ning

/// @file main.cpp
/// @brief Entry point for the TrajSim trajectory simulation.
/// @author Vincent Ning
/// @date 2025

#include "config/config_loader.hpp"
#include "models/gravity.hpp"
#include "models/atmosphere.hpp"
#include "models/vehicle/vehicle.hpp"
#include "models/vehicle/aerodynamics.hpp"
#include "models/vehicle/engine_models/liquid_engine.hpp"
#include "models/dynamics.hpp"
#include "models/integrator.hpp"
#include "models/guidance/guidance.hpp"
#include "core/constants.hpp"
#include "core/utils.hpp"

#include <iostream>
#include <iomanip>
#include <cmath>
#include <numbers>

using namespace trajsim;

int main() {
    // =========================================================================
    // 1. Load configuration
    // =========================================================================
    auto result = loadConfig("config/config_mission_1.json");
    const AppConfig& cfg = result.config;
    const ReferenceMission& mission = cfg.mission;

    if (result.hasWarnings()) {
        for (const auto& w : result.warnings)
            std::cerr << "[WARN] " << w << "\n";
    }

    std::cout << "=== TrajSim Mission 1 Simulation ===\n";
    std::cout << "Target SMA: " << mission.semiMajorAxis << " m\n";
    std::cout << "Target ecc: " << mission.eccentricity << "\n";
    std::cout << "Target inc: " << mission.inclination * radToDeg << " deg\n\n";

    // =========================================================================
    // 2. Create environment models
    // =========================================================================
    Gravity gravity(mission.latitude, mission.heightLaunchSite, mission.aimingAzimuth);
    AtmosphereModel atmosphere;

    // =========================================================================
    // 3. Create vehicle (engine + aerodynamics from CSV tables)
    // =========================================================================
    const auto& engineCfg = cfg.vehicle.stages[0].engine;
    LiquidEngine::Config leCfg{
        .id = "UpperStage",
        .thrust = engineCfg.thrust,
        .isp = engineCfg.isp,
        .mdot = engineCfg.mdot,
        .nozzleExitArea = engineCfg.nozzleExitArea,
        .nozzleExitPressure = 1.0,
        .mountOffset = Vec3::zero(),
        .dryMass = 1.0,
        .gimbalLimit = 0.0,
        .throttleMin = 1.0
    };
    auto engine = LiquidEngine::fromConfig(leCfg);

    auto highSpeedTable = readCsv("files/highspeed.csv");
    auto lowSpeedTable = readCsv("files/lowspeed.csv");
    Aerodynamics aero(cfg.aerodynamics, highSpeedTable, lowSpeedTable);

    Vehicle::Config vCfg{.numberOfStage = 1, .mass = cfg.vehicle.mass};
    Vehicle vehicle(vCfg, std::move(aero), std::move(engine));

    // =========================================================================
    // 4. Create dynamics
    // =========================================================================
    Dynamics dynamics(gravity, atmosphere, vehicle);

    // =========================================================================
    // 5. Set initial state
    // =========================================================================
    VehicleState state;
    state.position = mission.initialPosition;
    state.velocity = mission.initialVelocity;
    state.acceleration = Vec3::zero();
    state.vehicleMass = mission.initialMass;

    // =========================================================================
    // 6. Precompute constants
    // =========================================================================
    const double exitVelocity = engineCfg.isp * EarthConstants::GRAVITATIONAL_ACCELERATION;
    const Vec3 gCutoff = gravity.computeAcceleration(mission.positionTerminal);
    const double dt = cfg.simulation.timeStepRK4;

    // Engine output at full throttle in vacuum
    EngineOutput engineOut = vehicle.getEngine().computeOutput(1.0, 0.0, 0.0, Vec3::zero());
    const double thrustMag = engineOut.force.x;
    const double mdot = engineOut.mdot;

    // =========================================================================
    // 7. Create guidance
    // =========================================================================
    Guidance guidance(cfg.guidance[0], mission, state, gCutoff, exitVelocity, -mdot);

    std::cout << "Engine: thrust=" << thrustMag << " N, mdot=" << mdot
              << " kg/s, Ve=" << exitVelocity << " m/s\n";
    std::cout << "dt=" << dt << " s, t0=" << mission.initialTime
              << " s, tCutoff=" << mission.cutoffTime << " s\n\n";

    // =========================================================================
    // 8. Simulation loop
    // =========================================================================
    std::cout << std::fixed << std::setprecision(4);
    std::cout << std::setw(10) << "Time"
              << std::setw(16) << "Pos_X"
              << std::setw(16) << "Pos_Y"
              << std::setw(16) << "Pos_Z"
              << std::setw(14) << "|V| (m/s)"
              << std::setw(12) << "Mass (kg)"
              << std::setw(12) << "Phi (deg)"
              << std::setw(12) << "Psi (deg)"
              << "\n";
    std::cout << std::string(118, '-') << "\n";

    double t = mission.initialTime;
    double nextPrint = t;
    const double printInterval = 1.0;
    bool exitMet = false;

    while (t < mission.cutoffTime) {
        // Print every ~1 second
        if (t >= nextPrint) {
            double vMag = state.velocity.mag();
            SteeringAngles steer = guidance.computeSteering(t, gravity.computeAcceleration(state.position));
            std::cout << std::setw(10) << t
                      << std::setw(16) << state.position.x
                      << std::setw(16) << state.position.y
                      << std::setw(16) << state.position.z
                      << std::setw(14) << vMag
                      << std::setw(12) << state.vehicleMass
                      << std::setw(12) << steer.phi * radToDeg
                      << std::setw(12) << steer.psi * radToDeg
                      << "\n";
            nextPrint += printInterval;
        }

        // Get current gravity for guidance
        Vec3 g0 = gravity.computeAcceleration(state.position);

        // Get steering command
        SteeringAngles steering = guidance.computeSteering(t, g0);

        // Integrate one RK4 step
        state = Integrator::stepRK4(dynamics, state, steering, thrustMag, mdot, t, dt);
        t += dt;

        // Check exit criteria
        if (guidance.exit(state)) {
            exitMet = true;
            break;
        }
    }

    // =========================================================================
    // 9. Final output
    // =========================================================================
    std::cout << "\n=== Simulation Complete ===\n";
    std::cout << "Final time:     " << t << " s\n";
    std::cout << "Final position: (" << state.position.x << ", "
              << state.position.y << ", " << state.position.z << ") m\n";
    std::cout << "Final velocity: (" << state.velocity.x << ", "
              << state.velocity.y << ", " << state.velocity.z << ") m/s\n";
    std::cout << "Final |V|:      " << state.velocity.mag() << " m/s\n";
    std::cout << "Final mass:     " << state.vehicleMass << " kg\n";
    std::cout << "Exit criteria:  " << (exitMet ? "MET" : "NOT MET") << "\n";

    // Print target vs actual terminal state
    std::cout << "\nTarget position: (" << mission.positionTerminal.x << ", "
              << mission.positionTerminal.y << ", " << mission.positionTerminal.z << ") m\n";
    std::cout << "Target velocity: (" << mission.velocityTerminal.x << ", "
              << mission.velocityTerminal.y << ", " << mission.velocityTerminal.z << ") m/s\n";

    return 0;
}
