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
#include <fstream>
#include <cmath>
#include <numbers>

using namespace trajsim;

int main() {
    // =========================================================================
    // 1. Load configuration
    // =========================================================================
    auto result = loadConfig("config/config_mission_2.json");
    const AppConfig& cfg = result.config;
    const ReferenceMission& mission = cfg.mission;

    if (result.hasWarnings()) {
        for (const auto& w : result.warnings)
            std::cerr << "[WARN] " << w << "\n";
    }

    std::cout << "=== TrajSim Mission 2 Simulation ===\n";
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
    const auto& engineCfg = cfg.vehicle.stage.engineCfg[0];
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
    Aerodynamics aero(cfg.vehicle.aeroCfg, highSpeedTable, lowSpeedTable);

    Vehicle vehicle(cfg.vehicle, std::move(aero), std::move(engine));

    // =========================================================================
    // 4. Create dynamics
    // =========================================================================
    Dynamics dynamics(gravity, atmosphere, vehicle, cfg.simulation.atmosphereCeiling);

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
    const double guidanceCycle = cfg.guidance[0].algorithms[0].igmConfig.guidanceCycle;

    // Engine output at full throttle in vacuum
    EngineOutput engineOut = vehicle.getEngine().computeOutput(1.0, 0.0, 0.0, Vec3::zero());
    const double thrustMag = engineOut.force.mag();
    const double mdot = engineOut.mdot;
    const int nEngines = cfg.vehicle.stage.numberOfEngine[0];

    // =========================================================================
    // 7. Create guidance
    // =========================================================================
    Guidance guidance(cfg.guidance[0], mission, state, gCutoff, exitVelocity, mdot);

    std::cout << "Engine: thrust=" << thrustMag << " N, mdot=" << mdot
              << " kg/s, Ve=" << exitVelocity << " m/s\n";
    std::cout << "dt=" << dt << " s, t0=" << mission.initialTime
              << " s, propellant=" << vehicle.getPropellantMass() << " kg\n\n";

    // =========================================================================
    // 8. Simulation loop
    // =========================================================================

    // Open CSV for telemetry export
    std::ofstream csv("output/telemetry.csv");
    csv << std::setprecision(10);
    csv << "time,pos_x,pos_y,pos_z,vel_x,vel_y,vel_z,mass,"
        << "altitude,delta_pos_x,delta_pos_y,delta_pos_z,"
        << "delta_vel_x,delta_vel_y,delta_vel_z,"
        << "phi_deg,psi_deg,time_to_go,"
        << "sma,eccentricity,inclination_deg,"
        << "target_sma,target_ecc,target_inc_deg\n";

    std::cout << std::fixed << std::setprecision(2);
    std::cout << std::setw(8) << "Time"
              << std::setw(9) << "TGO"
              << std::setw(10) << "Phi"
              << std::setw(10) << "Psi"
              << std::setw(12) << "SMA"
              << std::setw(12) << "Ecc"
              << std::setw(9) << "Inc"
              << std::setw(10) << "|dPos|"
              << std::setw(10) << "|dVel|"
              << std::setw(10) << "Mass"
              << std::setw(12) << "RangeAngle"
              << "\n";
    std::cout << std::setw(8) << "(s)"
              << std::setw(9) << "(s)"
              << std::setw(10) << "(deg)"
              << std::setw(10) << "(deg)"
              << std::setw(12) << "(m)"
              << std::setw(12) << "(-)"
              << std::setw(9) << "(deg)"
              << std::setw(10) << "(m)"
              << std::setw(10) << "(m/s)"
              << std::setw(10) << "(kg)"
              << std::setw(12) << "(deg)"
              << "\n";
    std::cout << std::string(112, '-') << "\n";

    double t = mission.initialTime;
    double nextPrint = t;
    const double printInterval = 1.0;
    bool exitMet = false;
    SteeringAngles steering = mission.initialSteeringAngles;

    while (vehicle.getPropellantMass() > 0.0) {
        Vec3 g0 = gravity.computeAcceleration(state.position);
        if (t > mission.initialTime + guidanceCycle)
            steering = guidance.computeSteering(t, g0);

        // Record every ~1 second
        if (t >= nextPrint) {
            double tgo = guidance.getTimeToGo();
            OrbitalElements oe = guidance.computeOrbitalElements(state);
            double alt = gravity.computeAltitude(state.position);

            Vec3 dPos = state.position - mission.positionTerminal;
            Vec3 dVel = state.velocity - mission.velocityTerminal;

            // Console
            std::cout << std::setprecision(1) << std::setw(8) << t
                      << std::setprecision(2) << std::setw(9) << tgo
                      << std::setprecision(2) << std::setw(10) << steering.phi * radToDeg
                      << std::setprecision(2) << std::setw(10) << steering.psi * radToDeg
                      << std::setprecision(0) << std::setw(12) << oe.semiMajorAxis
                      << std::setprecision(6) << std::setw(12) << oe.eccentricity
                      << std::setprecision(3) << std::setw(9) << oe.inclination * radToDeg
                      << std::setprecision(0) << std::setw(10) << dPos.mag()
                      << std::setprecision(1) << std::setw(10) << dVel.mag()
                      << std::setprecision(1) << std::setw(10) << state.vehicleMass
                      << std::setprecision(3) << std::setw(12) << std::remainder(guidance.getRangeAngle() * radToDeg, 360.0)
                      << "\n";

            // CSV
            csv << t << ","
                << state.position.x << "," << state.position.y << "," << state.position.z << ","
                << state.velocity.x << "," << state.velocity.y << "," << state.velocity.z << ","
                << state.vehicleMass << ","
                << alt << ","
                << dPos.x << "," << dPos.y << "," << dPos.z << ","
                << dVel.x << "," << dVel.y << "," << dVel.z << ","
                << steering.phi * radToDeg << "," << steering.psi * radToDeg << ","
                << tgo << ","
                << oe.semiMajorAxis << "," << oe.eccentricity << "," << oe.inclination * radToDeg << ","
                << mission.semiMajorAxis << "," << mission.eccentricity << ","
                << mission.inclination * radToDeg << "\n";

            nextPrint += printInterval;
        }

        // Integrate one RK4 step
        state = Integrator::stepRK4(dynamics, state, steering, thrustMag, mdot, t, dt);
        vehicle.consumePropellant(nEngines * mdot * dt);
        t += dt;

        // Check exit criteria
        if (guidance.exit(state)) {
            exitMet = true;
            break;
        }
    }

    csv.close();

    // =========================================================================
    // 9. Final output
    // =========================================================================
    // Final orbital elements
    OrbitalElements oeFinal = guidance.computeOrbitalElements(state);
    Vec3 dPosFinal = state.position - mission.positionTerminal;
    Vec3 dVelFinal = state.velocity - mission.velocityTerminal;

    std::cout << "\n" << std::string(60, '=') << "\n";
    std::cout << "  SIMULATION COMPLETE - Target Reached: "
              << (exitMet ? "Yes" : "No") << "\n";
    std::cout << std::string(60, '=') << "\n";
    std::cout << "Telemetry: output/telemetry.csv\n";
    std::cout << "Final time: " << std::setprecision(4) << t << " s\n";
    std::cout << "Final mass: " << std::setprecision(2) << state.vehicleMass << " kg\n\n";

    std::cout << std::setw(20) << "Parameter"
              << std::setw(16) << "Actual"
              << std::setw(16) << "Target"
              << std::setw(16) << "Error" << "\n";
    std::cout << std::string(68, '-') << "\n";

    std::cout << std::setprecision(1)
              << std::setw(20) << "SMA (m)"
              << std::setw(16) << oeFinal.semiMajorAxis
              << std::setw(16) << mission.semiMajorAxis
              << std::setw(16) << oeFinal.semiMajorAxis - mission.semiMajorAxis << "\n";

    std::cout << std::setprecision(7)
              << std::setw(20) << "Eccentricity"
              << std::setw(16) << oeFinal.eccentricity
              << std::setw(16) << mission.eccentricity
              << std::setw(16) << oeFinal.eccentricity - mission.eccentricity << "\n";

    std::cout << std::setprecision(4)
              << std::setw(20) << "Inclination (deg)"
              << std::setw(16) << oeFinal.inclination * radToDeg
              << std::setw(16) << mission.inclination * radToDeg
              << std::setw(16) << (oeFinal.inclination - mission.inclination) * radToDeg << "\n";

    std::cout << "\n" << std::setprecision(2)
              << std::setw(20) << "Pos error (m)"
              << std::setw(16) << dPosFinal.mag() << "\n"
              << std::setw(20) << "  dX"
              << std::setw(16) << dPosFinal.x << "\n"
              << std::setw(20) << "  dY"
              << std::setw(16) << dPosFinal.y << "\n"
              << std::setw(20) << "  dZ"
              << std::setw(16) << dPosFinal.z << "\n";

    std::cout << std::setprecision(3)
              << std::setw(20) << "Vel error (m/s)"
              << std::setw(16) << dVelFinal.mag() << "\n"
              << std::setw(20) << "  dVx"
              << std::setw(16) << dVelFinal.x << "\n"
              << std::setw(20) << "  dVy"
              << std::setw(16) << dVelFinal.y << "\n"
              << std::setw(20) << "  dVz"
              << std::setw(16) << dVelFinal.z << "\n";

    return 0;
}
