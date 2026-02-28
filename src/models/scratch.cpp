#include "models/guidance/guidance.hpp"
#include "models/reference_mission.hpp"
#include "models/gravity.hpp"
#include "models/atmosphere.hpp"
#include "models/dynamics.hpp"
#include "models/integrator.hpp"
#include "models/vehicle/vehicle.hpp"
#include "models/vehicle/aerodynamics.hpp"
#include "models/vehicle/engine_models/liquid_engine.hpp"
#include "core/constants.hpp"
#include "core/utils.hpp"
#include <iostream>
#include <numbers>

using namespace trajsim;

int main() {
    const ReferenceMission& mission = MISSION_1;

    // Environment
    Gravity gravity(mission.latitude, mission.heightLaunchSite, mission.aimingAzimuth);
    AtmosphereModel atmosphere;

    // Vehicle — mirrors main.cpp
    LiquidEngine::Config leCfg{
        .id                 = "UpperStage",
        .thrust             = 3000.0,
        .isp                = 315.0,
        .mdot               = 0.973,
        .nozzleExitArea     = 0.1637,
        .nozzleExitPressure = 1.0,
        .mountOffset        = Vec3::zero(),
        .dryMass            = 1.0,
        .gimbalLimit        = 0.0,
        .throttleMin        = 1.0
    };
    auto engine = LiquidEngine::fromConfig(leCfg);

    auto highSpeedTable = readCsv("files/highspeed.csv");
    auto lowSpeedTable  = readCsv("files/lowspeed.csv");

    Aerodynamics::Config aeroCfg{
        .refArea        = 11.341149,
        .refLength      = 70.582,
        .machMin        = 0.1,
        .machMax        = 8.0,
        .machTransition = 0.3
    };
    Aerodynamics aero(aeroCfg, highSpeedTable, lowSpeedTable);

    Vehicle::Config vehicleCfg{
        .numberOfStage = 1,
        .mass          = mission.initialMass,
        .stage         = {},
        .aeroCfg       = aeroCfg
    };
    Vehicle vehicle(vehicleCfg, std::move(aero), std::move(engine));
    Dynamics dynamics(gravity, atmosphere, vehicle);

    // Engine constants
    const double exitVelocity = 315.0 * EarthConstants::GRAVITATIONAL_ACCELERATION;
    const Vec3   gCutoff      = gravity.computeAcceleration(mission.positionTerminal);
    const double mdot         = 0.973;

    EngineOutput engineOut = vehicle.getEngine().computeOutput(1.0, 0.0, 0.0, Vec3::zero());
    const double thrustMag = engineOut.force.mag();

    // Initial state
    VehicleState state{
        .position     = mission.initialPosition,
        .velocity     = mission.initialVelocity,
        .acceleration = Vec3::zero(),
        .vehicleMass  = mission.initialMass
    };

    // Guidance
    IterativeGuidance::Config igmCfg{
        .guidanceCycle                = 0.01,
        .steeringHoldTime             = 2.0,
        .maxConvergenceIterations     = 100,
        .timeToGoConvergenceTolerance = 1e-5,
        .tolerance                    = 1e-10,
        .pitchCorrectionStopTime      = 35,
        .yawCorrectionStopTime        = 15
    };
    Guidance::AlgorithmEntry entry{
        .type           = "IterativeGuidance",
        .exitCriteria   = {},
        .igmConfig      = igmCfg,
        .openLoopConfig = {}
    };
    Guidance::Config guidanceCfg{
        .stage           = 1,
        .maxSteeringRate = 5.0,
        .algorithms      = {entry}
    };
    Guidance guidance(guidanceCfg, mission, state, gCutoff, exitVelocity, -mdot);

    // Run for 5 seconds, print every 1 second
    const double dt   = 0.001;
    const double tEnd = mission.initialTime + 5.0;
    double t          = mission.initialTime;
    double nextPrint  = t;

    std::cout << std::fixed;
    std::cout << "t(s)       phi(deg)   psi(deg)   tgo(s)    mass(kg)\n";
    std::cout << std::string(56, '-') << "\n";

    while (t <= tEnd) {
        Vec3 g0 = gravity.computeAcceleration(state.position);
        SteeringAngles steering = guidance.computeSteering(t, g0);

        if (t >= nextPrint) {
            std::cout << t
                      << "  " << steering.phi * 180.0 / std::numbers::pi
                      << "  " << steering.psi * 180.0 / std::numbers::pi
                      << "  " << guidance.getTimeToGo()
                      << "  " << state.vehicleMass << "\n";
            nextPrint += 1.0;
        }

        state = Integrator::stepRK4(dynamics, state, steering, thrustMag, mdot, t, dt);
        t += dt;
    }

    return 0;
}
