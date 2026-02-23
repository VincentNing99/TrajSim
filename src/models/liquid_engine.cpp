// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Vincent Ning

/// @file liquid_engine.cpp
/// @brief Liquid rocket engine model implementation.

#include "models/vehicle/engine_models/liquid_engine.hpp"
#include <cassert>

namespace trajsim {

LiquidEngine::LiquidEngine(Config cfg) : cfg{std::move(cfg)} {}

EngineOutput LiquidEngine::computeOutput(
    double throttle,
    double /*burnTime*/,
    double ambientPressure,
    const Vec3& /*gimbalAngles*/)
{
    EngineOutput output;

    // Thrust = momentum thrust + pressure thrust (nozzle exit pressure vs ambient)
    // F = F_nominal + A_e * (P_e - P_amb)
    const double thrust = cfg.thrust
        + cfg.nozzleExitArea * (cfg.nozzleExitPressure - ambientPressure);

    // Apply throttle setting and orient along engine axis (+X in body frame)
    output.force = Vec3{thrust * throttle, 0.0, 0.0};

    output.mdot = cfg.mdot * throttle;

    // TODO: compute moment from gimbal deflection and thrust offset
    output.moment = Vec3::zero();

    return output;
}

std::unique_ptr<LiquidEngine> LiquidEngine::fromConfig(const Config& cfg)
{
    // Validate identification
    assert(!cfg.id.empty() && "Engine ID cannot be empty");

    // Validate propulsion parameters
    assert(cfg.thrust > 0.0 && "Thrust must be positive");
    assert(cfg.isp > 0.0 && "Isp must be positive");

    // Validate physical parameters
    assert(cfg.dryMass > 0.0 && "Dry mass must be positive");
    assert(cfg.nozzleExitArea > 0.0 && "Nozzle exit area must be positive");
    assert(cfg.nozzleExitPressure > 0.0 && "Nozzle exit pressure must be positive");

    // Validate control limits
    assert(cfg.gimbalLimit >= 0.0 && "Gimbal limit cannot be negative");
    assert(cfg.throttleMin >= 0.0 && cfg.throttleMin <= 1.0
        && "Throttle min must be in [0, 1]");

    return std::unique_ptr<LiquidEngine>(new LiquidEngine(cfg));
}

}  // namespace trajsim
