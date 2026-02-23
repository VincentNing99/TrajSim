#pragma once
// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Vincent Ning

/// @file integrator.hpp
/// @brief RK4 integrator for vehicle state propagation.

#include "core/vec3.hpp"
#include "core/types.hpp"
#include "models/dynamics.hpp"

namespace trajsim {

class Integrator {
public:
    /// @brief Advance vehicle state by one RK4 step.
    /// @param dynamics        EOM evaluator
    /// @param state           Current vehicle state (not modified)
    /// @param steering        Steering angles (held constant over the step)
    /// @param thrustMagnitude Thrust force magnitude [N]
    /// @param massFlowRate    Propellant mass flow rate [kg/s]
    /// @param t               Current simulation time [s]
    /// @param dt              Time step [s]
    /// @return Updated vehicle state at t + dt
    [[nodiscard]] static VehicleState stepRK4(const Dynamics& dynamics,
                                              const VehicleState& state,
                                              const SteeringAngles& steering,
                                              double thrustMagnitude,
                                              double massFlowRate,
                                              double t,
                                              double dt) noexcept;

private:
    /// @brief Apply a state derivative to produce an intermediate state.
    /// @param state  Base state
    /// @param deriv  Derivative to apply
    /// @param dt     Fraction of time step
    /// @return Intermediate state at state + deriv * dt
    [[nodiscard]] static VehicleState applyDerivative(const VehicleState& state,
                                                      const StateDerivative& deriv,
                                                      double dt) noexcept;
};

}  // namespace trajsim
