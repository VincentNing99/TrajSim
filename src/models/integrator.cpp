// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Vincent Ning

/// @file integrator.cpp
/// @brief RK4 integrator implementation.

#include "models/integrator.hpp"

namespace trajsim {

VehicleState Integrator::applyDerivative(const VehicleState& state,
                                          const StateDerivative& deriv,
                                          double dt) noexcept {
    return VehicleState{
        state.position + deriv.positionDot * dt,
        state.velocity + deriv.velocityDot * dt,
        Vec3::zero(),
        state.vehicleMass + deriv.massDot * dt
    };
}

VehicleState Integrator::stepRK4(const Dynamics& dynamics,
                                  const VehicleState& state,
                                  const SteeringAngles& steering,
                                  double thrustMagnitude,
                                  double massFlowRate,
                                  double t,
                                  double dt) noexcept {
    double dtHalf = dt * 0.5;

    StateDerivative k1 = dynamics.evaluate(state, steering, thrustMagnitude, massFlowRate, t);
    VehicleState s2 = applyDerivative(state, k1, dtHalf);

    StateDerivative k2 = dynamics.evaluate(s2, steering, thrustMagnitude, massFlowRate, t + dtHalf);
    VehicleState s3 = applyDerivative(state, k2, dtHalf);

    StateDerivative k3 = dynamics.evaluate(s3, steering, thrustMagnitude, massFlowRate, t + dtHalf);
    VehicleState s4 = applyDerivative(state, k3, dt);

    StateDerivative k4 = dynamics.evaluate(s4, steering, thrustMagnitude, massFlowRate, t + dt);

    // y_{n+1} = y_n + (dt/6)(k1 + 2k2 + 2k3 + k4)
    double dt6 = dt / 6.0;
    Vec3 acceleration = (k1.velocityDot + k2.velocityDot * 2.0 + k3.velocityDot * 2.0 + k4.velocityDot) * (1.0 / 6.0);

    return VehicleState{
        state.position + (k1.positionDot + k2.positionDot * 2.0 + k3.positionDot * 2.0 + k4.positionDot) * dt6,
        state.velocity + (k1.velocityDot + k2.velocityDot * 2.0 + k3.velocityDot * 2.0 + k4.velocityDot) * dt6,
        acceleration,
        state.vehicleMass + (k1.massDot + 2.0 * k2.massDot + 2.0 * k3.massDot + k4.massDot) * dt6
    };
}

}  // namespace trajsim
