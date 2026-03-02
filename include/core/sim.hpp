#pragma once
// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Vincent Ning

/// @file sim.hpp
/// @brief Simulation configuration parameters for the trajectory integrator.

#include <stdexcept>
#include <string>

namespace trajsim {

/// @brief Simulation configuration controlling integrator and guidance timing.
struct SimConfig {
    double timeStepRK4;         ///< RK4 integration time step [s]
    double tolerance;           ///< Numerical convergence tolerance [-]
    double atmosphereCeiling;   ///< Altitude above which aerodynamics are skipped [m]

    /// @brief Validates all parameters are physically meaningful.
    /// @throws std::invalid_argument if any value is non-positive.
    void validate() const {
        if (timeStepRK4 <= 0)
            throw std::invalid_argument("SimConfig: timeStepRK4 must be > 0, got " + std::to_string(timeStepRK4));
        if (tolerance <= 0)
            throw std::invalid_argument("SimConfig: tolerance must be > 0, got " + std::to_string(tolerance));
        if (atmosphereCeiling <= 0)
            throw std::invalid_argument("SimConfig: atmosphereCeiling must be > 0, got " + std::to_string(atmosphereCeiling));
    }
};

}  // namespace trajsim
