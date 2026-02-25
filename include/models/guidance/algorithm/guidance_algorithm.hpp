#pragma once
// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Vincent Ning

/// @file guidance_algorithm.hpp
/// @brief Abstract base class for guidance algorithms.

#include <string_view>
#include "core/vec3.hpp"
#include "core/types.hpp"

namespace trajsim {

// =============================================================================
// Guidance I/O
// =============================================================================

/// @brief Input data bundle for guidance algorithm computation.
struct GuidanceInput {
    double time;                ///< Current simulation time [s]
    VehicleState state;         ///< Current vehicle state
    double massFlowRate;        ///< Propellant mass flow rate [kg/s]
    double exitVelocity;        ///< Engine exhaust velocity [m/s]
    Vec3 gravity;               ///< Gravity at current position [m/s^2]
    Vec3 gravityCutoff;         ///< Gravity at cutoff position [m/s^2]
};

/// @brief Output from a guidance algorithm computation.
struct GuidanceOutput {
    SteeringAngles steering;    ///< Computed steering angles [rad]
};

// =============================================================================
// GuidanceAlgorithm Base Class
// =============================================================================

/// @brief Abstract interface for guidance algorithms.
///
/// Concrete implementations (OpenLoopGuidance, IterativeGuidance) provide
/// specific steering computation and cutoff logic. Follows the EngineModel
/// pattern: virtual interface, deleted copy, default move, protected ctor.
class GuidanceAlgorithm {
public:
    virtual ~GuidanceAlgorithm() = default;
    GuidanceAlgorithm(const GuidanceAlgorithm&) = delete;
    GuidanceAlgorithm& operator=(const GuidanceAlgorithm&) = delete;
    GuidanceAlgorithm(GuidanceAlgorithm&&) = default;
    GuidanceAlgorithm& operator=(GuidanceAlgorithm&&) = default;

    /// @brief Compute steering angles for current state.
    [[nodiscard]] virtual GuidanceOutput computeSteering(const GuidanceInput& input) = 0;

    /// @brief Check if cutoff conditions are met.
    [[nodiscard]] virtual bool cutoff(const VehicleState& state) const = 0;

    /// @brief Get algorithm name for identification.
    [[nodiscard]] virtual std::string_view name() const noexcept = 0;

    /// @brief Get terminal state if available (IGM provides this).
    [[nodiscard]] virtual const TerminalState* getTerminalState() const noexcept { return nullptr; }

protected:
    GuidanceAlgorithm() = default;
};

}  // namespace trajsim
