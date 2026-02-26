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
    Vec3 gravity;               ///< Gravity at current position [m/s^2]
};

// =============================================================================
// GuidanceAlgorithm Base Class
// =============================================================================

/// @brief Abstract interface for guidance algorithms.
///
/// Concrete implementations (OpenLoopGuidance, IterativeGuidance) provide
/// specific steering computation and exit logic. Follows the EngineModel
/// pattern: virtual interface, deleted copy, default move, protected ctor.
class GuidanceAlgorithm {
public:
    virtual ~GuidanceAlgorithm() = default;
    GuidanceAlgorithm(const GuidanceAlgorithm&) = delete;
    GuidanceAlgorithm& operator=(const GuidanceAlgorithm&) = delete;
    GuidanceAlgorithm(GuidanceAlgorithm&&) = default;
    GuidanceAlgorithm& operator=(GuidanceAlgorithm&&) = default;

    /// @brief Compute steering angles for current state.
    [[nodiscard]] virtual SteeringAngles computeSteering(const GuidanceInput& input) = 0;

    /// @brief Check if exit conditions are met.
    [[nodiscard]] virtual bool exit(const VehicleState& state) const = 0;

    /// @brief Get algorithm name for identification.
    [[nodiscard]] virtual std::string_view name() const noexcept = 0;

    /// @brief Get terminal state if available (IGM provides this).
    [[nodiscard]] virtual const TerminalState* getTerminalState() const noexcept { return nullptr; }

protected:
    GuidanceAlgorithm() = default;
};

}  // namespace trajsim
