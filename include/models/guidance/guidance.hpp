#pragma once
// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Vincent Ning

/// @file guidance.hpp
/// @brief Per-stage guidance coordinator owning a sequence of algorithms.

#include <memory>
#include <string>
#include <vector>
#include "models/guidance/guidance_algorithm.hpp"
#include "models/guidance/open_loop_guidance.hpp"
#include "models/guidance/iterative_guidance.hpp"
#include "models/reference_mission.hpp"
#include "core/utils.hpp"

namespace trajsim {

// =============================================================================
// Guidance Coordinator
// =============================================================================

/// @brief Per-stage guidance coordinator.
///
/// Owns an ordered sequence of GuidanceAlgorithm objects. Delegates steering
/// computation to the active algorithm, applies rate limiting, and auto-advances
/// to the next algorithm when cutoff is reached.
class Guidance {
public:
    /// @brief Algorithm entry in the config — type string plus algorithm-specific JSON.
    struct AlgorithmEntry {
        std::string type;

        // IterativeGuidance params (only used when type == "IterativeGuidance")
        IterativeGuidance::Config igmConfig;
        IterativeGuidance::CutoffCriteria igmCutoffCriteria;

        // OpenLoopGuidance params (only used when type == "OpenLoopGuidance")
        OpenLoopGuidance::Config openLoopConfig;
        OpenLoopGuidance::CutoffCriteria openLoopCutoffCriteria;
    };

    struct Config {
        int stage;                              ///< 1-based stage number
        double tolerance;                       ///< Shared numerical tolerance
        double maxSteeringRate;                 ///< Max steering rate [deg/s]
        std::vector<AlgorithmEntry> algorithms; ///< Ordered algorithm entries

        void validate() const {
            if (tolerance <= 0)
                throw std::invalid_argument("Guidance::Config: tolerance must be > 0");
            if (maxSteeringRate <= 0)
                throw std::invalid_argument("Guidance::Config: maxSteeringRate must be > 0");
            if (algorithms.empty())
                throw std::invalid_argument("Guidance::Config: algorithms must not be empty");
        }
    };

    // =========================================================================
    // Construction
    // =========================================================================

    /// @brief Construct guidance coordinator from config and mission.
    explicit Guidance(Config config,
                      const ReferenceMission& mission,
                      const VehicleState& vehicleState);

    // =========================================================================
    // Steering Commands
    // =========================================================================

    /// @brief Compute steering, delegating to active algorithm with rate limiting.
    [[nodiscard]] SteeringAngles computeSteering(double t,
                                                  double massFlowRate = 0.0,
                                                  double engineExitVelocity = 0.0,
                                                  Vec3 g0 = Vec3::zero(),
                                                  Vec3 gCutoff = Vec3::zero());

    /// @brief Check cutoff of active algorithm.
    [[nodiscard]] bool cutoff(const VehicleState& state) const;

    // =========================================================================
    // Algorithm Sequencing
    // =========================================================================

    /// @brief Advance to next algorithm in sequence.
    void advanceAlgorithm();

    /// @brief Get the active algorithm.
    [[nodiscard]] GuidanceAlgorithm* activeAlgorithm() const noexcept;

    // =========================================================================
    // Accessors
    // =========================================================================

    /// @brief Get terminal state from active algorithm (if available).
    [[nodiscard]] const TerminalState* getTerminalState() const noexcept;

private:
    Config guidanceConfig;
    const VehicleState& vehicleState;
    std::vector<std::unique_ptr<GuidanceAlgorithm>> algorithms;
    size_t activeIndex = 0;

    // Rate limiting state
    SteeringAngles prevSteering;
    bool hasPrevSteering = false;
};

}  // namespace trajsim
