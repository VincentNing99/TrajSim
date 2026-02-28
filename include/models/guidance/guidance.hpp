#pragma once
// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Vincent Ning

/// @file guidance.hpp
/// @brief Per-stage guidance coordinator owning a sequence of algorithms.

#include <memory>
#include <string>
#include <vector>
#include "models/guidance/algorithm/guidance_algorithm.hpp"
#include "models/guidance/algorithm/open_loop_guidance.hpp"
#include "models/guidance/algorithm/iterative_guidance.hpp"
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
/// to the next algorithm when exit conditions are met.
class Guidance {
public:
    /// @brief Algorithm entry in the config — type string plus algorithm-specific JSON.
    struct AlgorithmEntry {
        std::string type;
        ExitCriteria exitCriteria;  ///< Shared expression-tree exit criteria

        // IterativeGuidance params (only used when type == "IterativeGuidance")
        IterativeGuidance::Config igmConfig;

        // OpenLoopGuidance params (only used when type == "OpenLoopGuidance")
        OpenLoopGuidance::Config openLoopConfig;
    };

    struct Config {
        int stage;                              ///< 1-based stage number
        double maxSteeringRate;                 ///< Max steering rate [deg/s]
        std::vector<AlgorithmEntry> algorithms; ///< Ordered algorithm entries

        void validate() const {
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
                      const VehicleState& vehicleState,
                      Vec3 gravityCutoff = Vec3::zero(),
                      double exitVelocity = 0.0,
                      double massFlowRate = 0.0);

    // =========================================================================
    // Steering Commands
    // =========================================================================

    /// @brief Compute steering, delegating to active algorithm with rate limiting.
    [[nodiscard]] SteeringAngles computeSteering(double t,
                                                  Vec3 g0 = Vec3::zero());

    /// @brief Check exit conditions of active algorithm.
    [[nodiscard]] bool exit(const VehicleState& state) const;

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

    /// @brief Get time-to-go from active algorithm.
    [[nodiscard]] double getTimeToGo() const noexcept;

    /// @brief Compute current orbital elements from vehicle state.
    [[nodiscard]] OrbitalElements computeOrbitalElements(const VehicleState& state) const;

private:
    Config guidanceConfig;
    const VehicleState& vehicleState;
    std::vector<std::unique_ptr<GuidanceAlgorithm>> algorithms;
    size_t activeIndex = 0;

    // Rate limiting state
    SteeringAngles prevSteering;
    bool hasPrevSteering = false;

    // Guidance cycle gating — only recompute every guidanceCycle seconds
    double lastGuidanceTime = -1e30;
};

}  // namespace trajsim
