#pragma once
// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Vincent Ning

/// @file open_loop_guidance.hpp
/// @brief Open-loop guidance using pre-programmed steering profile.

#include <string>
#include <vector>
#include "models/guidance/algorithm/guidance_algorithm.hpp"

namespace trajsim {

/// @brief Open-loop guidance algorithm using pre-programmed steering profiles.
///
/// Computes steering angles by interpolating a time-indexed table of
/// pitch/yaw/roll commands loaded from a CSV file.
class OpenLoopGuidance : public GuidanceAlgorithm {
public:
    struct Config {
        double tolerance;   ///< Numerical tolerance for interpolation

        void validate() const {
            if (tolerance <= 0)
                throw std::invalid_argument("OpenLoopGuidance::Config: tolerance must be > 0");
        }
    };

    /// @brief Construct open-loop guidance.
    /// @param config Algorithm configuration.
    /// @param initialTime Time offset for profile interpolation [s].
    explicit OpenLoopGuidance(Config config, double initialTime);

    /// @brief Load steering profile from CSV file.
    /// @param filepath Path to CSV with columns [time, pitch, yaw, roll].
    void loadSteeringProfile(const std::string& filepath);

    // GuidanceAlgorithm interface
    [[nodiscard]] SteeringAngles computeSteering(const GuidanceInput& input) override;
    [[nodiscard]] bool exit(const VehicleState& state) const override;
    [[nodiscard]] std::string_view name() const noexcept override { return "OpenLoopGuidance"; }

private:
    Config config;
    double timeInitial;
    mutable double lastElapsed = 0.0;
    std::vector<std::vector<double>> steeringProfile;
};

}  // namespace trajsim
