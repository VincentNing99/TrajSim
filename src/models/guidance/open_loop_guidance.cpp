// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Vincent Ning

/// @file open_loop_guidance.cpp
/// @brief Open-loop guidance implementation.

#include "models/guidance/open_loop_guidance.hpp"
#include "core/utils.hpp"
#include <cassert>
#include <cmath>

namespace trajsim {

OpenLoopGuidance::OpenLoopGuidance(Config config, CutoffCriteria cutoffCriteria, double initialTime)
    : config(config)
    , cutoffCriteria(cutoffCriteria)
    , timeInitial(initialTime)
{
}

void OpenLoopGuidance::loadSteeringProfile(const std::string& filepath) {
    steeringProfile = readCsv(filepath);
    assert(!steeringProfile.empty() && "Steering profile is empty");
    assert(steeringProfile[0].size() >= 4 && "Steering profile must have [time, pitch, yaw, roll]");
}

GuidanceOutput OpenLoopGuidance::computeSteering(const GuidanceInput& input) {
    const double elapsed = input.time - timeInitial;

    // Clamp to first point if before profile start
    if (elapsed <= steeringProfile[0][0]) {
        return GuidanceOutput{SteeringAngles{
            steeringProfile[0][1],  // phi (pitch)
            steeringProfile[0][2],  // psi (yaw)
            steeringProfile[0][3]   // gamma (roll)
        }};
    }

    // Clamp to last point if after profile end
    const auto& last = steeringProfile.back();
    if (elapsed >= last[0]) {
        return GuidanceOutput{SteeringAngles{last[1], last[2], last[3]}};
    }

    // Find bracketing indices for interpolation
    std::size_t i = 0;
    while (i < steeringProfile.size() - 1 && steeringProfile[i + 1][0] < elapsed) {
        ++i;
    }

    // Linear interpolation
    const auto& p0 = steeringProfile[i];
    const auto& p1 = steeringProfile[i + 1];

    const double dt = p1[0] - p0[0];

    // Guard against division by zero (duplicate timestamps in profile)
    if (std::fabs(dt) < config.tolerance) {
        return GuidanceOutput{SteeringAngles{p0[1], p0[2], p0[3]}};
    }

    const double alpha = (elapsed - p0[0]) / dt;

    return GuidanceOutput{SteeringAngles{
        p0[1] + alpha * (p1[1] - p0[1]),  // phi (pitch)
        p0[2] + alpha * (p1[2] - p0[2]),  // psi (yaw)
        p0[3] + alpha * (p1[3] - p0[3])   // gamma (roll)
    }};
}

bool OpenLoopGuidance::cutoff(const VehicleState& /*state*/) const {
    // Cutoff logic TBD — criteria stored but evaluation not yet defined
    return false;
}

}  // namespace trajsim
