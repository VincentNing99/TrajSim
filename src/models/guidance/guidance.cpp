// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Vincent Ning

/// @file guidance.cpp
/// @brief Per-stage guidance coordinator implementation.

#include "models/guidance/guidance.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace trajsim {

Guidance::Guidance(Config config,
                   const ReferenceMission& mission,
                   const VehicleState& vehicleState,
                   Vec3 gravityCutoff,
                   double exitVelocity,
                   double massFlowRate)
    : guidanceConfig(std::move(config))
    , vehicleState(vehicleState)
{
    guidanceConfig.validate();

    for (const auto& entry : guidanceConfig.algorithms) {
        if (entry.type == "IterativeGuidance") {
            entry.igmConfig.validate();
            algorithms.push_back(std::make_unique<IterativeGuidance>(
                entry.igmConfig,
                entry.exitCriteria,
                mission,
                vehicleState,
                gravityCutoff,
                exitVelocity,
                massFlowRate));
        } else if (entry.type == "OpenLoopGuidance") {
            entry.openLoopConfig.validate();
            algorithms.push_back(std::make_unique<OpenLoopGuidance>(
                entry.openLoopConfig,
                mission.initialTime));
        } else {
            throw std::invalid_argument("Guidance: unknown algorithm type '" + entry.type + "'");
        }
    }
}

SteeringAngles Guidance::computeSteering(double t,
                                          Vec3 g0) {
    if (algorithms.empty() || activeIndex >= algorithms.size()) {
        throw std::logic_error("Guidance: no active algorithm");
    }

    // Auto-advance if active algorithm's exit conditions are met and it's not the last
    if (activeIndex < algorithms.size() - 1 && algorithms[activeIndex]->exit(vehicleState)) {
        advanceAlgorithm();
    }

    double guidanceCycle = algorithms[activeIndex]->getGuidanceCycle();

    // Guidance cycle gating: only recompute at guidanceCycle intervals,
    // return cached steering otherwise (old code ran every 10 RK4 steps = 0.01s)
    if (guidanceCycle > 0.0 && hasPrevSteering && (t - lastGuidanceTime) < guidanceCycle) {
        return prevSteering;
    }
    lastGuidanceTime = t;

    GuidanceInput input{};
    input.time = t;
    input.state = vehicleState;
    input.gravity = g0;

    SteeringAngles result = algorithms[activeIndex]->computeSteering(input);

    // Apply rate limiting
    if (hasPrevSteering) {
        double dt = guidanceCycle > 0.0 ? guidanceCycle : (t - lastGuidanceTime);

        if (dt > 0.0) {
            double maxRate = guidanceConfig.maxSteeringRate * degToRad;
            double delPhi = std::clamp((result.phi - prevSteering.phi) / dt, -maxRate, maxRate);
            double delPsi = std::clamp((result.psi - prevSteering.psi) / dt, -maxRate, maxRate);

            result.phi = prevSteering.phi + delPhi * dt;
            result.psi = prevSteering.psi + delPsi * dt;
        }
    }

    prevSteering = result;
    hasPrevSteering = true;

    return result;
}

bool Guidance::exit(const VehicleState& state) const {
    if (algorithms.empty() || activeIndex >= algorithms.size()) {
        return false;
    }
    return algorithms[activeIndex]->exit(state);
}

void Guidance::advanceAlgorithm() {
    if (activeIndex < algorithms.size() - 1) {
        ++activeIndex;
    }
}

GuidanceAlgorithm* Guidance::activeAlgorithm() const noexcept {
    if (activeIndex < algorithms.size()) {
        return algorithms[activeIndex].get();
    }
    return nullptr;
}


const TerminalState* Guidance::getTerminalState() const noexcept {
    if (activeIndex < algorithms.size()) {
        return algorithms[activeIndex]->getTerminalState();
    }
    return nullptr;
}

double Guidance::getTimeToGo() const noexcept {
    if (activeIndex < algorithms.size()) {
        return algorithms[activeIndex]->getTimeToGo();
    }
    return 0.0;
}

double Guidance::getRangeAngle() const noexcept {
    if (activeIndex < algorithms.size()) {
        return algorithms[activeIndex]->getRangeAngle();
    }
    return 0.0;
}

OrbitalElements Guidance::computeOrbitalElements(const VehicleState& state) const {
    if (activeIndex < algorithms.size()) {
        return algorithms[activeIndex]->computeOrbitalElements(state);
    }
    return {0.0, 0.0, 0.0};
}

}  // namespace trajsim
