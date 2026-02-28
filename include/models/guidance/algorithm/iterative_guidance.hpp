#pragma once
// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Vincent Ning

/// @file iterative_guidance.hpp
/// @brief Iterative Guidance Mode (IGM) closed-loop terminal targeting.

#include "models/guidance/algorithm/guidance_algorithm.hpp"
#include "models/guidance/exit_criteria.hpp"
#include "models/reference_mission.hpp"
#include "core/mat3.hpp"
#include "core/constants.hpp"
#include "core/utils.hpp"
#include <cmath>
#include <numbers>

namespace trajsim {

/// @brief Iterative Guidance Mode algorithm for closed-loop terminal targeting.
///
/// Computes steering commands to achieve a target terminal orbital state
/// using iterative time-to-go convergence, delta-V computation, position
/// corrections, and frame transformations.
class IterativeGuidance : public GuidanceAlgorithm {
public:

    struct Config {
        double guidanceCycle;                   ///< Guidance update cycle [s]
        double steeringHoldTime;                ///< Time before cutoff to hold steering [s]
        int maxConvergenceIterations;           ///< Max iterations for time-to-go convergence
        double timeToGoConvergenceTolerance;    ///< Convergence tolerance for time-to-go [s]
        int K1K2Hold;
        int K3K4Hold;

        void validate() const {
            if (guidanceCycle <= 0)
                throw std::invalid_argument("IterativeGuidance::Config: guidanceCycle must be > 0");
            if (steeringHoldTime < 0)
                throw std::invalid_argument("IterativeGuidance::Config: steeringHoldTime must be >= 0");
            if (maxConvergenceIterations < 1)
                throw std::invalid_argument("IterativeGuidance::Config: maxConvergenceIterations must be >= 1");
            if (timeToGoConvergenceTolerance <= 0)
                throw std::invalid_argument("IterativeGuidance::Config: timeToGoConvergenceTolerance must be > 0");
            if (K1K2Hold < 0)
                throw std::invalid_argument("IterativeGuidance::Config: pitch positional corrections coefficients hold time must be > 0");
            if (K3K4Hold < 0)
                throw std::invalid_argument("IterativeGuidance::Config: yaw positional corrections coefficients hold time must be > 0");
        }
    };

    /// @brief Construct iterative guidance from mission parameters.
    /// @param config Algorithm configuration.
    /// @param exitCriteria Exit conditions.
    /// @param tolerance Numerical tolerance for guard checks.
    /// @param mission Reference mission containing terminal state.
    /// @param vehicleState Initial vehicle state.
    explicit IterativeGuidance(Config config,
                               ExitCriteria exitCriteria,
                               double tolerance,
                               const ReferenceMission& mission,
                               const VehicleState& vehicleState,
                               Vec3 gravityCutoff,
                               double exitVelocity,
                               double massFlowRate);

    // GuidanceAlgorithm interface
    [[nodiscard]] SteeringAngles computeSteering(const GuidanceInput& input) override;
    [[nodiscard]] bool exit(const VehicleState& state) const override;
    [[nodiscard]] std::string_view name() const noexcept override { return "IterativeGuidance"; }
    [[nodiscard]] const TerminalState* getTerminalState() const noexcept override { return &terminalState; }
    [[nodiscard]] double getTimeToGo() const noexcept override { return timeToGo; }
    [[nodiscard]] OrbitalElements computeOrbitalElements(const VehicleState& state) const override;

    // Public methods for test compatibility
    void updateTimeToGoLttw(double delV, double tau, const Vec3& g, const Vec3& vt, const Vec3& vc);
    void updateTimeToGoHttw(double delV, double tau, double engineExitVelocity, const Vec3& g, const Vec3& vt, const Vec3& vc);
    void convergeTimeToGo(const Vec3& g, const Vec3& vC, double tau, double engineExitVelocity, double thrustToWeight);
    void computeDeltaV(const Vec3& g, const Vec3& currentVelocity);
    [[nodiscard]] SteeringAngles computeVelocitySteeringAngles();
    [[nodiscard]] SteeringAngles applyPositionCorrections(const SteeringAngles& velocityAngles, double tau, double engineExitVelocity, const Vec3& g);
    [[nodiscard]] SteeringAngles transformSteeringToInertial(SteeringAngles correctedAngles);
    void buildRotationMatrix();

private:
    [[nodiscard]] static TerminalState terminalStateFromMission(const ReferenceMission& mission) noexcept {
        return TerminalState{
            .trueAnomaly         = mission.trueAnomaly,
            .argumentOfPeriapsis = mission.argumentOfPeriapsis,
            .longitudeOfAn       = mission.longitudeAscendingNode,
            .inclination         = mission.inclination,
            .eccentricity        = mission.eccentricity,
            .semiMajorAxis       = mission.semiMajorAxis,
            .argumentOfLatitude  = mission.argumentOfLatitudeInitial(),
            .position            = mission.positionTerminal,
            .velocity            = mission.velocityTerminal
        };
    }

    Config config;
    ExitCriteria exitCriteria;
    double tolerance;
    const VehicleState& vehicleState;

    // Engine & environment constants (computed once at construction)
    Vec3 gravityCutoff;
    double exitVelocity;
    double massFlowRate;

    // Launch site
    double aimingAzimuth;
    double launchLatitude;
    double longitudeOfAnLaunchsite;
    Vec3 positionLaunchSite;

    // Terminal targets
    TerminalState terminalState;
    Vec3 terminalVelocity;
    Vec3 terminalPosition;

    // Current state in terminal frame
    Vec3 instantaneousVelocity;
    Vec3 instantaneousPosition;

    // Timing
    double timeToGo;
    double timeToGoLastStep;

    // Steering
    SteeringAngles steering;
    SteeringAngles steeringLastStep;

    // Frame transforms & guidance state
    Mat3 rmLaunchToEquatorial;
    Mat3 rmInertialToTerminal;
    Mat3 rmInertialToOrbital;
    DeltaV deltaV;
};

}  // namespace trajsim
