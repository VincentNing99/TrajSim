#pragma once
// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Vincent Ning

/// @file iterative_guidance.hpp
/// @brief Iterative Guidance Mode (IGM) closed-loop terminal targeting.

#include "models/guidance/algorithm/guidance_algorithm.hpp"
#include "models/reference_mission.hpp"
#include "core/mat3.hpp"
#include "core/constants.hpp"
#include "core/utils.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <numbers>
#include <optional>

namespace trajsim {

/// @brief Iterative Guidance Mode algorithm for closed-loop terminal targeting.
///
/// Computes steering commands to achieve a target terminal orbital state
/// using iterative time-to-go convergence, delta-V computation, position
/// corrections, and frame transformations.
class IterativeGuidance : public GuidanceAlgorithm {
public:

    /// @brief Expression tree node for composable cutoff conditions.
    ///
    /// Leaf nodes compare a computed orbital parameter against the terminal
    /// reference value. And/Or nodes combine children with boolean logic.
    struct CutoffNode {
        enum class Type : uint8_t { Leaf, And, Or };
        enum class Parameter : uint8_t { SemiMajorAxis, Eccentricity, Inclination };
        enum class Comparison : uint8_t { WithinTolerance, ExceedsBy };

        Type type;
        Parameter parameter{};    ///< Leaf only
        Comparison comparison{};  ///< Leaf only
        double value{};           ///< Tolerance or threshold (leaf only)
        std::vector<CutoffNode> children;  ///< And/Or only

        static CutoffNode leaf(Parameter p, Comparison c, double v) {
            CutoffNode n;
            n.type = Type::Leaf;
            n.parameter = p;
            n.comparison = c;
            n.value = v;
            return n;
        }
        static CutoffNode andOf(std::vector<CutoffNode> ch) {
            CutoffNode n;
            n.type = Type::And;
            n.children = std::move(ch);
            return n;
        }
        static CutoffNode orOf(std::vector<CutoffNode> ch) {
            CutoffNode n;
            n.type = Type::Or;
            n.children = std::move(ch);
            return n;
        }

        [[nodiscard]] bool evaluate(double a, double e, double i,
                                    const TerminalState& ts) const {
            switch (type) {
            case Type::Leaf: {
                double computed = 0.0, reference = 0.0;
                switch (parameter) {
                    case Parameter::SemiMajorAxis: computed = a; reference = ts.semiMajorAxis; break;
                    case Parameter::Eccentricity:  computed = e; reference = ts.eccentricity;  break;
                    case Parameter::Inclination:   computed = i; reference = ts.inclination;    break;
                }
                if (comparison == Comparison::WithinTolerance)
                    return std::fabs(computed - reference) <= value;
                else
                    return (computed - reference) > value;
            }
            case Type::And:
                return std::all_of(children.begin(), children.end(),
                    [&](const CutoffNode& c) { return c.evaluate(a, e, i, ts); });
            case Type::Or:
                return std::any_of(children.begin(), children.end(),
                    [&](const CutoffNode& c) { return c.evaluate(a, e, i, ts); });
            }
            return false;
        }
    };

    struct CutoffCriteria {
        std::optional<CutoffNode> root;  ///< nullopt = never cutoff
    };

    struct Config {
        double guidanceCycle;                   ///< Guidance update cycle [s]
        double steeringHoldTime;                ///< Time before cutoff to hold steering [s]
        int maxConvergenceIterations;           ///< Max iterations for time-to-go convergence
        double timeToGoConvergenceTolerance;    ///< Convergence tolerance for time-to-go [s]

        void validate() const {
            if (guidanceCycle <= 0)
                throw std::invalid_argument("IterativeGuidance::Config: guidanceCycle must be > 0");
            if (steeringHoldTime < 0)
                throw std::invalid_argument("IterativeGuidance::Config: steeringHoldTime must be >= 0");
            if (maxConvergenceIterations < 1)
                throw std::invalid_argument("IterativeGuidance::Config: maxConvergenceIterations must be >= 1");
            if (timeToGoConvergenceTolerance <= 0)
                throw std::invalid_argument("IterativeGuidance::Config: timeToGoConvergenceTolerance must be > 0");
        }
    };

    /// @brief Construct iterative guidance from mission parameters.
    /// @param config Algorithm configuration.
    /// @param cutoffCriteria Cutoff conditions.
    /// @param tolerance Numerical tolerance for guard checks.
    /// @param mission Reference mission containing terminal state.
    /// @param vehicleState Initial vehicle state.
    explicit IterativeGuidance(Config config,
                               CutoffCriteria cutoffCriteria,
                               double tolerance,
                               const ReferenceMission& mission,
                               const VehicleState& vehicleState);

    // GuidanceAlgorithm interface
    [[nodiscard]] GuidanceOutput computeSteering(const GuidanceInput& input) override;
    [[nodiscard]] bool cutoff(const VehicleState& state) const override;
    [[nodiscard]] std::string_view name() const noexcept override { return "IterativeGuidance"; }
    [[nodiscard]] const TerminalState* getTerminalState() const noexcept override { return &terminalState; }

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
    CutoffCriteria cutoffCriteria;
    double tolerance;
    const VehicleState& vehicleState;

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
