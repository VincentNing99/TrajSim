// models/guidance.hpp
#pragma once
// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Vincent Ning

/// @file guidance.hpp
/// @brief Guidance algorithms

#include <optional>
#include <string>
#include <vector>
#include "core/constants.hpp"
#include "core/utils.hpp"
#include "core/mat3.hpp"
#include "core/vec3.hpp"
#include "reference_mission.hpp"
#include "core/types.hpp"

namespace trajsim {

// ============================================================================
// Guidance Mode Enum
// ============================================================================

/// @brief Guidance mode enumeration
enum class GuidanceMode {
    OpenLoop,  ///< Open-loop guidance using pre-programmed steering profile
    IGM        ///< Iterative Guidance Mode (closed-loop terminal targeting)
};

/// @brief Converts GuidanceMode enum to string
[[nodiscard]] inline std::string guidanceModeToString(GuidanceMode mode) {
    switch (mode) {
        case GuidanceMode::OpenLoop: return "openLoop";
        case GuidanceMode::IGM:      return "IGM";
    }
    return "Unknown";
}

// ============================================================================
// Guidance Class
// ============================================================================

/// @brief Guidance system for open-loop and iterative trajectory steering.
/// @details Computes steering commands based on pre-programmed pitch/yaw profiles
///          or iterative guidance targeting terminal orbital conditions.
class Guidance {
public:
    struct Config {
        int stages;                                                 ///< number of stage
        std::vector<GuidanceMode> mode;                             ///< Guidance mode per stage
        double tolerance;                                           ///< Numerical tolerance for modes
        double maxSteeringRate;                                     ///< [deg/s]
        std::vector<double> guidanceCycle;                          ///< [s]
        // cutoff conditions
        std::vector<double> inclinationTolerance;                 ///< [rad]
        std::vector<double> eccentricityTolerance;                ///< dimensionless
        std::vector<double> steeringHoldTime;                     ///< [s] Time before cutoff to hold steering commands
        std::vector<int>    maxConvergenceIterations;             ///< Max iterations for time-to-go convergence
        std::vector<double> timeToGoConvergenceTolerance;         ///< [s] Tolerance for time-to-go convergence

        void validate() const {
            if (tolerance <= 0)
                throw std::invalid_argument("GuidanceConfig: tolerance must be > 0");
            if (maxSteeringRate <= 0)
                throw std::invalid_argument("GuidanceConfig: maxSteeringRate must be > 0");
            if (stages <= 0)
                throw std::invalid_argument("GuidanceConfig: stages must be > 0");

            // Validate vector sizes match number of stages
            if (mode.size() != static_cast<size_t>(stages))
                throw std::invalid_argument("GuidanceConfig: mode size must match stages");
            if (guidanceCycle.size() != static_cast<size_t>(stages))
                throw std::invalid_argument("GuidanceConfig: guidanceCycle size must match stages");
            if (inclinationTolerance.size() != static_cast<size_t>(stages))
                throw std::invalid_argument("GuidanceConfig: inclinationTolerance size must match stages");
            if (eccentricityTolerance.size() != static_cast<size_t>(stages))
                throw std::invalid_argument("GuidanceConfig: eccentricityTolerance size must match stages");
            if (steeringHoldTime.size() != static_cast<size_t>(stages))
                throw std::invalid_argument("GuidanceConfig: steeringHoldTime size must match stages");
            if (maxConvergenceIterations.size() != static_cast<size_t>(stages))
                throw std::invalid_argument("GuidanceConfig: maxConvergenceIterations size must match stages");
            if (timeToGoConvergenceTolerance.size() != static_cast<size_t>(stages))
                throw std::invalid_argument("GuidanceConfig: timeToGoConvergenceTolerance size must match stages");

            // Validate per-stage values
            for (size_t i = 0; i < static_cast<size_t>(stages); ++i) {
                // Mode is now enum, no string validation needed (type-safe)

                if (mode[i] == GuidanceMode::IGM) {
                    if (guidanceCycle[i] <= 0)
                        throw std::invalid_argument("GuidanceConfig: guidanceCycle[" + std::to_string(i) + "] must be > 0 for IGM mode");
                    if (inclinationTolerance[i] < 0)
                        throw std::invalid_argument("GuidanceConfig: inclinationTolerance[" + std::to_string(i) + "] must be >= 0 for IGM mode");
                    if (eccentricityTolerance[i] < 0)
                        throw std::invalid_argument("GuidanceConfig: eccentricityTolerance[" + std::to_string(i) + "] must be >= 0 for IGM mode");
                    if (steeringHoldTime[i] < 0)
                        throw std::invalid_argument("GuidanceConfig: steeringHoldTime[" + std::to_string(i) + "] must be >= 0 for IGM mode");
                    if (maxConvergenceIterations[i] < 1)
                        throw std::invalid_argument("GuidanceConfig: maxConvergenceIterations[" + std::to_string(i) + "] must be >= 1 for IGM mode");
                    if (timeToGoConvergenceTolerance[i] <= 0)
                        throw std::invalid_argument("GuidanceConfig: timeToGoConvergenceTolerance[" + std::to_string(i) + "] must be > 0 for IGM mode");
                }
            }
        }
    };

    // ========================================================================
    // Construction
    // ========================================================================

    /// @brief Constructs a Guidance object from a ReferenceMission.
    /// @param mission Reference mission containing all trajectory parameters.
    /// @param guidanceCfg Guidance configuration (cycle time, convergence limits).
    /// @param stageIndex Zero-based stage index for accessing per-stage config vectors.
    explicit Guidance(const ReferenceMission& mission,
                      Config guidanceCfg,
                      const VehicleState& vehicleState,
                      size_t stageIndex = 0)
        : guidanceCfg(guidanceCfg)
        , currentStage(stageIndex)
        , vehicleState(vehicleState)
        , aimingAzimuth(mission.aimingAzimuth)
        , launchLatitude(mission.latitude)
        , longitudeOfAnLaunchsite(mission.lanLaunchsite)
        , positionLaunchSite(mission.positionLaunchSite())
        , terminalState(terminalStateFromMission(mission))
        , timeInitial(mission.initialTime)
        , steering(mission.initialSteeringAngles)
        , steeringLastStep(mission.initialSteeringAngles)
    {
        timeToGo = mission.cutoffTime - mission.initialTime;
        timeToGoLastStep = timeToGo;
        buildRotationMatrix();
    }

    // ========================================================================
    // Steering Commands
    // ========================================================================

    /// @brief Computes steering based on current stage's guidance mode.
    /// @param t Current time [s].
    /// @param massFlowRate Propellant mass flow rate [kg/s] (only used for IGM).
    /// @param engineExitVelocity Engine exhaust velocity [m/s] (only used for IGM).
    /// @param g0 Gravity at current position (only used for IGM).
    /// @param gCutoff Gravity at cutoff position (only used for IGM).
    /// @return Steering angles (pitch, yaw, roll) in body frame [rad].
    [[nodiscard]] SteeringAngles computeSteering(double t,
                                                  double massFlowRate = 0.0,
                                                  double engineExitVelocity = 0.0,
                                                  Vec3 g0 = Vec3::zero(),
                                                  Vec3 gCutoff = Vec3::zero());

    /// @brief Computes open-loop steering angles from pre-programmed profile.
    /// @param t Current time [s].
    /// @return Euler angles (pitch, yaw, roll) in body frame [rad].
    [[nodiscard]] SteeringAngles openLoopGuidance(double t) const;

    /// @brief Computes iterative guidance steering toward terminal state.
    /// @param t Current time [s].
    /// @param state Current vehicle state.
    /// @param massFlowRate Propellant mass flow rate [kg/s].
    /// @param engineExitVelocity Engine exhaust velocity [m/s].
    /// @param gravityLocal Gravity at current position.
    /// @param gravityCutoff Gravity at cutoff position.
    /// @return Euler angles (pitch, yaw, roll) in body frame [rad].
    [[nodiscard]] SteeringAngles iterativeGuidance(double t, const VehicleState& state,
                                          double massFlowRate, double engineExitVelocity,
                                          Vec3 gravityLocal, Vec3 gravityCutoff);

    // ========================================================================
    // Utility Methods
    // ========================================================================

    /// @brief Checks if cutoff conditions are met.
    /// @return True if engine cutoff should occur.
    [[nodiscard]] bool cutoff(const VehicleState& state) const noexcept;

    /// @brief Updates time-to-go for Low Thrust-to-Weight (< 1.0) case.
    void updateTimeToGoLttw(double delV, double tau, const Vec3& g, const Vec3& vt, const Vec3& vc);

    /// @brief Updates time-to-go for High Thrust-to-Weight (>= 1.0) case.
    void updateTimeToGoHttw(double delV, double tau, double engineExitVelocity, const Vec3& g, const Vec3& vt, const Vec3& vc);

    /// @brief Convergence loop for iterative guidance time-to-go update.
    void convergeTimeToGo(const Vec3& g, const Vec3& vC, double tau, double engineExitVelocity, double thrustToWeight);

    /// @brief Builds rotation matrix from inertial to terminal guidance frame.
    void buildRotationMatrix();

    /// @brief Loads open-loop steering profile from CSV file.
    /// @param filepath Path to CSV file with columns [time, pitch, yaw, roll].
    /// @throws std::runtime_error If file cannot be opened.
    void loadSteeringProfile(const std::string& filepath) {
        steeringProfile = readCsv(filepath);
        assert(!steeringProfile.empty() && "Steering profile is empty");
        assert(steeringProfile[0].size() >= 4 && "Steering profile must have [time, pitch, yaw, roll]");
    }

    /// @brief Computes the required delta-V for current state and time-to-go.
    /// @param g Current gravitational acceleration vector.
    /// @param vTerminalCurr Current terminal velocity target.
    void computeDeltaV(const Vec3& g, const Vec3& vTerminalCurr);

    /// @brief Computes (pitch and yaw) from delta-V components.
    /// @return Steering angles (phi=pitch, psi=yaw) in radians.
    [[nodiscard]] SteeringAngles computeVelocitySteeringAngles();

    /// @brief Applies position-based corrections to steering angles.
    /// @param velocityAngles Steering angles computed from velocity error (before correction).
    /// @param tau Current time constant.
    /// @param engineExitVelocity Engine exhaust velocity [m/s].
    /// @param g Current gravitational acceleration vector.
    /// @return Corrected steering angles (phi=pitch, psi=yaw) in radians.
    [[nodiscard]] SteeringAngles applyPositionCorrections(const SteeringAngles& velocityAngles, double tau, double engineExitVelocity, const Vec3& g);

    /// @brief Translates steering commands back to inertial frame.
    /// @param correctedAngles Steering angles after position correction in terminal frame.
    /// @return Steering angles in inertial frame.
    [[nodiscard]] SteeringAngles transformSteeringToInertial(SteeringAngles correctedAngles);

    // ========================================================================
    // Accessors
    // ========================================================================

    /// @brief Returns the target terminal state.
    [[nodiscard]] const TerminalState& getTerminalState() const noexcept { return terminalState;}

private:
    // ========================================================================
    // Default State Factory
    // ========================================================================

    /// @brief Creates terminal state from a ReferenceMission.
    /// @param mission The reference mission to extract terminal state from.
    /// @return TerminalState initialized with mission values.
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

    // ========================================================================
    // Member Data
    // ========================================================================

    // Configuration
    Config guidanceCfg;
    size_t currentStage;                            ///< Zero-based index for current stage
    const VehicleState& vehicleState;               ///< Reference to vehicle state

    // Launch site
    double aimingAzimuth;                       ///< Azimuth of launch inertial frame [rad].
    double launchLatitude;                      ///< Geodetic latitude of launch site [rad].
    double longitudeOfAnLaunchsite;             ///< Longitude of ascending node from launch site [rad].
    Vec3 positionLaunchSite;                    ///< Launch site position vector.

    // Terminal targets
    TerminalState terminalState;                ///< Target orbital state at cutoff.
    Vec3 terminalVelocity;                      ///< Target velocity in terminal frame.
    Vec3 terminalPosition;                      ///< Target position in terminal frame. 

    // Current state in terminal frame
    Vec3 instantaneousVelocity;                 ///< Current velocity in terminal frame.
    Vec3 instantaneousPosition;                 ///< Current position in terminal frame.

    // Timing
    double timeInitial;                         ///< Initial guidance time [s].
    double timeToGo;                            ///< Estimated time to cutoff [s].
    double timeToGoLastStep;                        ///< Previous time-to-go estimate [s].

    // Steering
    std::vector<std::vector<double>> steeringProfile;  ///< Open-loop steering angle table.
    SteeringAngles steering;                           ///< Current steering command.
    SteeringAngles steeringLastStep;                   ///< Previous steering command.

    // Frame transforms & guidance state
    Mat3 rmLaunchToEquatorial;                  ///< Rotation: launch frame to equatorial frame.
    Mat3 rmInertialToTerminal;                  ///< Rotation: inertial -> terminal frame.
    Mat3 rmInertialToOrbital;                  ///< Rotation: terminal -> Orbital frame.
    DeltaV deltaV;                              ///< Required delta-V for guidance.

};

}  // namespace trajsim
