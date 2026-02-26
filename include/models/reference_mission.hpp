// models/reference_mission.hpp
#pragma once
// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Vincent Ning

/// @file reference_mission.hpp
/// @brief Reference mission struct for target orbit and launch site parameters.

#include <cmath>
#include "core/vec3.hpp"
#include "core/mat3.hpp"
#include "core/utils.hpp"
#include "core/constants.hpp"
#include "core/types.hpp"

namespace trajsim {

/// @brief Contains all reference parameters for a mission trajectory.
struct ReferenceMission {
    // ============================================================================
    // Target Orbital Elements
    // ============================================================================
    double semiMajorAxis;           ///< Semi-major axis at terminal point [m]
    double longitudeAscendingNode;  ///< Longitude of ascending node [rad]
    double inclination;             ///< Orbital inclination [rad]
    double eccentricity;            ///< Orbital eccentricity [-]
    double trueAnomaly;             ///< True anomaly at injection [rad]
    double argumentOfPeriapsis;     ///< Argument of perigee [rad]
    double ldnLaunchsite;           ///< Longitude to descending node from launch site [rad]
    double lanLaunchsite;           ///< Longitude to ascending node from launch site [rad]

    // ============================================================================
    // Launch Site Parameters
    // ============================================================================
    double aimingAzimuth;           ///< Angle from north to launch inertial frame x-axis [rad]
    double latitude;                ///< Geodetic latitude of launch site [rad]
    double geocentricLatitude;      ///< Geocentric latitude of launch site [rad]
    double launchSiteLongitude;     ///< Longitude of launch site [rad]
    double heightLaunchSite;        ///< Altitude above reference ellipsoid [m]

    // ============================================================================
    // Terminal State (Inertial Frame)
    // ============================================================================
    Vec3 velocityTerminal;          ///< Target velocity at injection point [m/s]
    Vec3 positionTerminal;          ///< Target position at injection point [m]

    // ============================================================================
    // Initial Vehicle State
    // ============================================================================
    Vec3 initialPosition;           ///< Initial position in local frame [m]
    Vec3 initialVelocity;           ///< Initial velocity in local frame [m/s]
    double initialMass;             ///< Initial vehicle mass [kg]

    // ============================================================================
    // Steering and Timing
    // ============================================================================
    SteeringAngles initialSteeringAngles;  ///< Initial steering angles [rad]
    double initialTime;             ///< Initial time [s]
    double cutoffTime;              ///< Cutoff time [s]

    // ============================================================================
    // Computed Properties
    // ============================================================================

    /// @brief Argument of latitude (argument of perigee + true anomaly) [rad]
    [[nodiscard]] double argumentOfLatitudeInitial() const {
        return argumentOfPeriapsis + trueAnomaly;
    }

    /// @brief Radius of curvature at launch site [m]
    [[nodiscard]] double radiusOfCurvature() const {
        return EarthConstants::RADIUS_OF_EQUATOR * EarthConstants::SEMI_MINOR_AXIS /
               std::sqrt(std::pow(EarthConstants::RADIUS_OF_EQUATOR, 2) * std::pow(std::sin(geocentricLatitude), 2) +
                         std::pow(EarthConstants::SEMI_MINOR_AXIS, 2) * std::pow(std::cos(geocentricLatitude), 2)) +
               heightLaunchSite;
    }

    /// @brief Position vector at launch site [m]
    [[nodiscard]] Vec3 positionLaunchSite() const {
        double r = radiusOfCurvature();
        double latDiff = latitude - geocentricLatitude;
        return {
            -r * std::sin(latDiff) * std::cos(aimingAzimuth),
            r * std::cos(latDiff),
            r * std::sin(aimingAzimuth) * std::sin(latDiff)
        };
    }

    /// @brief aiming azimuth rotation
    [[nodiscard]] Mat3 aimingAzimuthRotation() const {
        return {
            std::cos(aimingAzimuth), 0.0, std::sin(aimingAzimuth),
            0.0, 1.0, 0.0,
            -std::sin(aimingAzimuth), 0.0, std::cos(aimingAzimuth)
        };
    }

    /// @brief latitude rotation
    [[nodiscard]] Mat3 latitudeRotation() const {
        return {
            1.0, 0.0, 0.0,
            0.0, std::cos(latitude), -std::sin(latitude),
            0.0, std::sin(latitude), std::cos(latitude)
        };
    }

    /// @brief Rotation matrix: launch frame to equatorial frame
    [[nodiscard]] Mat3 rmLaunchToEquatorial() const {
        return {
            -std::cos(aimingAzimuth) * sin(latitude), std::cos(latitude), std::sin(aimingAzimuth) * std::sin(latitude),
            std::sin(aimingAzimuth), 0.0, std::cos(aimingAzimuth),
            std::cos(aimingAzimuth) * std::cos(latitude), std::sin(latitude), -std::sin(aimingAzimuth) * std::cos(latitude)
        };
    }

    /// @brief Rotation matrix: LAN rotation about z-axis
    [[nodiscard]] Mat3 rmLanRotation() const {
        return {
            std::cos(lanLaunchsite), std::sin(lanLaunchsite), 0.0,
            -std::sin(lanLaunchsite), std::cos(lanLaunchsite), 0.0,
            0.0, 0.0, 1.0
        };
    }

    /// @brief Rotation matrix: orbital inclination rotation about x-axis
    [[nodiscard]] Mat3 rmInclinationRotation() const {
        return {
            std::cos(inclination), 0.0, -std::sin(inclination),
            0.0, 1.0, 0.0,
            std::sin(inclination), 0.0, std::cos(inclination)
        };
    }

    /// @brief Combined rotation matrix: inertial to orbital plane
    [[nodiscard]] Mat3 rmInertialToOrbital() const {
        return rmInclinationRotation() * (rmLanRotation() * rmLaunchToEquatorial());
    }

    /// @brief Rotation matrix: range angle rotation about z-axis
    [[nodiscard]] Mat3 rmArgumentOfLatitude() const {
        double argLat = argumentOfLatitudeInitial();
        return {
            std::cos(argLat), std::sin(argLat), 0,
            -std::sin(argLat), std::cos(argLat), 0,
            0, 0, 1
        };
    }

    /// @brief Final rotation matrix: inertial frame to terminal guidance frame
    [[nodiscard]] Mat3 rmInertialToTerminal() const {
        return rmArgumentOfLatitude() * rmInertialToOrbital();
    }
};

// ============================================================================
// Mission Definitions
// ============================================================================

/// @brief Mission 1: Higher altitude orbit (reference_traj.hpp)
inline const ReferenceMission MISSION_1 = {
    .semiMajorAxis = 6903085.00,
    .longitudeAscendingNode = 260.861370 * degToRad,
    .inclination = 97.496919 * degToRad,
    .eccentricity = 8.851433e-06,
    .trueAnomaly = 154.051241 * degToRad,
    .argumentOfPeriapsis = 185.130932 * degToRad,
    .ldnLaunchsite = (174.7704 + 90.0) * degToRad,
    .lanLaunchsite = (174.7704 - 90.0) * degToRad,
    .aimingAzimuth = 191.47506 * degToRad,
    .latitude = 40.80768 * degToRad,
    .geocentricLatitude = 40.61741 * degToRad,
    .launchSiteLongitude = 100.13805 * degToRad,
    .heightLaunchSite = 1000.0,
    .velocityTerminal = {-7124.1826, 2631.5395, 249.9496},
    .positionTerminal = {-2405466.9, -12845047.6, 205013.6},
    .initialPosition = {-912205.4, -13212262.5, 149163.6},
    .initialVelocity = {-7440.6404, 953.8138, 294.5883},
    .initialMass = 6731.1,
    .initialSteeringAngles = {172.7593 * degToRad, 0.0, 0.0},
    .initialTime = 3140.3405,
    .cutoffTime = 3344.5194
};

/// @brief Mission 2: Lower altitude orbit (reference_traj2.hpp)
inline const ReferenceMission MISSION_2 = {
    .semiMajorAxis = 6740599.40,
    .longitudeAscendingNode = 272.847611 * degToRad,
    .inclination = 97.461979 * degToRad,
    .eccentricity = 2.402729e-02,
    .trueAnomaly = 0.451858 * degToRad,
    .argumentOfPeriapsis = 151.230742 * degToRad,
    .ldnLaunchsite = (174.6916 + 90.0) * degToRad,
    .lanLaunchsite = (174.6916 - 90.0) * degToRad,
    .aimingAzimuth = 191.47506 * degToRad,
    .latitude = 40.80768 * degToRad,
    .geocentricLatitude = 40.61741 * degToRad,
    .launchSiteLongitude = 100.13805 * degToRad,
    .heightLaunchSite = 1000.0,
    .velocityTerminal = {7677.0525, -1739.1305, -290.0255},
    .positionTerminal = {1427834.9, 45222.2, -153813.3},
    .initialPosition = {91748.3, 74465.8, -51380.7},
    .initialVelocity = {2211.1545, 1037.2763, -338.5107},
    .initialMass = 121175.5,
    .initialSteeringAngles = {27.0329 * degToRad, 0.0, 0.0},
    .initialTime = 149.8000,
    .cutoffTime = 470.4166
};

} // namespace trajsim
