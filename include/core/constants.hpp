#pragma once
// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Vincent Ning

/// @file constants.hpp
/// @brief Physical constants and simulation-level configuration.

namespace trajsim {

// -------------------------------------------------------------------------
// Atmosphere properties
// -------------------------------------------------------------------------

struct AtmosphereConstants {
    constexpr static double TEMPERATURE = 288.0;           ///< Sea level temperature [K]
    constexpr static double DENSITY = 1.225;               ///< Sea level air density [kg/m³]
    constexpr static double SPEED_OF_SOUND = 343.0;        ///< Sea level speed of sound [m/s]
    constexpr static double VELOCITY = 0.0;                ///< Initial vehicle speed in body frame [m/s]
    constexpr static double DYNAMIC_PRESSURE = 0.0;        ///< Initial dynamic pressure [Pa]
    constexpr static double MACH = 0.0;                    ///< Initial Mach number [-]
    constexpr static double SEA_LEVEL_PRESSURE = 101325.0; ///< Sea level pressure [Pa]
    constexpr static double GAS_CONSTANT = 287.05287;      ///< Specific gas constant for air [J/(kg·K)]
    constexpr static double GAMMA = 1.4;                   ///< Ratio of specific heats [-]
};

// -------------------------------------------------------------------------
// Earth constants
// -------------------------------------------------------------------------

struct EarthConstants {
    constexpr static double GRAVITATIONAL_CONST = 3.986005e14;        ///< Gravitational parameter [m³/s²]
    constexpr static double MASS_OF_EARTH = 5.972e24;                 ///< Mass of the Earth [kg]
    constexpr static double FLATTENING = 1.0 / 298.257223563;        ///< Geodetic flattening [-]
    constexpr static double RADIUS_OF_EQUATOR = 6378137.0;           ///< Equatorial radius [m]
    constexpr static double SEMI_MINOR_AXIS = RADIUS_OF_EQUATOR * (1 - FLATTENING); ///< Semi-minor axis [m]
    constexpr static double J2 = 1.08263e-3;                         ///< J2 perturbation [-]
    constexpr static double J4 = -2.37091e-6;                        ///< J4 perturbation [-]
    constexpr static double J = (3.0 / 2.0) * J2;                   ///< 3/2 * J2
    constexpr static double EARTH_ROTATION_RATE = 7.292115e-5;       ///< Earth rotation rate [rad/s]
    constexpr static double GRAVITATIONAL_ACCELERATION = 9.806650;   ///< Gravitational acceleration [m/s²]
    constexpr static double FIRST_ECCENTRICITY = 0.0818191908426;    ///< First eccentricity [-]
    constexpr static double SECOND_ECCENTRICITY = 0.0820944379497;   ///< Second eccentricity [-]
};

}  // namespace trajsim
