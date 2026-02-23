#pragma once
// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Vincent Ning

/// @file types.hpp
/// @brief Common types shared across TrajSim modules

#include <cstdint>
#include <string_view>

namespace trajsim {

/// @brief Flight stage enumeration for simulation phase tracking.
enum class FlightStage : std::uint8_t {
    Prelaunch,      ///< On pad, pre-ignition
    Ignition,       ///< Engine start sequence
    Liftoff,        ///< Vehicle has left the pad
    MaxQ,           ///< Maximum dynamic pressure
    ThrottleDown,   ///< Thrust reduction phase
    Meco,           ///< Main engine cutoff
    Separation,     ///< Stage separation
    Coast,          ///< Unpowered flight
    Reentry,        ///< Atmospheric reentry
    Terminal        ///< End of simulation
};
/// @brief Convert FlightStage to string for logging/debug.
[[nodiscard]] constexpr std::string_view toString(FlightStage stage) noexcept {
    switch (stage) {
        case FlightStage::Prelaunch:     return "Prelaunch";
        case FlightStage::Ignition:      return "Ignition";
        case FlightStage::Liftoff:       return "Liftoff";
        case FlightStage::MaxQ:          return "MaxQ";
        case FlightStage::ThrottleDown:  return "ThrottleDown";
        case FlightStage::Meco:          return "Meco";
        case FlightStage::Separation:    return "Separation";
        case FlightStage::Coast:         return "Coast";
        case FlightStage::Reentry:       return "Reentry";
        case FlightStage::Terminal:      return "Terminal";
    }
    return "Unknown";
}

// ============================================================================
// Terminal State Definition
// ============================================================================

/// @brief Terminal orbital state at injection point.
/// @details Contains both Keplerian elements and Cartesian state vectors
///          defining the target conditions at engine cutoff.
struct TerminalState {
    // Orbital elements at insertion
    double trueAnomaly;          ///< True anomaly [rad].
    double argumentOfPeriapsis; ///< Argument of periapsis [rad].
    double longitudeOfAn;       ///< Longitude of ascending node [rad] from Greenwhich.
    double inclination;           ///< Orbital inclination [rad].
    double eccentricity;          ///< Orbital eccentricity [-].
    double semiMajorAxis;       ///< Semi-major axis [m].
    double argumentOfLatitude;           ///< argument of periapsis + true anomaly [rad].

    // Cartesian state at cutoff
    Vec3 position;                ///< Position at cutoff [m].
    Vec3 velocity;                ///< Velocity at cutoff [m/s].
};

/// @brief Vehicle kinematic state.
struct VehicleState {
    Vec3 position;       ///< Position in local frame [m]
    Vec3 velocity;       ///< Velocity in local frame [m/s]
    Vec3 acceleration;   ///< Acceleration in local frame [m/s²]
    double vehicleMass;  ///< Current mass [kg]
};

struct DeltaV {
    double dvXi;
    double dvEta;
    double dvZeta;
    double dV;
};

struct SteeringAngles {
    double phi;
    double psi;
    double gamma;

    [[nodiscard]] bool is_finite() const noexcept {
        return std::isfinite(phi) && std::isfinite(psi) && std::isfinite(gamma);
    }
};

}  // namespace trajsim
