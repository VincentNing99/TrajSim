#pragma once
// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Vincent Ning

/// @file dynamics.hpp
/// @brief Equations of motion, aero angle computation, and frame transforms.
///
/// Dynamics is the central EOM evaluator. It aggregates forces from all sources
/// (thrust, aero, gravity, Coriolis) and produces state derivatives for the
/// integrator. It also computes kinematic quantities like angle of attack and
/// sideslip that are needed by the aero model.
///
/// Data flow:
///   Guidance → SteeringAngles → Dynamics → (alpha, beta) → Aero → forces
///                                       → gravity, Coriolis
///                                       → state derivatives → Integrator

#include "core/vec3.hpp"
#include "core/mat3.hpp"
#include "core/types.hpp"
#include "core/constants.hpp"
#include "models/gravity.hpp"
#include "models/atmosphere.hpp"
#include "models/vehicle/vehicle.hpp"
#include "models/guidance.hpp"

namespace trajsim {

// =============================================================================
// Aero Angles
// =============================================================================

/// @brief Aerodynamic angles computed from body-frame velocity.
struct AeroAngles {
    double alpha;    ///< Angle of attack [rad]
    double beta;     ///< Sideslip angle [rad]
};

// =============================================================================
// State Derivative
// =============================================================================

/// @brief State derivative output from EOM evaluation.
struct StateDerivative {
    Vec3 positionDot;    ///< dr/dt [m/s]
    Vec3 velocityDot;    ///< dv/dt [m/s²]
    double massDot;      ///< dm/dt [kg/s] (negative during burn)
};

// =============================================================================
// Dynamics
// =============================================================================

class Dynamics {
public:
    /// @brief Construct dynamics with environment and vehicle models.
    /// @param gravity    Gravity model (J2 oblate Earth)
    /// @param atmosphere Atmosphere model (US Std 1976)
    /// @param vehicle    Vehicle (owns aerodynamics and engine)
    Dynamics(const Gravity& gravity,
             const AtmosphereModel& atmosphere,
             const Vehicle& vehicle)
        : gravity(gravity)
        , atmosphere(atmosphere)
        , vehicle(vehicle)
    {}

    // =========================================================================
    // Kinematic Quantities
    // =========================================================================

    /// @brief Compute angle of attack and sideslip from body-frame velocity.
    /// @param velocityBody Velocity in body frame [m/s]
    /// @return Aero angles (alpha, beta, airspeed)
    [[nodiscard]] AeroAngles computeAeroAngles(const SteeringAngles& steering, const Vec3& velocity, const Vec3& position) const noexcept;

    /// @brief Build body-to-local rotation matrix from steering angles.
    /// @param angles Current steering angles (phi, psi, gamma)
    /// @return Rotation matrix: v_local = R * v_body
    [[nodiscard]] static Mat3 rmBodyToLocal(const SteeringAngles& angles) noexcept;

    // =========================================================================
    // EOM Evaluation
    // =========================================================================

    /// @brief Evaluate equations of motion at current state.
    /// @param state Current vehicle state
    /// @param steering Current steering angles
    /// @param thrustMagnitude Thrust force magnitude [N]
    /// @param massFlowRate Propellant mass flow rate [kg/s]
    /// @param t Current simulation time [s]
    /// @return State derivatives for the integrator
    [[nodiscard]] StateDerivative evaluate(const VehicleState& state,
                                            const SteeringAngles& steering,
                                            double thrustMagnitude,
                                            double massFlowRate,
                                            double t) const;

private:
    const Gravity& gravity;
    const AtmosphereModel& atmosphere;
    const Vehicle& vehicle;
    mutable Vec3 velocityBody;
};

}  // namespace trajsim
