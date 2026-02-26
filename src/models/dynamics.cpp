// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Vincent Ning

/// @file dynamics.cpp
/// @brief Equations of motion implementation.
///
/// Coordinate frames:
///   - Local frame: origin at launch site, Y = local vertical (up),
///     X-Z = local horizontal, rotated by aiming azimuth.
///   - Body frame: X = thrust axis (forward), Y = "up" in body, Z = "right".
///   - Euler sequence (3-2-1): phi (Z, pitch) → psi (Y, yaw) → gamma (X, roll).
///
/// Forces aggregated:
///   1. Thrust — along body X-axis, transformed to local via rmBodyToLocal.
///   2. Aerodynamic — body-frame forces from coefficient lookup, same transform.
///   3. Gravity — J2 oblate Earth, computed directly in local frame.
///   4. Earth rotation — subtracted from inertial velocity to get airspeed.

#include "models/dynamics.hpp"
#include "models/gravity.hpp"
#include <cmath>
#include <numbers>

namespace trajsim {

AeroAngles Dynamics::computeAeroAngles(const SteeringAngles& steering, const Vec3& velocity, const Vec3& position) const noexcept {
    // Geocentric position: add launch-site offset to get distance from Earth center
    Vec3 vehicleGeocentricRadius = position + gravity.getLaunchGeocentricRadiusVector();

    // Atmosphere co-rotation velocity: v_atm = ω_E × r_geocentric
    Vec3 omegaEarth = EarthConstants::EARTH_ROTATION_RATE * gravity.getPolarAxis();
    Vec3 velocityAir = cross(omegaEarth, vehicleGeocentricRadius);

    // Airspeed = inertial velocity − atmosphere velocity (no wind model)
    // Transform from local frame to body frame using inverse of body-to-local
    // Local-to-body = Rx(γ) · Ry(ψ) · Rz(φ)  (reverse order of body-to-local)
    Mat3 localToBody = Mat3::rotation_x(steering.gamma)
                     * Mat3::rotation_y(steering.psi)
                     * Mat3::rotation_z(steering.phi);
    velocityBody = localToBody * (velocity - velocityAir);

    double vMag = velocityBody.mag();
    if (vMag < 1e-12) {
        return AeroAngles{0.0, 0.0};
    }
    // Angle of attack: angle between velocity projection on XY plane and body X-axis
    double alpha = std::atan2(velocityBody.y, velocityBody.x);
    alpha = std::clamp(alpha, -std::numbers::pi / 2.0, std::numbers::pi / 2.0);

    // Sideslip angle: lateral velocity component relative to total speed
    double beta  = std::asin(std::clamp(velocityBody.z / vMag, -1.0, 1.0));
    return AeroAngles{alpha, beta};
}

Mat3 Dynamics::rmBodyToLocal(const SteeringAngles& angles) noexcept {
    // 3-2-1 Euler rotation: Rz(φ) · Ry(ψ) · Rx(γ)
    // Transforms vectors from body frame to local launch frame: v_local = R · v_body
    return Mat3::rotation_z(angles.phi)
         * Mat3::rotation_y(angles.psi)
         * Mat3::rotation_x(angles.gamma);
}

StateDerivative Dynamics::evaluate(const VehicleState& state,
                                    const SteeringAngles& steering,
                                    double thrustMagnitude,
                                    double massFlowRate,
                                    double t) const {
    // 1. Altitude check — skip aerodynamics above the atmosphere (~86 km)
    double altitude = gravity.computeAltitude(state.position);
    static constexpr double kAtmosphereCeiling = 86000.0;

    Vec3 aeroForceBody = Vec3::zero();
    if (altitude < kAtmosphereCeiling) {
        AeroAngles aeroAngles = computeAeroAngles(steering, state.velocity, state.position);
        AtmosphereState atmoState = atmosphere.computeStates(velocityBody, altitude);

        double alphaDeg = aeroAngles.alpha * (180.0 / std::numbers::pi);
        double betaDeg  = aeroAngles.beta  * (180.0 / std::numbers::pi);
        AeroResult aeroResult = vehicle.getAero().compute(atmoState.machNumber, alphaDeg, betaDeg,
                                                           atmoState.dynamicPressure);
        aeroForceBody = aeroResult.force;
    }

    // 2. Thrust acts along body X-axis
    Vec3 thrustBody{thrustMagnitude, 0.0, 0.0};

    // 3. Rotate total body-frame force (thrust + aero) into local frame
    Vec3 totalForceBody = thrustBody + aeroForceBody;
    Vec3 totalForceLocal = rmBodyToLocal(steering) * totalForceBody;

    // 4. Gravity acceleration computed directly in local frame (J2 model)
    Vec3 gravityAcc = gravity.computeAcceleration(state.position);

    // 5. Newton's second law: a = F/m + g
    double invMass = 1.0 / state.vehicleMass;
    Vec3 acceleration = totalForceLocal * invMass + gravityAcc;

    return StateDerivative{
        state.velocity,
        acceleration,
        -massFlowRate
    };
}

}  // namespace trajsim
