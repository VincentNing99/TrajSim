// models/guidance.cpp
// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Vincent Ning

/// @file guidance.cpp
/// @brief Implementation of guidance model.
///
/// Iterative Guidance Mode (IGM) algorithm overview:
///   1. Transform state to terminal guidance frame.
///   2. Converge time-to-go via iterative updates (LTTW or HTTW path).
///   3. Compute delta-V: velocity deficit between current and terminal state.
///   4. Derive velocity-based steering angles from delta-V components.
///   5. Apply position corrections (range angle, altitude targeting).
///   6. Transform corrected steering back to inertial frame.
///   7. Rate-limit steering commands to prevent structural loads.

#include "models/guidance.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
#include <numbers>

namespace trajsim {

SteeringAngles Guidance::openLoopGuidance(double t) const {
    const double elapsed = t - timeInitial;

    // Clamp to first point if before profile start
    if (elapsed <= steeringProfile[0][0]) {
        return SteeringAngles{
            steeringProfile[0][1],  // phi (pitch)
            steeringProfile[0][2],  // psi (yaw)
            steeringProfile[0][3]   // gamma (roll)
        };
    }

    // Clamp to last point if after profile end
    const auto& last = steeringProfile.back();
    if (elapsed >= last[0]) {
        return SteeringAngles{last[1], last[2], last[3]};
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
    if (std::fabs(dt) < guidanceCfg.tolerance) {
        return SteeringAngles{p0[1], p0[2], p0[3]};
    }

    const double alpha = (elapsed - p0[0]) / dt;

    return SteeringAngles{
        p0[1] + alpha * (p1[1] - p0[1]),  // phi (pitch)
        p0[2] + alpha * (p1[2] - p0[2]),  // psi (yaw)
        p0[3] + alpha * (p1[3] - p0[3])   // gamma (roll)
    };
}

SteeringAngles Guidance::iterativeGuidance(double t, const VehicleState& state,
                                  double massFlowRate, double engineExitVelocity,
                                  Vec3 g0, Vec3 gCutoff) {
    // τ = m / ṁ  — characteristic time for propellant depletion
    double tau = -state.vehicleMass / massFlowRate;

    // Transform all vectors into the terminal guidance frame for targeting
    instantaneousVelocity = rmInertialToTerminal * state.velocity;
    instantaneousPosition = rmInertialToTerminal * state.position;
    terminalVelocity = rmInertialToTerminal * terminalState.velocity;
    terminalPosition = rmInertialToTerminal * terminalState.position;

    // Thrust-to-weight ratio selects the time-to-go update method
    double thrust = massFlowRate * engineExitVelocity;
    double thrustToWeight = thrust / (state.vehicleMass * EarthConstants::GRAVITATIONAL_ACCELERATION);

    // Average gravity over the remaining burn (linear approximation)
    Vec3 g = 0.5 * (rmInertialToTerminal * g0 + rmInertialToTerminal * gCutoff);

    // Iteratively solve for time-to-go until convergence
    convergeTimeToGo(g, instantaneousVelocity, tau, engineExitVelocity, thrustToWeight);

    computeDeltaV(g, instantaneousVelocity);

    SteeringAngles velocityAngles = computeVelocitySteeringAngles();

    if (timeToGo <= guidanceCfg.steeringHoldTime[currentStage]) {
        // Hold steering commands for final seconds before cutoff
        return steering;
    }

    SteeringAngles correctedAngles = applyPositionCorrections(velocityAngles, tau, engineExitVelocity, g);

    SteeringAngles inertial_angles = transformSteeringToInertial(correctedAngles);
    double dt = guidanceCfg.guidanceCycle[currentStage];
    double max_rate =  guidanceCfg.maxSteeringRate * degToRad; // radians per second
    double del_phi = std::clamp((inertial_angles.phi - steeringLastStep.phi) / dt, -max_rate, max_rate);
    double del_psi = std::clamp((inertial_angles.psi - steeringLastStep.psi) / dt, -max_rate, max_rate);

    steering.phi = steeringLastStep.phi + del_phi * dt;
    steering.psi = steeringLastStep.psi + del_psi * dt;

    steeringLastStep = steering;

    return steering;  // pitch, yaw, roll
}

void Guidance::updateTimeToGoLttw(double delV, double tau, const Vec3& g, const Vec3& vt, const Vec3& vc)
{
    // Low Thrust-to-Weight (LTTW) time-to-go approximation.
    // Solves for t_go from: |v_t - v_c - g·t|² = ΔV²
    // Expanding: a·t² + b·t + c = 0, where a = |g|², b = 2(v_c - v_t)·g
    double a1 = dot(g, g);
    double b1 = 2 * (dot(vc, g) - dot(vt, g));

    // Check for division by zero in h calculation
    double h;
    if (std::fabs(a1) < guidanceCfg.tolerance) {
        h = timeToGo;  // Keep current value if can't compute
    } else {
        h = -b1 / (2 * a1);
    }

    timeToGoLastStep = timeToGo;
    timeToGo = h;
}

void Guidance::updateTimeToGoHttw(double delV, double tau, double engineExitVelocity, const Vec3& g, const Vec3& vt, const Vec3& vc)
{
    // High Thrust-to-Weight (HTTW) time-to-go update using the rocket equation.
    // A = v_e · ln(τ / (τ - t_go))  — ideal ΔV from Tsiolkovsky equation
    // G = 0.5·(ΔV²/A - A)           — correction term
    // t_go_new = t_go + G·(τ - t_go)/v_e

    if (tau - timeToGo < guidanceCfg.tolerance) {
        throw std::runtime_error("IGM failure: timeToGo exceeds propellant budget (tau - timeToGo <= 0)");
    }

    double A = engineExitVelocity * log(tau / (tau - timeToGo));

    if (std::fabs(A) < guidanceCfg.tolerance) {
        timeToGoLastStep = timeToGo;
        return;
    }

    double G = 0.5 * (std::pow(delV, 2) / A - A);
    timeToGoLastStep = timeToGo;
    timeToGo = timeToGo + G * ((tau - timeToGo) / engineExitVelocity);
}

void Guidance::convergeTimeToGo(const Vec3& g, const Vec3& vC, double tau, double engineExitVelocity, double thrustToWeight)
{
    // Initial time-to-go estimate to ensure convergence loop executes
    if (thrustToWeight < 1.0)
    {
        updateTimeToGoLttw(deltaV.dV, tau, g, terminalVelocity, vC);
    }
    else
    {
        updateTimeToGoHttw(deltaV.dV, tau, engineExitVelocity, g, terminalVelocity, vC);
    }

    int iteration = 0;

    while(std::fabs(timeToGo - timeToGoLastStep) >= guidanceCfg.timeToGoConvergenceTolerance[currentStage] && iteration < guidanceCfg.maxConvergenceIterations[currentStage])
    {
        if (thrustToWeight < 1.0)
        {
            updateTimeToGoLttw(deltaV.dV, tau, g, terminalVelocity, vC);
        }
        else
        {
            updateTimeToGoHttw(deltaV.dV, tau, engineExitVelocity, g, terminalVelocity, vC);
        }
        iteration++;
        // Safety check for NaN/infinity - throw exception for flight-critical failure
        if (std::isnan(timeToGo) || std::isinf(timeToGo)) {
            throw std::runtime_error("IGM convergence failure: timeToGo became NaN/infinity at iteration "
                                     + std::to_string(iteration));
        }
    }

    if (iteration >= guidanceCfg.maxConvergenceIterations[currentStage]) {
        std::cerr << "WARNING: IGM convergence did not complete in " << guidanceCfg.maxConvergenceIterations[currentStage] << " iterations" << std::endl;
    }
}

void Guidance::computeDeltaV(const Vec3& g, const Vec3& currentVelocity)
{
    // Required ΔV = v_terminal - v_current - g · t_go
    // Subtracts the velocity gained from gravity during the remaining burn
    deltaV.dvXi = terminalVelocity.x - currentVelocity.x - g.x * timeToGo;
    deltaV.dvEta = terminalVelocity.y - currentVelocity.y - g.y * timeToGo;
    deltaV.dvZeta = terminalVelocity.z - currentVelocity.z - g.z * timeToGo;
    deltaV.dV = std::sqrt(std::pow(deltaV.dvXi, 2) + std::pow(deltaV.dvEta, 2) + std::pow(deltaV.dvZeta, 2));
}

SteeringAngles Guidance::computeVelocitySteeringAngles()
{
    // Pitch angle: φ = atan2(Δv_η, Δv_ξ)  — angle in the ξ-η plane
    SteeringAngles angles;

    if (deltaV.dvXi != 0) {
        angles.phi = std::atan2(deltaV.dvEta, deltaV.dvXi);
    } else {
        angles.phi = (deltaV.dvEta >= 0) ? std::numbers::pi / 2.0 : -std::numbers::pi / 2.0;
    }

    // Yaw angle: ψ = atan2(Δv_ζ, |Δv_pitch|) — out-of-plane steering
    double delPitch = std::sqrt(std::pow(deltaV.dvXi, 2) + std::pow(deltaV.dvEta, 2));
    if (delPitch != 0) {
        angles.psi = std::atan2(deltaV.dvZeta, delPitch);
    } else {
        angles.psi = (deltaV.dvZeta >= 0) ? std::numbers::pi / 2.0 : -std::numbers::pi / 2.0;
    }

    return angles;
}

SteeringAngles Guidance::applyPositionCorrections(const SteeringAngles& velocityAngles, double tau, double engineExitVelocity, const Vec3& g)
{
    // Position correction uses integrals of the thrust acceleration profile:
    //   A = v_e · ln(τ/(τ-t))              — total ΔV (Tsiolkovsky)
    //   J = v_e · [τ·ln(τ/(τ-t)) - t]      — first moment (velocity × time)
    //   S = v_e · [(τ-t)·ln(τ/(τ-t)) - t]  — position integral
    //   Q = v_e · [½t² + τ·S/v_e]          — second moment
    // These are used to solve a 2×2 linear system for steering corrections
    // K1, K2 (pitch) and K3, K4 (yaw) that drive position error to zero.
    SteeringAngles corrected = velocityAngles;
    double A = engineExitVelocity * log(tau / (tau - timeToGo));
    double J = engineExitVelocity * (tau * log(tau/(tau-timeToGo)) - timeToGo);
    double S = engineExitVelocity * ((tau - timeToGo) * log(tau / (tau - timeToGo)) - timeToGo);
    double Q = engineExitVelocity * (0.5 * std::pow(timeToGo, 2) + tau * ((tau - timeToGo) * log(tau / (tau - timeToGo)) - timeToGo));

    // Range angle: current orbital angle from ascending node
    double betaE = std::atan2(-instantaneousPosition.x, instantaneousPosition.y);
    double betaT;
    if (std::fabs(instantaneousPosition.y) < guidanceCfg.tolerance) {
        betaT = 0.0;  // At equator crossing - no range angle correction
    } else {
        betaT = (instantaneousVelocity.x * timeToGo - S + 0.5 * g.x * std::pow(timeToGo, 2)) / instantaneousPosition.y;
    }
    double rangeAngle;

    if (betaE + betaT > terminalState.argumentOfLatitude)
    {
        rangeAngle = betaE - betaT;
    }
    else
    {
        rangeAngle = betaE + betaT;
    }

    // Update the terminal frame rotation using the corrected range angle
    Mat3 rmOrbitalToTerminal = {std::cos(rangeAngle), std::sin(rangeAngle), 0,
                        -std::sin(rangeAngle), std::cos(rangeAngle), 0,
                        0, 0, 1};
    rmInertialToTerminal = rmOrbitalToTerminal * rmInertialToOrbital;


    double Ay = A;
    double By = J;
    double Cy = S * std::cos(velocityAngles.psi);
    double Dy = Q * std::cos(velocityAngles.psi);
    double Ey = instantaneousPosition.z + instantaneousVelocity.z * timeToGo + 0.5 * g.z * std::pow(timeToGo, 2) + S * std::sin(velocityAngles.psi);

    // Guard: J (By) near zero - position correction matrix singular
    if (std::fabs(By) < guidanceCfg.tolerance) {
        return corrected;  // Return velocity angles without position correction
    }

    double detY = By * Cy - Ay * Dy;
    if (std::fabs(detY) < guidanceCfg.tolerance) {
        return corrected;  // Singular matrix - skip yaw correction
    }

    double K3 = By * Ey / detY;
    double K4 = Ay * K3 / By;

    double Ap = Ay;
    double Bp = By;
    double Cp = Cy * std::cos(velocityAngles.phi);
    double Dp = Q * std::cos(velocityAngles.phi);
    double Ep = -terminalPosition.y + instantaneousPosition.y + instantaneousVelocity.y * timeToGo + 0.5 * g.y * std::pow(timeToGo, 2) - S * std::sin(velocityAngles.phi);

    double detP = Ap * Dp - Bp * Cp;
    if (std::fabs(detP) < guidanceCfg.tolerance) {
        return corrected;  // Singular matrix - skip pitch correction
    }

    double K1 = Bp * Ep / detP;
    double K2 = Ap * K1 / Bp;

    corrected.phi = velocityAngles.phi + K2 * guidanceCfg.guidanceCycle[currentStage] - K1;
    corrected.psi = velocityAngles.psi + K4 * guidanceCfg.guidanceCycle[currentStage] - K3;

    return corrected;
}

SteeringAngles Guidance::transformSteeringToInertial(SteeringAngles correctedAngles)
{
    // Convert steering angles to a unit direction vector in the terminal frame,
    // then rotate back to the inertial frame using R^T (inverse of orthogonal R).
    Vec3 steeringCosTerminal = {std::cos(correctedAngles.psi) * std::cos(correctedAngles.phi),
                                std::cos(correctedAngles.psi) * std::sin(correctedAngles.phi),
                                std::sin(correctedAngles.psi)};
    Vec3 steeringCosInertial = rmInertialToTerminal.transposed() * steeringCosTerminal;

    SteeringAngles inertial;
    inertial.psi = std::asin(steeringCosInertial[2]);

    // Check for gimbal lock condition (psi near ±90°)
    if (std::fabs(std::cos(inertial.psi)) < guidanceCfg.tolerance) {
        throw std::runtime_error("IGM gimbal lock: Cannot transform steering angles when psi = ±90° (cos(psi) = 0). "
                                 "Division by zero in phi calculation. Current psi = " + std::to_string(inertial.psi * radToDeg) + " degrees.");
    }

    inertial.phi = std::atan2(steeringCosInertial.y / std::cos(inertial.psi), steeringCosInertial.x / std::cos(inertial.psi));

    return inertial;
}

bool Guidance::cutoff(const VehicleState& state) const noexcept
{
    // Transform from local launch frame to Earth-centered equatorial frame
    // for orbital element computation
    Vec3 positionECI = state.position + positionLaunchSite;
    Vec3 positionEquatorial = rmLaunchToEquatorial * positionECI;
    Vec3 velocityEquatorial = rmLaunchToEquatorial * state.velocity;

    double positionMag = positionEquatorial.mag();
    if (positionMag < guidanceCfg.tolerance) {
        return false;  // Invalid state: position at origin
    }

    // Vis-viva: ε = v² - 2μ/r  →  a = -μ/ε
    double energy = std::pow(velocityEquatorial.mag(), 2) - 2 * EarthConstants::GRAVITATIONAL_CONST / positionMag;
    double semiMajorAxis = -EarthConstants::GRAVITATIONAL_CONST / energy;

    // Angular momentum: h = r × v  (defines orbital plane)
    Vec3 angularMomentum = cross(positionEquatorial, velocityEquatorial);
    double angularMomentumMag = angularMomentum.mag();
    if (angularMomentumMag < guidanceCfg.tolerance) {
        return false;  // Invalid state: no angular momentum (radial trajectory)
    }

    double radialVelocity = dot(positionEquatorial, velocityEquatorial) / positionMag;
    // Inclination: i = acos(h_z / |h|)
    double inclination = std::acos(angularMomentum.z / angularMomentumMag);

    // Right ascension of ascending node (RAAN): Ω from node vector n = z × h
    Vec3 nodeVector = cross(Vec3::unit_z(), angularMomentum);
    double nodeVectorMag = nodeVector.mag();
    double raan = 0.0;
    if (nodeVectorMag > guidanceCfg.tolerance) {
        raan = std::acos(nodeVector.x / nodeVectorMag);
        if (nodeVector.y < 0) {
            raan = 2.0 * std::numbers::pi - raan;
        }
        // else: raan keeps the computed value from acos
    }

    // Eccentricity vector: e = (1/μ)·[(v² - μ/r)·r - (r·v)·v]  — points toward periapsis
    Vec3 eccentricityVector = 1 / EarthConstants::GRAVITATIONAL_CONST * ((std::pow(velocityEquatorial.mag(), 2) - EarthConstants::GRAVITATIONAL_CONST / positionMag) * positionEquatorial - dot(positionEquatorial, velocityEquatorial) * velocityEquatorial);

    double eccentricity = eccentricityVector.mag();

    // argument of perigee
    [[maybe_unused]] double argPerigee = 0.0;
    if (nodeVectorMag != 0 && eccentricity > guidanceCfg.tolerance) {
        argPerigee = std::acos(dot(nodeVector, eccentricityVector) / (nodeVectorMag * eccentricity));
        if (eccentricityVector.z < 0) {
            argPerigee = 2.0 * std::numbers::pi - argPerigee;
        }
    }

    // true anomaly
    [[maybe_unused]] double trueAnomaly = 0.0;
    if (eccentricity > guidanceCfg.tolerance) {
        trueAnomaly = std::acos(dot(eccentricityVector, positionEquatorial) / (eccentricity * positionMag));
        if (radialVelocity < 0) {
            trueAnomaly = 2.0 * std::numbers::pi - trueAnomaly;
        }
    }

    // Cutoff when: SMA exceeds target AND eccentricity/inclination within tolerance
    if (semiMajorAxis - terminalState.semiMajorAxis > 0 &&
        std::fabs(eccentricity - terminalState.eccentricity) <= guidanceCfg.eccentricityTolerance[currentStage] &&
        std::fabs(inclination - terminalState.inclination) <= guidanceCfg.inclinationTolerance[currentStage]) {
        return true;
    }

    return false;
}

void Guidance::buildRotationMatrix()
{
    rmLaunchToEquatorial = {
            -std::cos(aimingAzimuth) * std::sin(launchLatitude), std::cos(launchLatitude), std::sin(aimingAzimuth) * std::sin(launchLatitude),
            std::sin(aimingAzimuth), 0.0, std::cos(aimingAzimuth),
            std::cos(aimingAzimuth) * std::cos(launchLatitude), std::sin(launchLatitude), -std::sin(aimingAzimuth) * std::cos(launchLatitude)
    };
    Mat3 rmLanRotation = {
        std::cos(longitudeOfAnLaunchsite), std::sin(longitudeOfAnLaunchsite), 0.0,
        -std::sin(longitudeOfAnLaunchsite), std::cos(longitudeOfAnLaunchsite), 0.0,
        0.0, 0.0, 1.0
    };
    Mat3 rmInclinationRotation = {
        std::cos(terminalState.inclination), 0.0, -std::sin(terminalState.inclination),
        0.0, 1.0, 0.0,
        std::sin(terminalState.inclination), 0.0, std::cos(terminalState.inclination)
    };
    Mat3 rmArgumentOfLatitude = {
        std::cos(terminalState.argumentOfLatitude), std::sin(terminalState.argumentOfLatitude), 0,
        -std::sin(terminalState.argumentOfLatitude), std::cos(terminalState.argumentOfLatitude), 0,
        0, 0, 1
    };
    rmInertialToOrbital = rmInclinationRotation * (rmLanRotation * rmLaunchToEquatorial);
    rmInertialToTerminal = rmArgumentOfLatitude * rmInertialToOrbital;
}

SteeringAngles Guidance::computeSteering(double t,
                                          double massFlowRate,
                                          double engineExitVelocity,
                                          Vec3 g0,
                                          Vec3 gCutoff) {
    // Dispatch based on current stage's guidance mode
    switch (guidanceCfg.mode[currentStage]) {
        case GuidanceMode::OpenLoop:
            return openLoopGuidance(t);

        case GuidanceMode::IGM:
            return iterativeGuidance(t, vehicleState, massFlowRate, engineExitVelocity, g0, gCutoff);
    }

    // Should never reach here (all enum cases handled)
    throw std::logic_error("Unhandled guidance mode in computeSteering");
}

}  // namespace trajsim