// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Vincent Ning

/// @file iterative_guidance.cpp
/// @brief Iterative Guidance Mode (IGM) implementation.

#include "models/guidance/algorithm/iterative_guidance.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
#include <numbers>

namespace trajsim {

IterativeGuidance::IterativeGuidance(Config config,
                                     ExitCriteria exitCriteria,
                                     const ReferenceMission& mission,
                                     const VehicleState& vehicleState,
                                     Vec3 gravityCutoff,
                                     double exitVelocity,
                                     double massFlowRate)
    : config(config)
    , exitCriteria(exitCriteria)
    , vehicleState(vehicleState)
    , gravityCutoff(gravityCutoff)
    , exitVelocity(exitVelocity)
    , massFlowRate(massFlowRate)
    , aimingAzimuth(mission.aimingAzimuth)
    , launchLatitude(mission.latitude)
    , longitudeOfAnLaunchsite(mission.lanLaunchsite)
    , positionLaunchSite(mission.positionLaunchSite())
    , terminalState(terminalStateFromMission(mission))
    , steering(mission.initialSteeringAngles)
    , steeringLastStep(mission.initialSteeringAngles)
{
    timeToGo = mission.cutoffTime - mission.initialTime;
    timeToGoLastStep = timeToGo;
    buildRotationMatrix();
}

SteeringAngles IterativeGuidance::computeSteering(const GuidanceInput& input) {
    const auto& state = input.state;
    double tau = state.vehicleMass / massFlowRate;
    instantaneousVelocity = rmInertialToTerminal * state.velocity;
    instantaneousPosition = rmInertialToTerminal * (state.position + positionLaunchSite);
    terminalVelocity = rmInertialToTerminal * terminalState.velocity;
    terminalPosition = rmInertialToTerminal * (terminalState.position + positionLaunchSite);

    double thrust = std::abs(massFlowRate) * exitVelocity;
    double engineAcceleration = thrust / state.vehicleMass;

    // Average gravity over the remaining burn (linear approximation)
    Vec3 g = 0.5 * (rmInertialToTerminal * input.gravity + rmInertialToTerminal * gravityCutoff);

    computeDeltaV(g, instantaneousVelocity);
    convergeTimeToGo(g, instantaneousVelocity, tau, exitVelocity, engineAcceleration);

    SteeringAngles velocityAngles = computeVelocitySteeringAngles();

    // Decrement time-to-go by one guidance cycle (matches old code's set_time_to_go)
    timeToGo -= config.guidanceCycle;

    if (timeToGo <= config.steeringHoldTime) {
        // Hold steering commands for final seconds before cutoff
        return steering;
    }

    SteeringAngles correctedAngles = applyPositionCorrections(velocityAngles, tau, exitVelocity, g);

    SteeringAngles inertialAngles = transformSteeringToInertial(correctedAngles);

    // Return raw steering — rate limiting is handled by the coordinator
    steering = inertialAngles;
    steeringLastStep = steering;

    return steering;
}

void IterativeGuidance::updateTimeToGoLttw(const Vec3& g, const Vec3& vt, const Vec3& vc) {
    // Low Thrust-to-Weight (LTTW) time-to-go approximation.
    double a1 = dot(g, g);
    double b1 = 2 * (dot(vc, g) - dot(vt, g));

    double h;
    if (std::fabs(a1) < config.tolerance) {
        h = timeToGo;
    } else {
        h = -b1 / (2 * a1);
    }

    timeToGoLastStep = timeToGo;
    timeToGo = h;
}

void IterativeGuidance::updateTimeToGoHttw(double tau, double engineExitVelocity) {
    if (tau - timeToGo < config.tolerance) {
        throw std::runtime_error("IGM failure: timeToGo exceeds propellant budget (tau - timeToGo <= 0)");
    }

    double A = engineExitVelocity * log(tau / (tau - timeToGo));

    if (std::fabs(A) < config.tolerance) {
        timeToGoLastStep = timeToGo;
        return;
    }

    double G = 0.5 * (std::pow(deltaV.dV, 2) / A - A);
    timeToGoLastStep = timeToGo;
    timeToGo = timeToGo + G * ((tau - timeToGo) / engineExitVelocity);
}

void IterativeGuidance::convergeTimeToGo(const Vec3& g, const Vec3& vC, double tau, double engineExitVelocity, double engineAcceleration) {
    int iteration = 0;
    
    while (std::fabs(timeToGo - timeToGoLastStep) >= config.timeToGoConvergenceTolerance && iteration < config.maxConvergenceIterations) {
        computeDeltaV(g, vC);
        updateTimeToGoHttw(tau, engineExitVelocity);
        iteration++;
        if (std::isnan(timeToGo) || std::isinf(timeToGo)) {
            throw std::runtime_error("IGM convergence failure: timeToGo became NaN/infinity at iteration "
                                     + std::to_string(iteration));
        }
    }

    if (iteration >= config.maxConvergenceIterations) {
        std::cerr << "WARNING: IGM convergence did not complete in " << config.maxConvergenceIterations << " iterations" << std::endl;
    }
}

void IterativeGuidance::computeDeltaV(const Vec3& g, const Vec3& currentVelocity) {
    deltaV.dvXi = terminalVelocity.x - currentVelocity.x - g.x * timeToGo;
    deltaV.dvEta = terminalVelocity.y - currentVelocity.y - g.y * timeToGo;
    deltaV.dvZeta = currentVelocity.z - terminalVelocity.z + g.z * timeToGo;
    deltaV.dV = std::sqrt(std::pow(deltaV.dvXi, 2) + std::pow(deltaV.dvEta, 2) + std::pow(deltaV.dvZeta, 2));
}

SteeringAngles IterativeGuidance::computeVelocitySteeringAngles() {
    SteeringAngles angles{};

    if (deltaV.dvXi != 0) {
        angles.phi = std::atan2(deltaV.dvEta, deltaV.dvXi);
    } else {
        angles.phi = (deltaV.dvEta >= 0) ? std::numbers::pi / 2.0 : -std::numbers::pi / 2.0;
    }

    double delPitch = std::sqrt(std::pow(deltaV.dvXi, 2) + std::pow(deltaV.dvEta, 2));
    if (delPitch != 0) {
        angles.psi = std::atan2(deltaV.dvZeta, delPitch);
    } else {
        angles.psi = (deltaV.dvZeta >= 0) ? std::numbers::pi / 2.0 : -std::numbers::pi / 2.0;
    }

    return angles;
}

SteeringAngles IterativeGuidance::applyPositionCorrections(const SteeringAngles& velocityAngles, double tau, double engineExitVelocity, const Vec3& g) {
    SteeringAngles corrected = velocityAngles;
    double A = engineExitVelocity * log(tau / (tau - timeToGo));
    double J = engineExitVelocity * (tau * log(tau / (tau - timeToGo)) - timeToGo);
    double S = engineExitVelocity * ((tau - timeToGo) * log(tau / (tau - timeToGo)) - timeToGo);
    double Q = engineExitVelocity * (0.5 * std::pow(timeToGo, 2) + tau * ((tau - timeToGo) * log(tau / (tau - timeToGo)) - timeToGo));

    // Compute betaE/betaT in the orbital frame, matching old code's update_terminal_frame.
    // Position includes launch site offset (launch_inertial_to_ECSF equivalent).
    Vec3 posOrbital = rmInertialToOrbital * (vehicleState.position + positionLaunchSite);
    Vec3 velOrbital = rmInertialToOrbital * vehicleState.velocity;
    Vec3 terminalPosOrbital = rmInertialToOrbital * terminalPosition;
    double betaE = std::atan2(-posOrbital.x, posOrbital.y);
    double betaT;
    if (std::fabs(terminalPosition.y) < config.tolerance) {
        betaT = 0.0;
    } else {
        betaT = (velOrbital.x * timeToGo - S + 0.5 * g.x * std::pow(timeToGo, 2)) / terminalPosition.y;
    }
    double rangeAngle = (posOrbital.x < terminalPosOrbital.x) ? betaE + betaT : betaE - betaT;

    Mat3 rmOrbitalToTerminal = {std::cos(rangeAngle), std::sin(rangeAngle), 0,
                        -std::sin(rangeAngle), std::cos(rangeAngle), 0,
                        0, 0, 1};
    rmInertialToTerminal = rmOrbitalToTerminal * rmInertialToOrbital;

    // Retransform state vectors into the updated terminal frame
    terminalPosition = rmInertialToTerminal * (terminalState.position + positionLaunchSite);
    terminalVelocity = rmInertialToTerminal * terminalState.velocity;
    instantaneousPosition = rmInertialToTerminal * (vehicleState.position + positionLaunchSite);
    instantaneousVelocity = rmInertialToTerminal * vehicleState.velocity;

    double Ay = A;
    double By = J;
    double Cy = S * std::cos(velocityAngles.psi);
    double Dy = Q * std::cos(velocityAngles.psi);
    double Ey = instantaneousPosition.z + instantaneousVelocity.z * timeToGo + 0.5 * g.z * std::pow(timeToGo, 2) + S * std::sin(velocityAngles.psi);

    if (std::fabs(By) < config.tolerance) {
        return corrected;
    }

    double detY = By * Cy - Ay * Dy;
    if (std::fabs(detY) < config.tolerance) {
        return corrected;
    }
    double K3 = 0.0, K4 = 0.0;
    if (timeToGo > config.yawCorrectionStopTime) {
        K3 = By * Ey / detY;
        K4 = Ay * K3 / By;
    }

    double K1 = 0.0, K2 = 0.0;
    if (timeToGo > config.pitchCorrectionStopTime) {
        double Ap = Ay;
        double Bp = By;
        double Cp = Cy * std::cos(velocityAngles.phi);
        double Dp = Q * std::cos(velocityAngles.phi);
        double Ep = -terminalPosition.y + instantaneousPosition.y + instantaneousVelocity.y * timeToGo + 0.5 * g.y * std::pow(timeToGo, 2) - S * std::sin(velocityAngles.phi);

        double detP = Ap * Dp - Bp * Cp;
        if (std::fabs(detP) < config.tolerance) {
            return corrected;
        }

        K1 = Bp * Ep / detP;
        K2 = Ap * K1 / Bp;
    }

    corrected.phi = velocityAngles.phi + K2 * config.guidanceCycle - K1;
    corrected.psi = velocityAngles.psi + K4 * config.guidanceCycle - K3;

    return corrected;
}

SteeringAngles IterativeGuidance::transformSteeringToInertial(SteeringAngles correctedAngles) {
    Vec3 steeringCosTerminal = {std::cos(correctedAngles.psi) * std::cos(correctedAngles.phi),
                                std::cos(correctedAngles.psi) * std::sin(correctedAngles.phi),
                                std::sin(correctedAngles.psi)};
    Vec3 steeringCosInertial = rmInertialToTerminal.transposed() * steeringCosTerminal;

    SteeringAngles inertial{};
    inertial.psi = std::asin(steeringCosInertial[2]);

    if (std::fabs(std::cos(inertial.psi)) < config.tolerance) {
        throw std::runtime_error("IGM gimbal lock: Cannot transform steering angles when psi = +/-90 deg (cos(psi) = 0). "
                                 "Division by zero in phi calculation. Current psi = " + std::to_string(inertial.psi * radToDeg) + " degrees.");
    }

    inertial.phi = std::atan2(steeringCosInertial.y / std::cos(inertial.psi), steeringCosInertial.x / std::cos(inertial.psi));

    return inertial;
}

bool IterativeGuidance::exit(const VehicleState& state) const {
    if (!exitCriteria.root) {
        return false;
    }

    Vec3 positionECI = state.position + positionLaunchSite;
    Vec3 positionEquatorial = rmLaunchToEquatorial * positionECI;
    Vec3 velocityEquatorial = rmLaunchToEquatorial * state.velocity;

    double positionMag = positionEquatorial.mag();
    if (positionMag < config.tolerance) {
        return false;
    }

    double energy = std::pow(velocityEquatorial.mag(), 2) - 2 * EarthConstants::GRAVITATIONAL_CONST / positionMag;
    double semiMajorAxis = -EarthConstants::GRAVITATIONAL_CONST / energy;

    Vec3 angularMomentum = cross(positionEquatorial, velocityEquatorial);
    double angularMomentumMag = angularMomentum.mag();
    if (angularMomentumMag < config.tolerance) {
        return false;
    }

    double inclination = std::acos(angularMomentum.z / angularMomentumMag);

    Vec3 eccentricityVector = 1 / EarthConstants::GRAVITATIONAL_CONST * ((std::pow(velocityEquatorial.mag(), 2) - EarthConstants::GRAVITATIONAL_CONST / positionMag) * positionEquatorial - dot(positionEquatorial, velocityEquatorial) * velocityEquatorial);

    double eccentricity = eccentricityVector.mag();

    ExitContext ctx;
    ctx.set(static_cast<size_t>(ExitNode::Parameter::SemiMajorAxis), semiMajorAxis, terminalState.semiMajorAxis);
    ctx.set(static_cast<size_t>(ExitNode::Parameter::Eccentricity), eccentricity, terminalState.eccentricity);
    ctx.set(static_cast<size_t>(ExitNode::Parameter::Inclination), inclination * radToDeg, terminalState.inclination * radToDeg);
    return exitCriteria.root->evaluate(ctx);
}

OrbitalElements IterativeGuidance::computeOrbitalElements(const VehicleState& state) const {
    Vec3 positionECI = state.position + positionLaunchSite;
    Vec3 positionEquatorial = rmLaunchToEquatorial * positionECI;
    Vec3 velocityEquatorial = rmLaunchToEquatorial * state.velocity;

    double positionMag = positionEquatorial.mag();
    if (positionMag < config.tolerance) {
        return {0.0, 0.0, 0.0};
    }

    double energy = std::pow(velocityEquatorial.mag(), 2) - 2 * EarthConstants::GRAVITATIONAL_CONST / positionMag;
    double semiMajorAxis = -EarthConstants::GRAVITATIONAL_CONST / energy;

    Vec3 angularMomentum = cross(positionEquatorial, velocityEquatorial);
    double angularMomentumMag = angularMomentum.mag();
    if (angularMomentumMag < config.tolerance) {
        return {semiMajorAxis, 0.0, 0.0};
    }

    double inclination = std::acos(angularMomentum.z / angularMomentumMag);

    Vec3 eccentricityVector = 1 / EarthConstants::GRAVITATIONAL_CONST *
        ((std::pow(velocityEquatorial.mag(), 2) - EarthConstants::GRAVITATIONAL_CONST / positionMag) * positionEquatorial
         - dot(positionEquatorial, velocityEquatorial) * velocityEquatorial);

    return {semiMajorAxis, eccentricityVector.mag(), inclination};
}

void IterativeGuidance::buildRotationMatrix() {
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

}  // namespace trajsim
