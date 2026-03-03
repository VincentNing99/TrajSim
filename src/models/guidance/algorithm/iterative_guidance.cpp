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
    , rangeAngle(mission.argumentOfPeriapsis + mission.trueAnomaly)
    , steering(mission.initialSteeringAngles)
    , steeringLastStep(mission.initialSteeringAngles)
{
    timeToGo = mission.cutoffTime - mission.initialTime;
    timeToGoLastStep = timeToGo;
    buildRotationMatrix();
    terminalVelocity = rmInertialToTerminal * terminalState.velocity;
    terminalPosition = rmInertialToTerminal * (terminalState.position + positionLaunchSite);
}

SteeringAngles IterativeGuidance::computeSteering(const GuidanceInput& input) {
    const auto& state = input.state;
    double tau = state.vehicleMass / massFlowRate;

    double A = exitVelocity * log(tau / (tau - timeToGo));
    double J = exitVelocity * (tau * log(tau / (tau - timeToGo)) - timeToGo);
    double S = exitVelocity * ((tau - timeToGo) * log(tau / (tau - timeToGo)) - timeToGo);
    double Q = exitVelocity * (0.5 * std::pow(timeToGo, 2) + tau * ((tau - timeToGo) * log(tau / (tau - timeToGo)) - timeToGo));
    Vec3 g = 0.5 * (rmInertialToTerminal * input.gravity + rmInertialToTerminal * gravityCutoff);

    Mat3 rmOrbitalToTerminal = {std::cos(rangeAngle), std::sin(rangeAngle), 0,
                        -std::sin(rangeAngle), std::cos(rangeAngle), 0,
                        0, 0, 1};
    rmInertialToTerminal = rmOrbitalToTerminal * rmInertialToOrbital;
    instantaneousVelocity = rmInertialToTerminal * state.velocity;
    instantaneousPosition = rmInertialToTerminal * (state.position + positionLaunchSite);
    terminalVelocity = rmInertialToTerminal * terminalState.velocity;
    terminalPosition = rmInertialToTerminal * (terminalState.position + positionLaunchSite);

    computeDeltaV(g);
    convergeTimeToGo(g, tau);

    SteeringAngles velocityAngles = computeVelocitySteeringAngles();

    if (timeToGo <= config.steeringHoldTime) {
        // Hold steering commands for final seconds before cutoff
        timeToGo -= config.guidanceCycle;
        return steering;
    }

    SteeringAngles correctedAngles = applyPositionCorrections(velocityAngles, A, J, S, Q, g);

    SteeringAngles inertialAngles = transformSteeringToInertial(correctedAngles);


    steering = inertialAngles;
    steeringLastStep = steering;
    timeToGo -= config.guidanceCycle;
    return steering;
}

void IterativeGuidance::updateTimeToGoCorrectionBrun(const Vec3& g) {
    double a1 = dot(g, g);
    double b1 = 2 * (dot(instantaneousVelocity, g) - dot(terminalVelocity, g));

    double h;
    if (std::fabs(a1) < config.tolerance) {
        h = timeToGo;
    } else {
        h = -b1 / (2 * a1);
    }

    timeToGoLastStep = timeToGo;
    timeToGo = h;
}

void IterativeGuidance::updateTimeToGo(double tau) {
    if (tau - timeToGo < config.tolerance) {
        throw std::runtime_error("IGM failure: timeToGo exceeds propellant budget (tau - timeToGo <= 0)");
    }
    double A = exitVelocity * log(tau / (tau - timeToGo));

    if (std::fabs(A) < config.tolerance) {
        timeToGoLastStep = timeToGo;
        return;
    }

    double G = 0.5 * (std::pow(deltaV.dV, 2) / A - A);
    timeToGoLastStep = timeToGo;
    timeToGo = timeToGo + G * ((tau - timeToGo) / exitVelocity);
}

void IterativeGuidance::convergeTimeToGo(const Vec3& g, double tau) {
    int iteration = 0;
    if (config.timeToGoMethod == 1) {
        updateTimeToGoCorrectionBrun(g);
    } else {
        updateTimeToGo(tau);
    }
    while (std::fabs(timeToGo - timeToGoLastStep) >= config.timeToGoConvergenceTolerance && iteration < config.maxConvergenceIterations) {
        computeDeltaV(g);
        if (config.timeToGoMethod == 1) {
            updateTimeToGoCorrectionBrun(g);
        } else {
            updateTimeToGo(tau);
        }
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

void IterativeGuidance::computeDeltaV(const Vec3& g) {
    deltaV.dvXi = terminalVelocity.x - instantaneousVelocity.x - g.x * timeToGo;
    deltaV.dvEta = terminalVelocity.y - instantaneousVelocity.y - g.y * timeToGo;
    deltaV.dvZeta = instantaneousVelocity.z - terminalVelocity.z + g.z * timeToGo;
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

SteeringAngles IterativeGuidance::applyPositionCorrections(const SteeringAngles& velocityAngles, double A, double J, double S, double Q, const Vec3& g) {
    SteeringAngles corrected = velocityAngles;

    double Ay = A;
    double By = J;
    double Cy = S * std::cos(velocityAngles.psi);
    double Dy = Q * std::cos(velocityAngles.psi);
    double Ey = -terminalPosition.z + instantaneousPosition.z + instantaneousVelocity.z * timeToGo + 0.5 * g.z * std::pow(timeToGo, 2) + S * std::sin(velocityAngles.psi);

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
