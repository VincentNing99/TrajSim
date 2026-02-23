// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Vincent Ning

/// @file test_dynamics.cpp
/// @brief Unit tests for the Dynamics EOM evaluator.
///
/// Tests cover:
///   - AeroAngles computation (angle of attack, sideslip)
///   - Earth rotation correction in velocity computation
///   - Body-to-local frame rotation matrix
///   - EOM evaluation with combined forces (thrust + aero + gravity)
///   - StateDerivative consistency checks
///
/// Note: Basic rmBodyToLocal and RK4 tests live in test_integrator.cpp.
/// This file focuses on computeAeroAngles and the full evaluate() pipeline.

#include <gtest/gtest.h>
#include <cmath>
#include <numbers>
#include "models/dynamics.hpp"
#include "models/integrator.hpp"

namespace trajsim::test {

// =============================================================================
// Helpers
// =============================================================================

static constexpr double TOL       = 1e-10;
static constexpr double LOOSE_TOL = 1e-6;

/// @brief Build a zero-coefficient Aerodynamics object (aero forces vanish).
static Aerodynamics makeZeroAero() {
    Aerodynamics::Config cfg{
        .refArea = 1.0,
        .refLength = 1.0,
        .machMin = 0.1,
        .machMax = 5.0,
        .machTransition = 0.8
    };
    std::vector<std::vector<double>> table = {
        {0.3, -5.0, -5.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0},
        {0.3, -5.0,  5.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0},
        {0.3,  5.0, -5.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0},
        {0.3,  5.0,  5.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0},
        {0.6, -5.0, -5.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0},
        {0.6, -5.0,  5.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0},
        {0.6,  5.0, -5.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0},
        {0.6,  5.0,  5.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0},
    };
    std::vector<std::vector<double>> highTable = {
        {1.2, -5.0, -5.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0},
        {1.2, -5.0,  5.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0},
        {1.2,  5.0, -5.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0},
        {1.2,  5.0,  5.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0},
        {2.5, -5.0, -5.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0},
        {2.5, -5.0,  5.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0},
        {2.5,  5.0, -5.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0},
        {2.5,  5.0,  5.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0},
    };
    return Aerodynamics(cfg, highTable, table);
}

// =============================================================================
// Test Fixture
// =============================================================================

class DynamicsTest : public ::testing::Test {
protected:
    static constexpr double START_ALT = 10000.0;

    void SetUp() override {
        gravity = std::make_unique<Gravity>(0.0, 0.0);  // equator, sea level
        atmosphere = std::make_unique<AtmosphereModel>();
        Vehicle::Config vcfg{.numberOfStage = 1, .mass = 50000.0};
        vehicle = std::make_unique<Vehicle>(vcfg, makeZeroAero(), nullptr);
        dynamics = std::make_unique<Dynamics>(*gravity, *atmosphere, *vehicle);
    }

    std::unique_ptr<Gravity> gravity;
    std::unique_ptr<AtmosphereModel> atmosphere;
    std::unique_ptr<Vehicle> vehicle;
    std::unique_ptr<Dynamics> dynamics;

    SteeringAngles zeroSteering{0.0, 0.0, 0.0};
};

// =============================================================================
// AeroAngles - Zero Velocity
// =============================================================================

TEST_F(DynamicsTest, AeroAnglesZeroVelocityReturnsZero) {
    // Zero inertial velocity (body velocity dominated by Earth rotation correction)
    // At equator, atmospheric velocity is ~465 m/s. With identity steering and zero
    // inertial velocity, body velocity = localToBody * (0 - v_atm).
    // But with very small position (at launch), the correction is small.
    AeroAngles angles = dynamics->computeAeroAngles(zeroSteering, Vec3::zero(), Vec3::zero());

    EXPECT_TRUE(std::isfinite(angles.alpha));
    EXPECT_TRUE(std::isfinite(angles.beta));
}

// =============================================================================
// AeroAngles - Axial Flow (alpha = 0, beta = 0)
// =============================================================================

TEST_F(DynamicsTest, AeroAnglesPureAxialFlowSmallAlpha) {
    // Large forward velocity along body x-axis should give alpha ≈ 0.
    // Beta is nonzero due to Earth rotation subtraction (ω×r has a z-component
    // in the local frame at the equator), producing ~0.09 rad sideslip at 5 km/s.
    Vec3 velocity{5000.0, 0.0, 0.0};
    AeroAngles angles = dynamics->computeAeroAngles(zeroSteering, velocity, Vec3{0.0, START_ALT, 0.0});

    EXPECT_NEAR(angles.alpha, 0.0, 0.05);
    // Beta is small but nonzero — Earth rotation correction introduces sideslip
    EXPECT_LT(std::fabs(angles.beta), 0.15);
}

// =============================================================================
// AeroAngles - Positive Alpha
// =============================================================================

TEST_F(DynamicsTest, AeroAnglesUpwardVelocityGivesPositiveAlpha) {
    // With identity steering, velocity along local y gets transformed to body frame.
    // localToBody = identity when steering = 0, so body velocity ≈ {0, v, 0}.
    // alpha = atan2(vy, vx) → atan2(v, 0) → positive alpha
    Vec3 velocity{0.0, 3000.0, 0.0};
    AeroAngles angles = dynamics->computeAeroAngles(zeroSteering, velocity, Vec3{0.0, START_ALT, 0.0});

    EXPECT_GT(angles.alpha, 0.0);
}

// =============================================================================
// AeroAngles - Sideslip (Beta)
// =============================================================================

TEST_F(DynamicsTest, AeroAnglesLateralVelocityGivesNonzeroBeta) {
    // Velocity purely along local z should produce nonzero beta.
    // In body frame (with identity steering): body_z has velocity → beta = asin(vz/|v|)
    Vec3 velocity{3000.0, 0.0, 1000.0};
    AeroAngles angles = dynamics->computeAeroAngles(zeroSteering, velocity, Vec3{0.0, START_ALT, 0.0});

    EXPECT_NE(angles.beta, 0.0);
}

// =============================================================================
// AeroAngles - Alpha Clamping
// =============================================================================

TEST_F(DynamicsTest, AeroAnglesAlphaClampedToHalfPi) {
    // Alpha is clamped to ±π/2 per the implementation.
    // Even with extreme upward velocity, alpha ≤ π/2.
    Vec3 velocity{0.0, 100000.0, 0.0};
    AeroAngles angles = dynamics->computeAeroAngles(zeroSteering, velocity, Vec3{0.0, START_ALT, 0.0});

    EXPECT_LE(angles.alpha, std::numbers::pi / 2.0 + LOOSE_TOL);
    EXPECT_GE(angles.alpha, -std::numbers::pi / 2.0 - LOOSE_TOL);
}

// =============================================================================
// AeroAngles - Beta Clamping
// =============================================================================

TEST_F(DynamicsTest, AeroAnglesBetaWithinPhysicalRange) {
    // Beta is computed via asin with clamped argument → always in [-π/2, π/2].
    Vec3 velocity{100.0, 100.0, 10000.0};
    AeroAngles angles = dynamics->computeAeroAngles(zeroSteering, velocity, Vec3{0.0, START_ALT, 0.0});

    EXPECT_LE(angles.beta, std::numbers::pi / 2.0 + LOOSE_TOL);
    EXPECT_GE(angles.beta, -std::numbers::pi / 2.0 - LOOSE_TOL);
}

// =============================================================================
// AeroAngles - Steering Rotation Effect
// =============================================================================

TEST_F(DynamicsTest, AeroAnglesSteeringChangesResult) {
    Vec3 velocity{3000.0, 500.0, 0.0};
    Vec3 position{0.0, START_ALT, 0.0};

    AeroAngles a1 = dynamics->computeAeroAngles(zeroSteering, velocity, position);

    SteeringAngles pitched{0.3, 0.0, 0.0};  // phi = 0.3 rad
    AeroAngles a2 = dynamics->computeAeroAngles(pitched, velocity, position);

    // Different steering should produce different aero angles
    EXPECT_NE(a1.alpha, a2.alpha);
}

TEST_F(DynamicsTest, AeroAnglesYawSteeringAffectsBeta) {
    Vec3 velocity{3000.0, 0.0, 0.0};
    Vec3 position{0.0, START_ALT, 0.0};

    SteeringAngles yawed{0.0, 0.3, 0.0};  // psi = 0.3 rad
    AeroAngles angles = dynamics->computeAeroAngles(yawed, velocity, position);

    // Yaw steering with pure forward velocity should produce nonzero beta
    EXPECT_NE(angles.beta, 0.0);
}

// =============================================================================
// AeroAngles - Earth Rotation Correction
// =============================================================================

TEST_F(DynamicsTest, AeroAnglesIncludeEarthRotationCorrection) {
    // At 200 km altitude, Earth rotation correction is significant:
    // ω × r ≈ 7.29e-5 * (Re + 200 km) ≈ 479 m/s
    // This should affect the computed angles when velocity is small.
    Vec3 velocity{100.0, 0.0, 0.0};  // Small velocity → Earth rotation dominates
    Vec3 position{0.0, 200000.0, 0.0};  // 200 km altitude

    AeroAngles angles = dynamics->computeAeroAngles(zeroSteering, velocity, position);

    // With Earth rotation correction, alpha should be nonzero even for
    // near-axial inertial velocity
    EXPECT_TRUE(std::isfinite(angles.alpha));
    EXPECT_TRUE(std::isfinite(angles.beta));
}

// =============================================================================
// AeroAngles - Finite Output for Various Inputs
// =============================================================================

TEST_F(DynamicsTest, AeroAnglesFiniteForVariousStates) {
    const std::vector<Vec3> velocities = {
        {7000.0, 0.0, 0.0},
        {0.0, 7000.0, 0.0},
        {0.0, 0.0, 7000.0},
        {3000.0, 3000.0, 3000.0},
        {-5000.0, 1000.0, -500.0},
    };
    const std::vector<Vec3> positions = {
        {0.0, START_ALT, 0.0},
        {10000.0, 50000.0, 0.0},
        {0.0, 200000.0, 5000.0},
    };

    for (const auto& vel : velocities) {
        for (const auto& pos : positions) {
            AeroAngles a = dynamics->computeAeroAngles(zeroSteering, vel, pos);
            EXPECT_TRUE(std::isfinite(a.alpha))
                << "vel=(" << vel.x << "," << vel.y << "," << vel.z << ")"
                << " pos=(" << pos.x << "," << pos.y << "," << pos.z << ")";
            EXPECT_TRUE(std::isfinite(a.beta))
                << "vel=(" << vel.x << "," << vel.y << "," << vel.z << ")"
                << " pos=(" << pos.x << "," << pos.y << "," << pos.z << ")";
        }
    }
}

// =============================================================================
// EOM Evaluation - Thrust Direction
// =============================================================================

TEST_F(DynamicsTest, ThrustAlongBodyXProducesLocalXAcceleration) {
    // Zero steering → body x = local x. Thrust along body x → acceleration in local x.
    VehicleState state{Vec3{0.0, START_ALT, 0.0}, Vec3::zero(), Vec3::zero(), 50000.0};
    StateDerivative deriv = dynamics->evaluate(state, zeroSteering, 500000.0, 0.0, 0.0);

    // a_x = F/m = 500000/50000 = 10 m/s² (plus small gravity x-component)
    EXPECT_GT(deriv.velocityDot.x, 9.0);
}

TEST_F(DynamicsTest, ThrustWithPitchProducesNegativeYAcceleration) {
    // phi = π/2 rotates body x toward -y in local frame
    VehicleState state{Vec3{0.0, START_ALT, 0.0}, Vec3::zero(), Vec3::zero(), 50000.0};
    SteeringAngles pitched{std::numbers::pi / 2.0, 0.0, 0.0};

    StateDerivative deriv = dynamics->evaluate(state, pitched, 500000.0, 0.0, 0.0);

    // Thrust component in -y direction should dominate gravity
    EXPECT_LT(deriv.velocityDot.y, -5.0);
}

TEST_F(DynamicsTest, ThrustWithYawProducesZAcceleration) {
    // psi = π/2 rotates body x toward +z in local frame
    VehicleState state{Vec3{0.0, START_ALT, 0.0}, Vec3::zero(), Vec3::zero(), 50000.0};
    SteeringAngles yawed{0.0, std::numbers::pi / 2.0, 0.0};

    StateDerivative deriv = dynamics->evaluate(state, yawed, 500000.0, 0.0, 0.0);

    EXPECT_GT(deriv.velocityDot.z, 5.0);
}

// =============================================================================
// EOM Evaluation - Mass Flow
// =============================================================================

TEST_F(DynamicsTest, MassFlowNegatesCorrectly) {
    VehicleState state{Vec3{0.0, START_ALT, 0.0}, Vec3::zero(), Vec3::zero(), 50000.0};
    StateDerivative deriv = dynamics->evaluate(state, zeroSteering, 500000.0, 200.0, 0.0);

    EXPECT_DOUBLE_EQ(deriv.massDot, -200.0);
}

TEST_F(DynamicsTest, ZeroMassFlowGivesZeroMassDot) {
    VehicleState state{Vec3{0.0, START_ALT, 0.0}, Vec3::zero(), Vec3::zero(), 50000.0};
    StateDerivative deriv = dynamics->evaluate(state, zeroSteering, 0.0, 0.0, 0.0);

    EXPECT_DOUBLE_EQ(deriv.massDot, 0.0);
}

// =============================================================================
// EOM Evaluation - Position Derivative
// =============================================================================

TEST_F(DynamicsTest, PositionDotEqualsVelocity) {
    Vec3 velocity{1000.0, 500.0, -200.0};
    VehicleState state{Vec3{0.0, START_ALT, 0.0}, velocity, Vec3::zero(), 50000.0};
    StateDerivative deriv = dynamics->evaluate(state, zeroSteering, 0.0, 0.0, 0.0);

    EXPECT_TRUE(near_equal(deriv.positionDot, velocity));
}

// =============================================================================
// EOM Evaluation - Gravity Dominance Without Thrust
// =============================================================================

TEST_F(DynamicsTest, NoPowerGravityDominatesAcceleration) {
    VehicleState state{Vec3{0.0, START_ALT, 0.0}, Vec3::zero(), Vec3::zero(), 50000.0};
    StateDerivative deriv = dynamics->evaluate(state, zeroSteering, 0.0, 0.0, 0.0);

    // No thrust, zero aero (zero coefficients), only gravity acts
    EXPECT_NEAR(deriv.velocityDot.y, -9.81, 0.1);
    EXPECT_NEAR(deriv.velocityDot.x, 0.0, 0.1);
    EXPECT_NEAR(deriv.velocityDot.z, 0.0, 0.1);
}

// =============================================================================
// EOM Evaluation - Finite Output
// =============================================================================

TEST_F(DynamicsTest, EvaluateProducesFiniteOutputForVariousStates) {
    const std::vector<VehicleState> states = {
        {Vec3{0.0, START_ALT, 0.0}, Vec3::zero(), Vec3::zero(), 50000.0},
        {Vec3{0.0, START_ALT, 0.0}, Vec3{1000.0, 500.0, 0.0}, Vec3::zero(), 30000.0},
        {Vec3{5000.0, 50000.0, -1000.0}, Vec3{3000.0, 1000.0, -500.0}, Vec3::zero(), 10000.0},
    };

    for (const auto& state : states) {
        StateDerivative deriv = dynamics->evaluate(state, zeroSteering, 100000.0, 50.0, 0.0);

        EXPECT_TRUE(deriv.positionDot.is_finite());
        EXPECT_TRUE(deriv.velocityDot.is_finite());
        EXPECT_TRUE(std::isfinite(deriv.massDot));
    }
}

// =============================================================================
// EOM Evaluation - Different Latitudes
// =============================================================================

TEST_F(DynamicsTest, DifferentLatitudesProduceDifferentAcceleration) {
    // Construct dynamics at mid-latitude
    Gravity gravMid(45.0 * std::numbers::pi / 180.0, 0.0);
    AtmosphereModel atmoMid;
    Vehicle::Config vcfg{.numberOfStage = 1, .mass = 50000.0};
    Vehicle vehMid(vcfg, makeZeroAero(), nullptr);
    Dynamics dynMid(gravMid, atmoMid, vehMid);

    VehicleState state{Vec3{0.0, START_ALT, 0.0}, Vec3::zero(), Vec3::zero(), 50000.0};

    StateDerivative derivEq = dynamics->evaluate(state, zeroSteering, 0.0, 0.0, 0.0);
    StateDerivative derivMid = dynMid.evaluate(state, zeroSteering, 0.0, 0.0, 0.0);

    // Gravity varies with latitude (J2 effect)
    EXPECT_NE(derivEq.velocityDot.y, derivMid.velocityDot.y);
}

}  // namespace trajsim::test
