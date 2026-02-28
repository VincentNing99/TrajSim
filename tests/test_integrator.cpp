// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Vincent Ning

/// @file test_integrator.cpp
/// @brief Unit tests for the RK4 integrator and dynamics module.
///
/// Tests verify:
///   - AeroAngles computation from body-frame velocity
///   - Body-to-local rotation matrix construction (passive, 3-2-1 Z-Y-X)
///   - RK4 correctness against analytical free-fall
///   - RK4 4th-order convergence for nonlinear ODE (mass-depleting thrust)
///   - Mass conservation and depletion
///   - Const-correctness (original state not modified)
///   - Physical consistency (energy, direction, symmetry)

#include <gtest/gtest.h>
#include <cmath>
#include <numbers>
#include "models/integrator.hpp"

namespace trajsim::test {

// =============================================================================
// Helpers
// =============================================================================

static constexpr double TOL        = 1e-10;
static constexpr double LOOSE_TOL  = 1e-6;

/// @brief Build a minimal Aerodynamics object for testing.
/// All coefficients are zero so aero forces vanish — isolates gravity + thrust.
static Aerodynamics makeZeroAero() {
    Aerodynamics::Config cfg{
        .refArea = 1.0,
        .refLength = 1.0,
        .machMin = 0.1,
        .machMax = 5.0,
        .machTransition = 0.8
    };
    // 2x2x2 tables with all coefficients = 0
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

/// @brief All vehicle states start at 10 km altitude to avoid negative-altitude
///        assertions in the atmosphere model during downward integration.
class IntegratorTest : public ::testing::Test {
protected:
    static constexpr double START_ALT = 10000.0;  // 10 km above launch site

    // Thrust parameters for powered flight tests
    static constexpr double POWERED_THRUST    = 600000.0;  // 200 kg/s * 3000 m/s
    static constexpr double POWERED_MASS_FLOW = 200.0;

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

    // Zero steering (identity rotation)
    SteeringAngles zeroSteering{0.0, 0.0, 0.0};

    // A stationary state at altitude
    VehicleState stationaryState() const {
        return VehicleState{
            Vec3{0.0, START_ALT, 0.0},
            Vec3::zero(),
            Vec3::zero(),
            10000.0
        };
    }

    // A coasting state (moving upward, no thrust)
    VehicleState coastingState() const {
        return VehicleState{
            Vec3{0.0, START_ALT, 0.0},
            Vec3{0.0, 1000.0, 0.0},
            Vec3::zero(),
            10000.0
        };
    }

    // A powered state (thrusting along body x-axis)
    VehicleState poweredState() const {
        return VehicleState{
            Vec3{0.0, START_ALT, 0.0},
            Vec3{0.0, 100.0, 0.0},
            Vec3::zero(),
            50000.0
        };
    }
};

// =============================================================================
// AeroAngles Tests
// =============================================================================

// AeroAngles tests removed — computeAeroAngles signature changed
// (now non-static, includes Earth rotation subtraction and frame transform).
// These tests need to be rewritten for the new API.

// =============================================================================
// Body-to-Local Rotation Tests
// =============================================================================

TEST_F(IntegratorTest, ZeroAnglesGiveIdentity) {
    Mat3 R = Dynamics::rmBodyToLocal({0.0, 0.0, 0.0});
    EXPECT_TRUE(near_equal(R, Mat3::identity()));
}

TEST_F(IntegratorTest, RotationIsOrthogonal) {
    SteeringAngles angles{0.3, 0.5, 0.7};
    Mat3 R = Dynamics::rmBodyToLocal(angles);
    EXPECT_TRUE(R.is_rotation());
}

TEST_F(IntegratorTest, PureGammaRotation) {
    // gamma controls X-axis rotation (roll)
    // bodyToLocal = Rx(-γ). At γ=π/2: Rx(-π/2) maps body y → local +z
    SteeringAngles angles{0.0, 0.0, std::numbers::pi / 2.0};  // phi=0, psi=0, gamma=π/2
    Mat3 R = Dynamics::rmBodyToLocal(angles);

    // Test body Y axis transformation
    Vec3 bodyY = R * Vec3{0.0, 1.0, 0.0};
    EXPECT_NEAR(bodyY.x, 0.0, TOL);
    EXPECT_NEAR(bodyY.y, 0.0, TOL);
    EXPECT_NEAR(bodyY.z, 1.0, TOL);
}

TEST_F(IntegratorTest, PurePsiRotation) {
    // psi controls Y-axis rotation (yaw)
    // bodyToLocal = Ry(-ψ). At ψ=π/2: Ry(-π/2) maps body x → local -z
    SteeringAngles angles{0.0, std::numbers::pi / 2.0, 0.0};  // phi=0, psi=π/2, gamma=0
    Mat3 R = Dynamics::rmBodyToLocal(angles);

    // Test body X axis transformation
    Vec3 bodyX = R * Vec3{1.0, 0.0, 0.0};
    EXPECT_NEAR(bodyX.x, 0.0, TOL);
    EXPECT_NEAR(bodyX.y, 0.0, TOL);
    EXPECT_NEAR(bodyX.z, -1.0, TOL);
}

TEST_F(IntegratorTest, RotationPreservesVectorMagnitude) {
    SteeringAngles angles{0.1, 0.2, 0.3};
    Mat3 R = Dynamics::rmBodyToLocal(angles);

    Vec3 v{3.0, 4.0, 5.0};
    Vec3 rotated = R * v;
    EXPECT_NEAR(rotated.mag(), v.mag(), TOL);
}

TEST_F(IntegratorTest, InverseRotationIsTranspose) {
    SteeringAngles angles{0.4, -0.3, 0.6};
    Mat3 R = Dynamics::rmBodyToLocal(angles);

    Vec3 v{1.0, 2.0, 3.0};
    Vec3 rotated = R * v;
    Vec3 recovered = R.transposed() * rotated;
    EXPECT_TRUE(near_equal(v, recovered));
}

TEST_F(IntegratorTest, ComposedRotationMatchesSequential) {
    // Convention: gamma→X (roll), psi→Y (yaw), phi→Z (pitch)
    // SteeringAngles = {phi, psi, gamma}
    SteeringAngles angles{0.2, 0.4, 0.6};  // phi=0.2, psi=0.4, gamma=0.6
    Mat3 R = Dynamics::rmBodyToLocal(angles);
    // bodyToLocal = Rz(-phi) * Ry(-psi) * Rx(-gamma) = transpose of localToBody
    Mat3 expected = Mat3::rotation_z(-0.2) * Mat3::rotation_y(-0.4) * Mat3::rotation_x(-0.6);
    EXPECT_TRUE(near_equal(R, expected));
}

// =============================================================================
// Dynamics::evaluate Tests
// =============================================================================

TEST_F(IntegratorTest, StationaryStateHasDownwardAcceleration) {
    auto state = stationaryState();
    StateDerivative deriv = dynamics->evaluate(state, zeroSteering, 0.0, 0.0, 0.0);

    EXPECT_TRUE(near_equal(deriv.positionDot, Vec3::zero()));
    EXPECT_LT(deriv.velocityDot.y, -9.0);
    EXPECT_NEAR(deriv.velocityDot.x, 0.0, 0.1);
    EXPECT_NEAR(deriv.velocityDot.z, 0.0, 0.1);
}

TEST_F(IntegratorTest, EvaluateReturnsFiniteDerivatives) {
    auto state = coastingState();
    StateDerivative deriv = dynamics->evaluate(state, zeroSteering, 0.0, 0.0, 0.0);

    EXPECT_TRUE(deriv.positionDot.is_finite());
    EXPECT_TRUE(deriv.velocityDot.is_finite());
    EXPECT_TRUE(std::isfinite(deriv.massDot));
}

TEST_F(IntegratorTest, ZeroMassFlowGivesZeroMassDot) {
    auto state = coastingState();
    StateDerivative deriv = dynamics->evaluate(state, zeroSteering, 0.0, 0.0, 0.0);
    EXPECT_DOUBLE_EQ(deriv.massDot, 0.0);
}

TEST_F(IntegratorTest, NonZeroMassFlowGivesNegativeMassDot) {
    auto state = poweredState();
    StateDerivative deriv = dynamics->evaluate(state, zeroSteering,
                                                POWERED_THRUST, POWERED_MASS_FLOW, 0.0);
    EXPECT_LT(deriv.massDot, 0.0);
    EXPECT_DOUBLE_EQ(deriv.massDot, -200.0);
}

TEST_F(IntegratorTest, PositionDotEqualsVelocity) {
    auto state = coastingState();
    StateDerivative deriv = dynamics->evaluate(state, zeroSteering, 0.0, 0.0, 0.0);
    EXPECT_TRUE(near_equal(deriv.positionDot, state.velocity));
}

// =============================================================================
// RK4 Integration - Const Correctness
// =============================================================================

TEST_F(IntegratorTest, StepDoesNotModifyInputState) {
    auto state = coastingState();
    auto stateCopy = state;

    [[maybe_unused]] auto next = Integrator::stepRK4(*dynamics, state, zeroSteering,
                                                      0.0, 0.0, 0.0, 0.1);

    EXPECT_TRUE(near_equal(state.position, stateCopy.position));
    EXPECT_TRUE(near_equal(state.velocity, stateCopy.velocity));
    EXPECT_DOUBLE_EQ(state.vehicleMass, stateCopy.vehicleMass);
}

// =============================================================================
// RK4 Integration - Free Fall
// =============================================================================

TEST_F(IntegratorTest, FreeFallPositionAfterOneStep) {
    auto state = stationaryState();
    double dt = 0.01;

    VehicleState next = Integrator::stepRK4(*dynamics, state, zeroSteering,
                                             0.0, 0.0, 0.0, dt);

    // y should decrease by ~0.5*g*dt^2 ≈ 4.9e-4 m
    double dy = next.position.y - state.position.y;
    EXPECT_NEAR(dy, -4.9e-4, 1e-5);
    EXPECT_NEAR(next.position.x, 0.0, TOL);
    EXPECT_NEAR(next.position.z, 0.0, TOL);
}

TEST_F(IntegratorTest, FreeFallVelocityAfterOneStep) {
    auto state = stationaryState();
    double dt = 0.01;

    VehicleState next = Integrator::stepRK4(*dynamics, state, zeroSteering,
                                             0.0, 0.0, 0.0, dt);

    EXPECT_NEAR(next.velocity.y, -0.0981, 1e-3);
}

TEST_F(IntegratorTest, FreeFallMassConserved) {
    auto state = stationaryState();
    double dt = 1.0;

    VehicleState next = Integrator::stepRK4(*dynamics, state, zeroSteering,
                                             0.0, 0.0, 0.0, dt);

    EXPECT_DOUBLE_EQ(next.vehicleMass, state.vehicleMass);
}

TEST_F(IntegratorTest, FreeFallMultipleStepsAccumulate) {
    auto state = stationaryState();
    double dt = 0.01;
    int nSteps = 100;  // 1 second total

    VehicleState current = state;
    for (int i = 0; i < nSteps; ++i) {
        current = Integrator::stepRK4(*dynamics, current, zeroSteering,
                                       0.0, 0.0, i * dt, dt);
    }

    // After 1s: vy ≈ -9.81 m/s, Δy ≈ -4.905 m
    EXPECT_NEAR(current.velocity.y, -9.81, 0.05);
    double dy = current.position.y - state.position.y;
    EXPECT_NEAR(dy, -4.905, 0.05);
}

// =============================================================================
// RK4 Integration - Coasting (Ballistic Arc)
// =============================================================================

TEST_F(IntegratorTest, CoastingPositionAdvances) {
    auto state = coastingState();
    double dt = 0.1;

    VehicleState next = Integrator::stepRK4(*dynamics, state, zeroSteering,
                                             0.0, 0.0, 0.0, dt);

    EXPECT_GT(next.position.y, state.position.y);
}

TEST_F(IntegratorTest, CoastingVelocityDecreasesDueToGravity) {
    auto state = coastingState();
    double dt = 1.0;

    VehicleState next = Integrator::stepRK4(*dynamics, state, zeroSteering,
                                             0.0, 0.0, 0.0, dt);

    EXPECT_LT(next.velocity.y, state.velocity.y);
}

TEST_F(IntegratorTest, CoastingHorizontalVelocityUnchanged) {
    auto state = coastingState();
    double dt = 0.1;

    VehicleState next = Integrator::stepRK4(*dynamics, state, zeroSteering,
                                             0.0, 0.0, 0.0, dt);

    EXPECT_NEAR(next.velocity.x, 0.0, 0.1);
    EXPECT_NEAR(next.velocity.z, 0.0, 0.1);
}

// =============================================================================
// RK4 Integration - Powered Flight
// =============================================================================

TEST_F(IntegratorTest, ThrustIncreasesVelocity) {
    auto state = poweredState();
    state.velocity = Vec3{0.0, 0.0, 0.0};
    double dt = 0.1;

    VehicleState next = Integrator::stepRK4(*dynamics, state, zeroSteering,
                                             POWERED_THRUST, POWERED_MASS_FLOW, 0.0, dt);

    // Thrust = 600 kN along local x, mass = 50t → a_x ≈ 12 m/s²
    EXPECT_GT(next.velocity.x, 0.0);
}

TEST_F(IntegratorTest, MassDepletionDuringBurn) {
    auto state = poweredState();
    double dt = 1.0;

    VehicleState next = Integrator::stepRK4(*dynamics, state, zeroSteering,
                                             POWERED_THRUST, POWERED_MASS_FLOW, 0.0, dt);

    // Mass should decrease by ~massFlowRate * dt = 200 kg
    EXPECT_NEAR(next.vehicleMass, state.vehicleMass - 200.0, 1.0);
}

TEST_F(IntegratorTest, AccelerationFieldPopulated) {
    auto state = stationaryState();
    double dt = 0.01;

    VehicleState next = Integrator::stepRK4(*dynamics, state, zeroSteering,
                                             0.0, 0.0, 0.0, dt);

    // Free-fall: acceleration should be ~-9.81 in y
    EXPECT_NEAR(next.acceleration.y, -9.81, 0.1);
    EXPECT_NEAR(next.acceleration.x, 0.0, 0.1);
    EXPECT_NEAR(next.acceleration.z, 0.0, 0.1);
}

// =============================================================================
// RK4 Integration - Steering
// =============================================================================

TEST_F(IntegratorTest, PhiRotatedThrustChangesYVelocity) {
    // phi controls Z-axis rotation (pitch)
    // bodyToLocal = Rz(-φ). At φ=π/2: Rz(-π/2) maps body x → local +y
    // Thrust along body-X will have positive Y component in local frame
    auto state = poweredState();
    state.velocity = Vec3::zero();
    SteeringAngles angles{std::numbers::pi / 2.0, 0.0, 0.0};  // phi=π/2
    double dt = 0.1;

    VehicleState next = Integrator::stepRK4(*dynamics, state, angles,
                                             POWERED_THRUST, POWERED_MASS_FLOW, 0.0, dt);

    // Thrust should increase Y velocity (positive, net of gravity)
    EXPECT_GT(next.velocity.y, 0.0);
}

TEST_F(IntegratorTest, PsiRotatedThrustChangesZVelocity) {
    // psi controls Y-axis rotation (yaw)
    // bodyToLocal = Ry(-ψ). At ψ=π/2: Ry(-π/2) maps body x → local -z
    // Thrust along body-X goes to local -Z
    auto state = poweredState();
    state.velocity = Vec3::zero();
    SteeringAngles angles{0.0, std::numbers::pi / 2.0, 0.0};  // psi=π/2
    double dt = 0.1;

    VehicleState next = Integrator::stepRK4(*dynamics, state, angles,
                                             POWERED_THRUST, POWERED_MASS_FLOW, 0.0, dt);

    // Thrust should decrease Z velocity (negative)
    EXPECT_LT(next.velocity.z, 0.0);
}

TEST_F(IntegratorTest, DifferentSteeringProducesDifferentResults) {
    auto state = poweredState();
    state.velocity = Vec3::zero();
    double dt = 0.1;

    SteeringAngles s1{0.0, 0.0, 0.0};
    SteeringAngles s2{0.0, 0.5, 0.0};  // Changed psi instead of gamma for clearer effect

    VehicleState n1 = Integrator::stepRK4(*dynamics, state, s1,
                                           POWERED_THRUST, POWERED_MASS_FLOW, 0.0, dt);
    VehicleState n2 = Integrator::stepRK4(*dynamics, state, s2,
                                           POWERED_THRUST, POWERED_MASS_FLOW, 0.0, dt);

    EXPECT_FALSE(near_equal(n1.velocity, n2.velocity));
}

// =============================================================================
// RK4 Convergence Order
// =============================================================================

TEST_F(IntegratorTest, ConvergenceOrderIsFourth) {
    // Use aggressive mass depletion for a strongly nonlinear ODE:
    //   a(t) = F/(m0 - mdot*t) + g
    // Mass drops from 2000 to 1100 kg over 3s → 45% change → strong nonlinearity.
    VehicleState state{
        Vec3{0.0, START_ALT, 0.0},
        Vec3{0.0, 500.0, 0.0},
        Vec3::zero(),
        2000.0
    };
    double thrustMag = 900000.0;   // 300 * 3000
    double massFlow  = 300.0;
    double T = 3.0;

    // Reference: 30000 steps
    int nRef = 30000;
    double dtRef = T / nRef;
    VehicleState ref = state;
    for (int i = 0; i < nRef; ++i)
        ref = Integrator::stepRK4(*dynamics, ref, zeroSteering,
                                   thrustMag, massFlow, i * dtRef, dtRef);

    // Coarse: 6 steps (dt = 0.5)
    int n1 = 6;
    double dt1 = T / n1;
    VehicleState r1 = state;
    for (int i = 0; i < n1; ++i)
        r1 = Integrator::stepRK4(*dynamics, r1, zeroSteering,
                                  thrustMag, massFlow, i * dt1, dt1);

    // Finer: 12 steps (dt = 0.25)
    int n2 = 12;
    double dt2 = T / n2;
    VehicleState r2 = state;
    for (int i = 0; i < n2; ++i)
        r2 = Integrator::stepRK4(*dynamics, r2, zeroSteering,
                                  thrustMag, massFlow, i * dt2, dt2);

    double err1 = (r1.position - ref.position).mag();
    double err2 = (r2.position - ref.position).mag();

    // err1 / err2 ≈ 2^4 = 16 for 4th-order method
    double ratio = err1 / err2;
    EXPECT_GT(ratio, 10.0);
    EXPECT_LT(ratio, 22.0);
}

// =============================================================================
// RK4 Integration - Edge Cases
// =============================================================================

TEST_F(IntegratorTest, ZeroTimestepReturnsOriginalState) {
    auto state = coastingState();
    VehicleState next = Integrator::stepRK4(*dynamics, state, zeroSteering,
                                             0.0, 0.0, 0.0, 0.0);

    EXPECT_TRUE(near_equal(next.position, state.position));
    EXPECT_TRUE(near_equal(next.velocity, state.velocity));
    EXPECT_DOUBLE_EQ(next.vehicleMass, state.vehicleMass);
}

TEST_F(IntegratorTest, VerySmallTimestep) {
    auto state = coastingState();
    double dt = 1e-10;

    VehicleState next = Integrator::stepRK4(*dynamics, state, zeroSteering,
                                             0.0, 0.0, 0.0, dt);

    EXPECT_TRUE(next.position.is_finite());
    EXPECT_TRUE(next.velocity.is_finite());
    EXPECT_TRUE(std::isfinite(next.vehicleMass));
}

TEST_F(IntegratorTest, LargeTimestepStillFinite) {
    // Start with upward velocity so altitude stays positive during dt=10s
    auto state = coastingState();
    double dt = 10.0;

    VehicleState next = Integrator::stepRK4(*dynamics, state, zeroSteering,
                                             0.0, 0.0, 0.0, dt);

    EXPECT_TRUE(next.position.is_finite());
    EXPECT_TRUE(next.velocity.is_finite());
}

TEST_F(IntegratorTest, NonZeroStartTime) {
    auto state = coastingState();
    double dt = 0.1;

    VehicleState n1 = Integrator::stepRK4(*dynamics, state, zeroSteering,
                                           0.0, 0.0, 0.0, dt);
    VehicleState n2 = Integrator::stepRK4(*dynamics, state, zeroSteering,
                                           0.0, 0.0, 100.0, dt);

    EXPECT_TRUE(near_equal(n1.position, n2.position));
    EXPECT_TRUE(near_equal(n1.velocity, n2.velocity));
}

// =============================================================================
// RK4 Integration - Multi-Step Consistency
// =============================================================================

TEST_F(IntegratorTest, TwoHalfStepsMatchOneFullStep) {
    auto state = coastingState();
    double dt = 0.1;

    VehicleState oneFull = Integrator::stepRK4(*dynamics, state, zeroSteering,
                                                0.0, 0.0, 0.0, dt);

    VehicleState half1 = Integrator::stepRK4(*dynamics, state, zeroSteering,
                                              0.0, 0.0, 0.0, dt / 2.0);
    VehicleState twoHalf = Integrator::stepRK4(*dynamics, half1, zeroSteering,
                                                0.0, 0.0, dt / 2.0, dt / 2.0);

    EXPECT_NEAR(oneFull.position.y, twoHalf.position.y, 1e-6);
    EXPECT_NEAR(oneFull.velocity.y, twoHalf.velocity.y, 1e-5);
}

TEST_F(IntegratorTest, LongIntegrationRemainsPhysical) {
    // 10 seconds of coasting upward from 10 km — altitude stays positive
    auto state = coastingState();
    double dt = 0.01;
    int nSteps = 1000;

    VehicleState current = state;
    for (int i = 0; i < nSteps; ++i) {
        current = Integrator::stepRK4(*dynamics, current, zeroSteering,
                                       0.0, 0.0, i * dt, dt);
        ASSERT_TRUE(current.position.is_finite()) << "step=" << i;
        ASSERT_TRUE(current.velocity.is_finite()) << "step=" << i;
    }

    // After 10s: Δvy ≈ -98.1 m/s, Δy ≈ 1000*10 - 490.5 ≈ 9509 m
    EXPECT_NEAR(current.velocity.y, 1000.0 - 98.1, 1.0);
    double dy = current.position.y - state.position.y;
    EXPECT_NEAR(dy, 9510.0, 10.0);
}

// =============================================================================
// RK4 Integration - Symmetry
// =============================================================================

TEST_F(IntegratorTest, SymmetricSteeringProducesSymmetricResult) {
    // Gamma (Y-axis rotation): ±gamma gives symmetric z-velocity (sin is odd)
    // and equal x-velocity (cos is even). y-velocity same (gravity only).
    auto state = poweredState();
    state.velocity = Vec3::zero();
    double dt = 0.01;

    SteeringAngles pitchUp{0.0, 0.0, 0.3};
    SteeringAngles pitchDown{0.0, 0.0, -0.3};

    VehicleState nU = Integrator::stepRK4(*dynamics, state, pitchUp,
                                           POWERED_THRUST, POWERED_MASS_FLOW, 0.0, dt);
    VehicleState nD = Integrator::stepRK4(*dynamics, state, pitchDown,
                                           POWERED_THRUST, POWERED_MASS_FLOW, 0.0, dt);

    EXPECT_NEAR(nU.velocity.x, nD.velocity.x, LOOSE_TOL);
    EXPECT_NEAR(nU.velocity.y, nD.velocity.y, LOOSE_TOL);
    EXPECT_NEAR(nU.velocity.z, -nD.velocity.z, LOOSE_TOL);
}

}  // namespace trajsim::test
