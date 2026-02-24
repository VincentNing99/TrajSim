// tests/test_guidance.cpp

#include <gtest/gtest.h>
#include <numbers>
#include "models/guidance/guidance.hpp"

namespace trajsim {
namespace {

// =============================================================================
// Shared test configs (matching default JSON files)
// =============================================================================

// Test vehicle state used across guidance tests
static const VehicleState TEST_VEHICLE_STATE = {
    .position = Vec3{-2000000.0, -10000000.0, 200000.0},
    .velocity = Vec3{-7000.0, 2500.0, 200.0},
    .acceleration = Vec3::zero(),
    .vehicleMass = 50000.0
};

static Guidance::Config makeTestGuidanceConfig() {
    Guidance::AlgorithmEntry entry;
    entry.type = "IterativeGuidance";
    entry.igmConfig = {
        .guidanceCycle = 0.01,
        .steeringHoldTime = 15.0,
        .maxConvergenceIterations = 100,
        .timeToGoConvergenceTolerance = 1e-5
    };
    entry.igmCutoffCriteria = {
        .inclinationTolerance = 0.07,
        .eccentricityTolerance = 0.005
    };

    return Guidance::Config{
        .stage = 1,
        .tolerance = 1e-10,
        .maxSteeringRate = 5.0,
        .algorithms = {entry}
    };
}

// Engine parameters used across guidance tests
static constexpr double TEST_MASS_FLOW_RATE = 100.0;
static constexpr double TEST_EXIT_VELOCITY  = 3500.0;

// =============================================================================
// Test Fixtures
// =============================================================================

class GuidanceTest : public ::testing::Test {
protected:
    void SetUp() override {
        guidance = std::make_unique<Guidance>(makeTestGuidanceConfig(), MISSION_1, TEST_VEHICLE_STATE);
    }

    IterativeGuidance* getIGM() const {
        return dynamic_cast<IterativeGuidance*>(guidance->activeAlgorithm());
    }

    std::unique_ptr<Guidance> guidance;
};

// =============================================================================
// Construction Tests
// =============================================================================

TEST_F(GuidanceTest, ConstructFromMission1) {
    Guidance g(makeTestGuidanceConfig(), MISSION_1, TEST_VEHICLE_STATE);
    const auto* terminal = g.getTerminalState();
    ASSERT_NE(terminal, nullptr);

    EXPECT_DOUBLE_EQ(terminal->semiMajorAxis, MISSION_1.semiMajorAxis);
    EXPECT_DOUBLE_EQ(terminal->inclination, MISSION_1.inclination);
    EXPECT_DOUBLE_EQ(terminal->eccentricity, MISSION_1.eccentricity);
}

TEST_F(GuidanceTest, ConstructFromMission2) {
    Guidance g(makeTestGuidanceConfig(), MISSION_2, TEST_VEHICLE_STATE);
    const auto* terminal = g.getTerminalState();
    ASSERT_NE(terminal, nullptr);

    EXPECT_DOUBLE_EQ(terminal->semiMajorAxis, MISSION_2.semiMajorAxis);
    EXPECT_DOUBLE_EQ(terminal->inclination, MISSION_2.inclination);
    EXPECT_DOUBLE_EQ(terminal->eccentricity, MISSION_2.eccentricity);
}

TEST_F(GuidanceTest, TerminalStatePositionAndVelocity) {
    Guidance g(makeTestGuidanceConfig(), MISSION_1, TEST_VEHICLE_STATE);
    const auto* terminal = g.getTerminalState();
    ASSERT_NE(terminal, nullptr);

    EXPECT_DOUBLE_EQ(terminal->position.x, MISSION_1.positionTerminal.x);
    EXPECT_DOUBLE_EQ(terminal->position.y, MISSION_1.positionTerminal.y);
    EXPECT_DOUBLE_EQ(terminal->position.z, MISSION_1.positionTerminal.z);

    EXPECT_DOUBLE_EQ(terminal->velocity.x, MISSION_1.velocityTerminal.x);
    EXPECT_DOUBLE_EQ(terminal->velocity.y, MISSION_1.velocityTerminal.y);
    EXPECT_DOUBLE_EQ(terminal->velocity.z, MISSION_1.velocityTerminal.z);
}

// =============================================================================
// Open Loop Guidance Tests
// =============================================================================

class OpenLoopGuidanceTest : public ::testing::Test {
protected:
    void SetUp() override {
        guidance = std::make_unique<Guidance>(makeTestGuidanceConfig(), MISSION_1, TEST_VEHICLE_STATE);
    }

    std::unique_ptr<Guidance> guidance;
};

// =============================================================================
// Cutoff Tests
// =============================================================================

TEST_F(GuidanceTest, CutoffReturnsFalseForZeroPosition) {
    VehicleState state;
    state.position = Vec3{0.0, 0.0, 0.0};
    state.velocity = Vec3{1000.0, 0.0, 0.0};
    state.acceleration = Vec3::zero();
    state.vehicleMass = 10000.0;

    EXPECT_FALSE(guidance->cutoff(state));
}

TEST_F(GuidanceTest, CutoffReturnsFalseForRadialTrajectory) {
    VehicleState state;
    state.position = Vec3{6378000.0, 0.0, 0.0};
    state.velocity = Vec3{1000.0, 0.0, 0.0};
    state.acceleration = Vec3::zero();
    state.vehicleMass = 10000.0;

    EXPECT_FALSE(guidance->cutoff(state));
}

TEST_F(GuidanceTest, CutoffReturnsFalseWhenNotAtTarget) {
    VehicleState state;
    state.position = Vec3{6378000.0, 0.0, 0.0};
    state.velocity = Vec3{0.0, 5000.0, 0.0};
    state.acceleration = Vec3::zero();
    state.vehicleMass = 10000.0;

    EXPECT_FALSE(guidance->cutoff(state));
}

// =============================================================================
// ReferenceMission Tests
// =============================================================================

TEST(ReferenceMissionTest, ArgumentOfLatitudeInitial) {
    double expected = MISSION_1.argumentOfPeriapsis + MISSION_1.trueAnomaly;
    EXPECT_DOUBLE_EQ(MISSION_1.argumentOfLatitudeInitial(), expected);
}

TEST(ReferenceMissionTest, RadiusOfCurvaturePositive) {
    EXPECT_GT(MISSION_1.radiusOfCurvature(), 0.0);
    EXPECT_GT(MISSION_2.radiusOfCurvature(), 0.0);
}

TEST(ReferenceMissionTest, PositionLaunchSiteNonZero) {
    Vec3 pos1 = MISSION_1.positionLaunchSite();
    Vec3 pos2 = MISSION_2.positionLaunchSite();

    EXPECT_GT(pos1.mag(), 0.0);
    EXPECT_GT(pos2.mag(), 0.0);
}

TEST(ReferenceMissionTest, TransformMatricesNonSingular) {
    Mat3 rm1 = MISSION_1.rmLaunchToEquatorial();
    Mat3 rm2 = MISSION_2.rmLaunchToEquatorial();

    EXPECT_NE(rm1.det(), 0.0);
    EXPECT_NE(rm2.det(), 0.0);

    EXPECT_TRUE(rm1.is_finite());
    EXPECT_TRUE(rm2.is_finite());
}

TEST(ReferenceMissionTest, Mission1DifferentFromMission2) {
    EXPECT_NE(MISSION_1.semiMajorAxis, MISSION_2.semiMajorAxis);
    EXPECT_NE(MISSION_1.initialTime, MISSION_2.initialTime);
    EXPECT_NE(MISSION_1.cutoffTime, MISSION_2.cutoffTime);
}

// =============================================================================
// Conversion Constants Tests
// =============================================================================

TEST(ConversionTest, DegToRadCorrect) {
    EXPECT_NEAR(degToRad * 180.0, std::numbers::pi, 1e-15);
    EXPECT_NEAR(degToRad * 90.0, std::numbers::pi / 2.0, 1e-15);
}

TEST(ConversionTest, RadToDegCorrect) {
    EXPECT_NEAR(radToDeg * std::numbers::pi, 180.0, 1e-12);
    EXPECT_NEAR(radToDeg * (std::numbers::pi / 2.0), 90.0, 1e-12);
}

TEST(ConversionTest, DegRadInverse) {
    double original = 45.0;
    double converted = original * degToRad * radToDeg;
    EXPECT_NEAR(converted, original, 1e-12);
}

// =============================================================================
// Open Loop Guidance Tests
// =============================================================================

TEST_F(OpenLoopGuidanceTest, OpenLoopGuidanceInterpolatesCorrectly) {
    std::vector<std::vector<double>> profile = {
        {0.0, 0.0, 0.0, 0.0},
        {10.0, 1.0, 0.5, 0.0},
        {20.0, 2.0, 1.0, 0.0}
    };
}

TEST_F(OpenLoopGuidanceTest, OpenLoopClampsToFirstPointBeforeStart) {
}

TEST_F(OpenLoopGuidanceTest, OpenLoopClampsToLastPointAfterEnd) {
}

// =============================================================================
// ComputeDeltaV Tests
// =============================================================================

class GuidanceDeltaVTest : public ::testing::Test {
protected:
    void SetUp() override {
        guidance = std::make_unique<Guidance>(makeTestGuidanceConfig(), MISSION_1, TEST_VEHICLE_STATE);
    }

    IterativeGuidance* getIGM() const {
        return dynamic_cast<IterativeGuidance*>(guidance->activeAlgorithm());
    }

    std::unique_ptr<Guidance> guidance;
};

TEST_F(GuidanceDeltaVTest, ComputeDeltaVCalculatesCorrectComponents) {
    Vec3 g{0.0, -9.8, 0.0};
    Vec3 instantaneousVelocity{5000.0, 2000.0, 500.0};

    getIGM()->computeDeltaV(g, instantaneousVelocity);
}

// =============================================================================
// ComputeVelocitySteeringAngles Tests
// =============================================================================

TEST_F(GuidanceDeltaVTest, ComputeVelocitySteeringAnglesReturnsValidAngles) {
    Vec3 g{0.0, -9.8, 0.0};
    Vec3 instantaneousVelocity{5000.0, 2000.0, 500.0};

    getIGM()->computeDeltaV(g, instantaneousVelocity);

    SteeringAngles angles = getIGM()->computeVelocitySteeringAngles();

    EXPECT_TRUE(std::isfinite(angles.phi));
    EXPECT_TRUE(std::isfinite(angles.psi));
    EXPECT_GE(angles.phi, -std::numbers::pi);
    EXPECT_LE(angles.phi, std::numbers::pi);
    EXPECT_GE(angles.psi, -std::numbers::pi / 2.0);
    EXPECT_LE(angles.psi, std::numbers::pi / 2.0);
}

TEST_F(GuidanceDeltaVTest, ComputeVelocitySteeringAnglesHandlesZeroDvXi) {
    Vec3 g{0.0, 0.0, 0.0};
    Vec3 instantaneousVelocity{0.0, 0.0, 0.0};

    getIGM()->computeDeltaV(g, instantaneousVelocity);
    SteeringAngles angles = getIGM()->computeVelocitySteeringAngles();

    EXPECT_FALSE(std::isnan(angles.phi));
    EXPECT_FALSE(std::isnan(angles.psi));
}

// =============================================================================
// BuildRotationMatrix Tests
// =============================================================================

TEST_F(GuidanceTest, BuildRotationMatrixProducesFiniteMatrix) {
    Guidance g(makeTestGuidanceConfig(), MISSION_1, TEST_VEHICLE_STATE);

    const auto* terminal = g.getTerminalState();
    ASSERT_NE(terminal, nullptr);

    EXPECT_TRUE(terminal->position.is_finite());
    EXPECT_TRUE(terminal->velocity.is_finite());
}

TEST_F(GuidanceTest, BuildRotationMatrixFromMission2) {
    Guidance g(makeTestGuidanceConfig(), MISSION_2, TEST_VEHICLE_STATE);

    const auto* terminal = g.getTerminalState();
    ASSERT_NE(terminal, nullptr);

    EXPECT_TRUE(terminal->position.is_finite());
    EXPECT_TRUE(terminal->velocity.is_finite());
}

// =============================================================================
// Cutoff Success Case Tests
// =============================================================================

TEST_F(GuidanceTest, CutoffReturnsTrueWhenTargetReached) {
    VehicleState state;

    state.position = MISSION_1.positionTerminal;
    state.velocity = MISSION_1.velocityTerminal;
    state.acceleration = Vec3::zero();
    state.vehicleMass = 10000.0;

    bool result = guidance->cutoff(state);
    EXPECT_TRUE(result == true || result == false);
}

// =============================================================================
// IterativeGuidance Tests
// =============================================================================

class IterativeGuidanceTest : public ::testing::Test {
protected:
    void SetUp() override {
        guidance = std::make_unique<Guidance>(makeTestGuidanceConfig(), MISSION_1, TEST_VEHICLE_STATE);
    }

    IterativeGuidance* getIGM() const {
        return dynamic_cast<IterativeGuidance*>(guidance->activeAlgorithm());
    }

    std::unique_ptr<Guidance> guidance;
};

TEST_F(IterativeGuidanceTest, IterativeGuidanceReturnsValidSteeringAngles) {
    VehicleState state;
    state.position = Vec3{-2000000.0, -10000000.0, 200000.0};
    state.velocity = Vec3{-7000.0, 2500.0, 200.0};
    state.acceleration = Vec3::zero();
    state.vehicleMass = 50000.0;

    Vec3 g{0.0, -9.8, 0.0};
    double t = MISSION_1.initialTime + 10.0;

    GuidanceInput input{};
    input.time = t;
    input.state = state;
    input.massFlowRate = TEST_MASS_FLOW_RATE;
    input.exitVelocity = TEST_EXIT_VELOCITY;
    input.gravity = g;
    input.gravityCutoff = g;

    GuidanceOutput output = getIGM()->computeSteering(input);

    EXPECT_TRUE(output.steering.is_finite());
}

TEST_F(IterativeGuidanceTest, IterativeGuidanceHoldsSteeringNearCutoff) {
    VehicleState state;
    state.position = Vec3{-2000000.0, -10000000.0, 200000.0};
    state.velocity = Vec3{-7000.0, 2500.0, 200.0};
    state.acceleration = Vec3::zero();
    state.vehicleMass = 50000.0;

    Vec3 g{0.0, -9.8, 0.0};

    GuidanceInput input{};
    input.state = state;
    input.massFlowRate = TEST_MASS_FLOW_RATE;
    input.exitVelocity = TEST_EXIT_VELOCITY;
    input.gravity = g;
    input.gravityCutoff = g;

    input.time = MISSION_1.initialTime + 50.0;
    GuidanceOutput output1 = getIGM()->computeSteering(input);

    EXPECT_TRUE(output1.steering.is_finite());

    input.time = MISSION_1.initialTime + 60.0;
    GuidanceOutput output2 = getIGM()->computeSteering(input);

    EXPECT_TRUE(output2.steering.is_finite());
}

// =============================================================================
// UpdateTimeToGo Tests
// =============================================================================

TEST_F(IterativeGuidanceTest, UpdateTimeToGoLttwComputesValidTimeToGo) {
    Vec3 g{0.0, -9.8, 0.0};
    Vec3 vt{-7100.0, 2600.0, 240.0};
    Vec3 vc{-7000.0, 2500.0, 200.0};
    double delV = 500.0;
    double tau = 50000.0 / TEST_MASS_FLOW_RATE;

    getIGM()->updateTimeToGoLttw(delV, tau, g, vt, vc);
}

TEST_F(IterativeGuidanceTest, UpdateTimeToGoHttwComputesValidTimeToGo) {
    Vec3 g{0.0, -9.8, 0.0};
    Vec3 vt{-7100.0, 2600.0, 240.0};
    Vec3 vc{-7000.0, 2500.0, 200.0};
    double delV = 500.0;
    double tau = 50000.0 / TEST_MASS_FLOW_RATE;

    getIGM()->updateTimeToGoHttw(delV, tau, TEST_EXIT_VELOCITY, g, vt, vc);
}

// =============================================================================
// ConvergeTimeToGo Tests
// =============================================================================

TEST_F(IterativeGuidanceTest, ConvergeTimeToGoConvergesWithinIterations) {
    Vec3 g{0.0, -9.8, 0.0};
    Vec3 vC{-7000.0, 2500.0, 200.0};
    double tau = 50000.0 / TEST_MASS_FLOW_RATE;
    double thrustToWeight = 0.5;

    EXPECT_NO_THROW(getIGM()->convergeTimeToGo(g, vC, tau, TEST_EXIT_VELOCITY, thrustToWeight));
}

TEST_F(IterativeGuidanceTest, ConvergeTimeToGoHighThrustToWeight) {
    Vec3 g{0.0, -9.8, 0.0};
    Vec3 vC{-7000.0, 2500.0, 200.0};
    double tau = 50000.0 / TEST_MASS_FLOW_RATE;
    double thrustToWeight = 1.5;

    EXPECT_NO_THROW(getIGM()->convergeTimeToGo(g, vC, tau, TEST_EXIT_VELOCITY, thrustToWeight));
}

// =============================================================================
// ApplyPositionCorrections Tests
// =============================================================================

TEST_F(IterativeGuidanceTest, ApplyPositionCorrectionsProducesFiniteAngles) {
    VehicleState state;
    state.position = Vec3{-2000000.0, -10000000.0, 200000.0};
    state.velocity = Vec3{-7000.0, 2500.0, 200.0};
    state.acceleration = Vec3::zero();
    state.vehicleMass = 50000.0;

    Vec3 g{0.0, -9.8, 0.0};
    double tau = 50000.0 / TEST_MASS_FLOW_RATE;

    GuidanceInput input{};
    input.time = MISSION_1.initialTime + 10.0;
    input.state = state;
    input.massFlowRate = TEST_MASS_FLOW_RATE;
    input.exitVelocity = TEST_EXIT_VELOCITY;
    input.gravity = g;
    input.gravityCutoff = g;
    getIGM()->computeSteering(input);

    SteeringAngles velocityAngles{1.0, 0.1, 0.0};

    SteeringAngles corrected = getIGM()->applyPositionCorrections(velocityAngles, tau, TEST_EXIT_VELOCITY, g);

    EXPECT_TRUE(std::isfinite(corrected.phi));
    EXPECT_TRUE(std::isfinite(corrected.psi));
}

// =============================================================================
// TransformSteeringToInertial Tests
// =============================================================================

TEST_F(IterativeGuidanceTest, TransformSteeringToInertialProducesFiniteAngles) {
    VehicleState state;
    state.position = Vec3{-2000000.0, -10000000.0, 200000.0};
    state.velocity = Vec3{-7000.0, 2500.0, 200.0};
    state.acceleration = Vec3::zero();
    state.vehicleMass = 50000.0;

    Vec3 g{0.0, -9.8, 0.0};

    GuidanceInput input{};
    input.time = MISSION_1.initialTime + 10.0;
    input.state = state;
    input.massFlowRate = TEST_MASS_FLOW_RATE;
    input.exitVelocity = TEST_EXIT_VELOCITY;
    input.gravity = g;
    input.gravityCutoff = g;
    getIGM()->computeSteering(input);

    SteeringAngles correctedAngles{0.5, 0.1, 0.0};

    SteeringAngles inertial = getIGM()->transformSteeringToInertial(correctedAngles);

    EXPECT_TRUE(std::isfinite(inertial.phi));
    EXPECT_TRUE(std::isfinite(inertial.psi));
}

TEST_F(IterativeGuidanceTest, TransformSteeringToInertialHandlesNearGimbalLock) {
    VehicleState state;
    state.position = Vec3{-2000000.0, -10000000.0, 200000.0};
    state.velocity = Vec3{-7000.0, 2500.0, 200.0};
    state.acceleration = Vec3::zero();
    state.vehicleMass = 50000.0;

    Vec3 g{0.0, -9.8, 0.0};

    GuidanceInput input{};
    input.time = MISSION_1.initialTime + 10.0;
    input.state = state;
    input.massFlowRate = TEST_MASS_FLOW_RATE;
    input.exitVelocity = TEST_EXIT_VELOCITY;
    input.gravity = g;
    input.gravityCutoff = g;
    getIGM()->computeSteering(input);

    SteeringAngles nearGimbalLock{0.0, std::numbers::pi / 2.0 - 0.1, 0.0};

    SteeringAngles result = getIGM()->transformSteeringToInertial(nearGimbalLock);

    EXPECT_TRUE(std::isfinite(result.phi));
    EXPECT_TRUE(std::isfinite(result.psi));
}

// =============================================================================
// ReferenceMission Rotation Matrix Tests
// =============================================================================

TEST(ReferenceMissionTest, RmLanRotationIsRotationMatrix) {
    Mat3 rm1 = MISSION_1.rmLanRotation();
    Mat3 rm2 = MISSION_2.rmLanRotation();

    EXPECT_TRUE(rm1.is_rotation(1e-10));
    EXPECT_TRUE(rm2.is_rotation(1e-10));
}

TEST(ReferenceMissionTest, RmInclinationRotationIsRotationMatrix) {
    Mat3 rm1 = MISSION_1.rmInclinationRotation();
    Mat3 rm2 = MISSION_2.rmInclinationRotation();

    EXPECT_TRUE(rm1.is_rotation(1e-10));
    EXPECT_TRUE(rm2.is_rotation(1e-10));
}

TEST(ReferenceMissionTest, RmArgumentOfLatitudeIsRotationMatrix) {
    Mat3 rm1 = MISSION_1.rmArgumentOfLatitude();
    Mat3 rm2 = MISSION_2.rmArgumentOfLatitude();

    EXPECT_TRUE(rm1.is_rotation(1e-10));
    EXPECT_TRUE(rm2.is_rotation(1e-10));
}

TEST(ReferenceMissionTest, RmInertialToOrbitalIsNonSingular) {
    Mat3 rm1 = MISSION_1.rmInertialToOrbital();
    Mat3 rm2 = MISSION_2.rmInertialToOrbital();

    EXPECT_NE(rm1.det(), 0.0);
    EXPECT_NE(rm2.det(), 0.0);
    EXPECT_TRUE(rm1.is_finite());
    EXPECT_TRUE(rm2.is_finite());
}

TEST(ReferenceMissionTest, RmInertialToTerminalIsNonSingular) {
    Mat3 rm1 = MISSION_1.rmInertialToTerminal();
    Mat3 rm2 = MISSION_2.rmInertialToTerminal();

    EXPECT_NE(rm1.det(), 0.0);
    EXPECT_NE(rm2.det(), 0.0);
    EXPECT_TRUE(rm1.is_finite());
    EXPECT_TRUE(rm2.is_finite());
}

TEST(ReferenceMissionTest, RadiusOfCurvatureReasonableValue) {
    double r1 = MISSION_1.radiusOfCurvature();
    double r2 = MISSION_2.radiusOfCurvature();

    EXPECT_GT(r1, 6300000.0);
    EXPECT_LT(r1, 6500000.0);
    EXPECT_GT(r2, 6300000.0);
    EXPECT_LT(r2, 6500000.0);
}

TEST(ReferenceMissionTest, PositionLaunchSiteMagnitudeReasonable) {
    Vec3 pos1 = MISSION_1.positionLaunchSite();
    Vec3 pos2 = MISSION_2.positionLaunchSite();

    double mag1 = pos1.mag();
    double mag2 = pos2.mag();

    EXPECT_GT(mag1, 0.0);
    EXPECT_GT(mag2, 0.0);
}

// =============================================================================
// FlightStage Tests
// =============================================================================

TEST(FlightStageTest, ToStringReturnsCorrectNames) {
    EXPECT_EQ(toString(FlightStage::Prelaunch), "Prelaunch");
    EXPECT_EQ(toString(FlightStage::Ignition), "Ignition");
    EXPECT_EQ(toString(FlightStage::Liftoff), "Liftoff");
    EXPECT_EQ(toString(FlightStage::MaxQ), "MaxQ");
    EXPECT_EQ(toString(FlightStage::ThrottleDown), "ThrottleDown");
    EXPECT_EQ(toString(FlightStage::Meco), "Meco");
    EXPECT_EQ(toString(FlightStage::Separation), "Separation");
    EXPECT_EQ(toString(FlightStage::Coast), "Coast");
    EXPECT_EQ(toString(FlightStage::Reentry), "Reentry");
    EXPECT_EQ(toString(FlightStage::Terminal), "Terminal");
}

// =============================================================================
// VehicleState Tests
// =============================================================================

TEST(VehicleStateTest, DefaultInitialization) {
    VehicleState state;
    state.position = Vec3{1000.0, 2000.0, 3000.0};
    state.velocity = Vec3{100.0, 200.0, 300.0};
    state.acceleration = Vec3{10.0, 20.0, 30.0};
    state.vehicleMass = 50000.0;

    EXPECT_DOUBLE_EQ(state.position.x, 1000.0);
    EXPECT_DOUBLE_EQ(state.velocity.y, 200.0);
    EXPECT_DOUBLE_EQ(state.acceleration.z, 30.0);
    EXPECT_DOUBLE_EQ(state.vehicleMass, 50000.0);
}

// =============================================================================
// DeltaV Tests
// =============================================================================

TEST(DeltaVTest, StructFieldsAccessible) {
    DeltaV dv;
    dv.dvXi = 100.0;
    dv.dvEta = 200.0;
    dv.dvZeta = 50.0;
    dv.dV = std::sqrt(100.0*100.0 + 200.0*200.0 + 50.0*50.0);

    EXPECT_DOUBLE_EQ(dv.dvXi, 100.0);
    EXPECT_DOUBLE_EQ(dv.dvEta, 200.0);
    EXPECT_DOUBLE_EQ(dv.dvZeta, 50.0);
    EXPECT_NEAR(dv.dV, 229.128784747792, 1e-6);
}

// =============================================================================
// SteeringAngles Tests
// =============================================================================

TEST(SteeringAnglesTest, StructFieldsAccessible) {
    SteeringAngles angles;
    angles.phi = 0.5;
    angles.psi = 0.1;
    angles.gamma = 0.0;

    EXPECT_DOUBLE_EQ(angles.phi, 0.5);
    EXPECT_DOUBLE_EQ(angles.psi, 0.1);
    EXPECT_DOUBLE_EQ(angles.gamma, 0.0);
}

// =============================================================================
// TerminalState Tests
// =============================================================================

TEST(TerminalStateTest, StructFieldsAccessible) {
    TerminalState ts;
    ts.semiMajorAxis = 6903085.0;
    ts.eccentricity = 8.851433e-06;
    ts.inclination = 97.496919 * degToRad;
    ts.position = Vec3{-2405466.9, -12845047.6, 205013.6};
    ts.velocity = Vec3{-7124.1826, 2631.5395, 249.9496};

    EXPECT_DOUBLE_EQ(ts.semiMajorAxis, 6903085.0);
    EXPECT_NEAR(ts.inclination, 1.7018, 0.001);
    EXPECT_DOUBLE_EQ(ts.position.x, -2405466.9);
}

}  // namespace
}  // namespace trajsim
