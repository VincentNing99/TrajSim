// tests/liquid_engine_test.cpp

#include <gtest/gtest.h>
#include "models/vehicle/engine_models/liquid_engine.hpp"

namespace trajsim {
namespace {

// =============================================================================
// Test Fixtures
// =============================================================================

class LiquidEngineTest : public ::testing::Test {
protected:
    /// @brief Create a valid baseline config for testing
    static LiquidEngine::Config makeValidConfig() {
        return LiquidEngine::Config{
            .id = "TestEngine",
            .thrust = 100000.0,              // 100 kN
            .isp = 300.0,                    // 300 s
            .mdot = 34.0,                    // kg/s
            .dryMass = 500.0,               // 500 kg
            .nozzleExitArea = 0.5,         // 0.5 m²
            .nozzleExitPressure = 50000.0, // 50 kPa
            .mountOffset = Vec3{0.0, 0.0, 0.0},
            .gimbalLimit = 0.1,             // ~5.7 deg
            .throttleMin = 0.4              // 40% min throttle
        };
    }
};

// =============================================================================
// Construction Tests
// =============================================================================

TEST_F(LiquidEngineTest, fromConfigValid) {
    auto cfg = makeValidConfig();
    auto engine = LiquidEngine::fromConfig(cfg);

    ASSERT_NE(engine, nullptr);
    EXPECT_EQ(engine->getId(), "TestEngine");
    EXPECT_DOUBLE_EQ(engine->getThrust(), 100000.0);
    EXPECT_DOUBLE_EQ(engine->getIsp(), 300.0);
    EXPECT_DOUBLE_EQ(engine->getDryMass(), 500.0);
}

TEST_F(LiquidEngineTest, fromConfigEmptyIdFails) {
    auto cfg = makeValidConfig();
    cfg.id = "";

    EXPECT_DEBUG_DEATH(LiquidEngine::fromConfig(cfg), "Engine ID cannot be empty");
}

TEST_F(LiquidEngineTest, fromConfigZeroThrustFails) {
    auto cfg = makeValidConfig();
    cfg.thrust = 0.0;

    EXPECT_DEBUG_DEATH(LiquidEngine::fromConfig(cfg), "Thrust must be positive");
}

TEST_F(LiquidEngineTest, fromConfigNegativeThrustFails) {
    auto cfg = makeValidConfig();
    cfg.thrust = -1000.0;

    EXPECT_DEBUG_DEATH(LiquidEngine::fromConfig(cfg), "Thrust must be positive");
}

TEST_F(LiquidEngineTest, fromConfigZeroIspFails) {
    auto cfg = makeValidConfig();
    cfg.isp = 0.0;

    EXPECT_DEBUG_DEATH(LiquidEngine::fromConfig(cfg), "Isp must be positive");
}

TEST_F(LiquidEngineTest, fromConfigInvalidThrottleMinFails) {
    auto cfg = makeValidConfig();
    cfg.throttleMin = 1.5;

    EXPECT_DEBUG_DEATH(LiquidEngine::fromConfig(cfg), "Throttle min must be in");
}

// =============================================================================
// Capability Tests
// =============================================================================

TEST_F(LiquidEngineTest, isThrottleableTrueWhenMinBelowOne) {
    auto cfg = makeValidConfig();
    cfg.throttleMin = 0.4;
    auto engine = LiquidEngine::fromConfig(cfg);

    EXPECT_TRUE(engine->isThrottleable());
}

TEST_F(LiquidEngineTest, isThrottleableFalseWhenMinIsOne) {
    auto cfg = makeValidConfig();
    cfg.throttleMin = 1.0;
    auto engine = LiquidEngine::fromConfig(cfg);

    EXPECT_FALSE(engine->isThrottleable());
}

TEST_F(LiquidEngineTest, isGimbaledTrueWhenLimitPositive) {
    auto cfg = makeValidConfig();
    cfg.gimbalLimit = 0.1;
    auto engine = LiquidEngine::fromConfig(cfg);

    EXPECT_TRUE(engine->isGimbaled());
}

TEST_F(LiquidEngineTest, isGimbaledFalseWhenLimitZero) {
    auto cfg = makeValidConfig();
    cfg.gimbalLimit = 0.0;
    auto engine = LiquidEngine::fromConfig(cfg);

    EXPECT_FALSE(engine->isGimbaled());
}

// =============================================================================
// Compute Output Tests
// =============================================================================

TEST_F(LiquidEngineTest, computeOutputFullThrottleVacuum) {
    auto cfg = makeValidConfig();
    auto engine = LiquidEngine::fromConfig(cfg);

    // Vacuum: ambient = 0, so pressure thrust = A_e * P_e
    auto output = engine->computeOutput(
        1.0,                    // full throttle
        0.0,                    // burnTime
        0.0,                    // vacuum
        Vec3{0.0, 0.0, 0.0}     // no gimbal
    );

    // F = thrust + A_e * (P_e - P_amb) = 100000 + 0.5 * (50000 - 0) = 125000 N
    EXPECT_DOUBLE_EQ(output.force.x, 125000.0);
    EXPECT_DOUBLE_EQ(output.force.y, 0.0);
    EXPECT_DOUBLE_EQ(output.force.z, 0.0);
}

TEST_F(LiquidEngineTest, computeOutputFullThrottleSeaLevel) {
    auto cfg = makeValidConfig();
    auto engine = LiquidEngine::fromConfig(cfg);

    constexpr double pSl = 101325.0;  // sea level pressure [Pa]

    auto output = engine->computeOutput(
        1.0,
        0.0,
        pSl,
        Vec3{0.0, 0.0, 0.0}
    );

    // F = 100000 + 0.5 * (50000 - 101325) = 100000 - 25662.5 = 74337.5 N
    EXPECT_NEAR(output.force.x, 74337.5, 0.1);
}

TEST_F(LiquidEngineTest, computeOutputThrottleScalesThrust) {
    auto cfg = makeValidConfig();
    auto engine = LiquidEngine::fromConfig(cfg);

    auto full = engine->computeOutput(1.0, 0.0, 0.0, Vec3{});
    auto half = engine->computeOutput(0.5, 0.0, 0.0, Vec3{});

    EXPECT_NEAR(half.force.x, full.force.x * 0.5, 0.1);
}

TEST_F(LiquidEngineTest, computeOutputThrottleScalesMdot) {
    auto cfg = makeValidConfig();
    auto engine = LiquidEngine::fromConfig(cfg);

    auto full = engine->computeOutput(1.0, 0.0, 0.0, Vec3{});
    auto half = engine->computeOutput(0.5, 0.0, 0.0, Vec3{});

    EXPECT_NEAR(half.mdot, full.mdot * 0.5, 1e-6);
}

// =============================================================================
// EngineOutput Tests
// =============================================================================

TEST(EngineOutputTest, zeroReturnsAllZeros) {
    constexpr auto output = EngineOutput::zero();

    EXPECT_DOUBLE_EQ(output.force.x, 0.0);
    EXPECT_DOUBLE_EQ(output.force.y, 0.0);
    EXPECT_DOUBLE_EQ(output.force.z, 0.0);
    EXPECT_DOUBLE_EQ(output.moment.x, 0.0);
    EXPECT_DOUBLE_EQ(output.moment.y, 0.0);
    EXPECT_DOUBLE_EQ(output.moment.z, 0.0);
    EXPECT_DOUBLE_EQ(output.mdot, 0.0);
}

}  // namespace
}  // namespace trajsim
