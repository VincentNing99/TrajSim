// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Vincent Ning

/// @file test_vehicle.cpp
/// @brief Unit tests for the Vehicle aggregator class.
///
/// Tests cover:
///   - Construction with valid subsystems
///   - Getter accessors return correct values
///   - Move semantics (Vehicle is non-copyable)
///   - Null engine handling
///   - Aero and engine subsystem access

#include <gtest/gtest.h>
#include "models/vehicle/vehicle.hpp"
#include "models/vehicle/engine_models/liquid_engine.hpp"

namespace trajsim::test {

// =============================================================================
// Helpers
// =============================================================================

/// @brief Build a minimal zero-coefficient Aerodynamics for testing.
static Aerodynamics makeTestAero() {
    Aerodynamics::Config cfg{
        .refArea = 10.0,
        .refLength = 5.0,
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

/// @brief Create a valid LiquidEngine for testing.
static std::unique_ptr<Engine> makeTestEngine() {
    LiquidEngine::Config cfg{
        .id = "TestEngine",
        .thrust = 100000.0,
        .isp = 300.0,
        .mdot = 34.0,
        .dryMass = 500.0,
        .nozzleExitArea = 0.5,
        .nozzleExitPressure = 50000.0,
        .mountOffset = Vec3{0.0, 0.0, 0.0},
        .gimbalLimit = 0.1,
        .throttleMin = 0.4
    };
    return LiquidEngine::fromConfig(cfg);
}

// =============================================================================
// Test Fixture
// =============================================================================

class VehicleTest : public ::testing::Test {
protected:
    static constexpr double MASS = 50000.0;
    static constexpr int NUM_STAGES = 2;
};

// =============================================================================
// Construction Tests
// =============================================================================

TEST_F(VehicleTest, ConstructionWithEngine) {
    Vehicle::Config cfg{.numberOfStage = NUM_STAGES, .mass = MASS};
    EXPECT_NO_THROW(Vehicle(cfg, makeTestAero(), makeTestEngine()));
}

TEST_F(VehicleTest, ConstructionWithNullEngine) {
    Vehicle::Config cfg{.numberOfStage = 1, .mass = MASS};
    EXPECT_NO_THROW(Vehicle(cfg, makeTestAero(), nullptr));
}

// =============================================================================
// Getter Tests
// =============================================================================

TEST_F(VehicleTest, GetMassReturnsConfigValue) {
    Vehicle::Config cfg{.numberOfStage = NUM_STAGES, .mass = MASS};
    Vehicle v(cfg, makeTestAero(), makeTestEngine());

    EXPECT_DOUBLE_EQ(v.getMass(), MASS);
}

TEST_F(VehicleTest, GetNumberOfStageReturnsConfigValue) {
    Vehicle::Config cfg{.numberOfStage = NUM_STAGES, .mass = MASS};
    Vehicle v(cfg, makeTestAero(), makeTestEngine());

    EXPECT_DOUBLE_EQ(v.getNumberOfStage(), NUM_STAGES);
}

TEST_F(VehicleTest, GetAeroReturnsValidReference) {
    Vehicle::Config cfg{.numberOfStage = 1, .mass = MASS};
    Vehicle v(cfg, makeTestAero(), makeTestEngine());

    const auto& aero = v.getAero();
    EXPECT_DOUBLE_EQ(aero.getConfig().refArea, 10.0);
    EXPECT_DOUBLE_EQ(aero.getConfig().refLength, 5.0);
}

TEST_F(VehicleTest, GetEngineReturnsValidReference) {
    Vehicle::Config cfg{.numberOfStage = 1, .mass = MASS};
    Vehicle v(cfg, makeTestAero(), makeTestEngine());

    const auto& engine = v.getEngine();
    EXPECT_EQ(engine.getId(), "TestEngine");
    EXPECT_DOUBLE_EQ(engine.getThrust(), 100000.0);
    EXPECT_DOUBLE_EQ(engine.getIsp(), 300.0);
}

TEST_F(VehicleTest, MutableEngineAccessWorks) {
    Vehicle::Config cfg{.numberOfStage = 1, .mass = MASS};
    Vehicle v(cfg, makeTestAero(), makeTestEngine());

    // Non-const access
    Engine& engine = v.getEngine();
    EXPECT_EQ(engine.getId(), "TestEngine");
}

// =============================================================================
// Move Semantics
// =============================================================================

TEST_F(VehicleTest, MoveConstructionTransfersOwnership) {
    Vehicle::Config cfg{.numberOfStage = 1, .mass = MASS};
    Vehicle v1(cfg, makeTestAero(), makeTestEngine());

    Vehicle v2(std::move(v1));
    EXPECT_DOUBLE_EQ(v2.getMass(), MASS);
    EXPECT_EQ(v2.getEngine().getId(), "TestEngine");
}

// =============================================================================
// Aero Subsystem Integration
// =============================================================================

TEST_F(VehicleTest, AeroComputeWorksAfterConstruction) {
    Vehicle::Config cfg{.numberOfStage = 1, .mass = MASS};
    Vehicle v(cfg, makeTestAero(), makeTestEngine());

    // Zero-coefficient aero should return zero forces
    AeroResult result = v.getAero().compute(0.5, 0.0, 0.0, 1000.0, 1.0);
    EXPECT_DOUBLE_EQ(result.force.x, 0.0);
    EXPECT_DOUBLE_EQ(result.force.y, 0.0);
    EXPECT_DOUBLE_EQ(result.force.z, 0.0);
}

// =============================================================================
// Engine Subsystem Integration
// =============================================================================

TEST_F(VehicleTest, EngineComputeOutputWorksAfterConstruction) {
    Vehicle::Config cfg{.numberOfStage = 1, .mass = MASS};
    Vehicle v(cfg, makeTestAero(), makeTestEngine());

    auto output = v.getEngine().computeOutput(1.0, 0.0, 0.0, Vec3{});
    EXPECT_GT(output.force.x, 0.0);  // Thrust along body x-axis
}

TEST_F(VehicleTest, EngineCapabilitiesAccessible) {
    Vehicle::Config cfg{.numberOfStage = 1, .mass = MASS};
    Vehicle v(cfg, makeTestAero(), makeTestEngine());

    EXPECT_TRUE(v.getEngine().isThrottleable());
    EXPECT_TRUE(v.getEngine().isGimbaled());
}

}  // namespace trajsim::test
