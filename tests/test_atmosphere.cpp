// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Vincent Ning

/// @file atmosphere_test.cpp
/// @brief Unit tests for AtmosphereModel
///
/// Reference values from US Standard Atmosphere 1976 (NOAA-S/T 76-1562)
/// Note: Reference values are defined at geopotential altitudes.
/// The model accepts geometric altitude and converts internally.

#include <gtest/gtest.h>
#include <cmath>
#include "models/atmosphere.hpp"

namespace trajsim::test {

// =============================================================================
// Test Fixture
// =============================================================================

class AtmosphereTest : public ::testing::Test {
protected:
    void SetUp() override {
        atmo = AtmosphereModel{};
        zeroVelocity = Vec3{0.0, 0.0, 0.0};
    }

    /// Convert geopotential altitude (reference) to geometric altitude (input)
    /// h_geometric = h_geopotential * R / (R - h_geopotential)
    static constexpr double toGeometric(double hGeopotential) {
        constexpr double R = 6356752.0;  // Polar radius [m]
        return hGeopotential * R / (R - hGeopotential);
    }

    AtmosphereModel atmo;
    Vec3 zeroVelocity;

    // Tolerance for floating point comparisons
    static constexpr double TEMP_TOL = 0.1;         // [K]
    static constexpr double PRESS_TOL = 1.0;        // [Pa] for low altitude
    static constexpr double PRESS_REL_TOL = 0.01;   // 1% relative for high altitude
    static constexpr double DENSITY_REL_TOL = 0.01;
};

// =============================================================================
// Sea Level Conditions (h = 0)
// =============================================================================

TEST_F(AtmosphereTest, SeaLevelTemperature) {
    auto state = atmo.computeStates(zeroVelocity, 0.0);
    EXPECT_NEAR(state.temperature, 288.15, TEMP_TOL);
}

TEST_F(AtmosphereTest, SeaLevelPressure) {
    auto state = atmo.computeStates(zeroVelocity, 0.0);
    EXPECT_NEAR(state.pressure, 101325.0, PRESS_TOL);
}

TEST_F(AtmosphereTest, SeaLevelDensity) {
    auto state = atmo.computeStates(zeroVelocity, 0.0);
    EXPECT_NEAR(state.density, 1.225, 0.001);
}

TEST_F(AtmosphereTest, SeaLevelSpeedOfSound) {
    auto state = atmo.computeStates(zeroVelocity, 0.0);
    EXPECT_NEAR(state.speedOfSound, 340.29, 0.5);
}

// =============================================================================
// Troposphere (0 - 11 km geopotential)
// =============================================================================

TEST_F(AtmosphereTest, Troposphere5km) {
    auto state = atmo.computeStates(zeroVelocity, toGeometric(5000.0));

    EXPECT_NEAR(state.temperature, 255.65, TEMP_TOL);
    EXPECT_NEAR(state.pressure, 54019.0, state.pressure * PRESS_REL_TOL);
}

TEST_F(AtmosphereTest, Troposphere10km) {
    auto state = atmo.computeStates(zeroVelocity, toGeometric(10000.0));

    EXPECT_NEAR(state.temperature, 223.15, TEMP_TOL);
    EXPECT_NEAR(state.pressure, 26436.0, state.pressure * PRESS_REL_TOL);
}

// =============================================================================
// Tropopause / Stratosphere Boundary (11 km geopotential)
// =============================================================================

TEST_F(AtmosphereTest, Tropopause11km) {
    auto state = atmo.computeStates(zeroVelocity, toGeometric(11000.0));

    EXPECT_NEAR(state.temperature, 216.65, TEMP_TOL);
    EXPECT_NEAR(state.pressure, 22632.0, state.pressure * PRESS_REL_TOL);
}

// =============================================================================
// Stratosphere - Isothermal Layer (11 - 20 km geopotential)
// =============================================================================

TEST_F(AtmosphereTest, StratosphereIsothermal15km) {
    auto state = atmo.computeStates(zeroVelocity, toGeometric(15000.0));

    EXPECT_NEAR(state.temperature, 216.65, TEMP_TOL);
}

TEST_F(AtmosphereTest, StratosphereIsothermal20km) {
    auto state = atmo.computeStates(zeroVelocity, toGeometric(20000.0));

    EXPECT_NEAR(state.temperature, 216.65, TEMP_TOL);
    EXPECT_NEAR(state.pressure, 5474.9, state.pressure * PRESS_REL_TOL);
}

// =============================================================================
// Stratosphere - Gradient Layer (20 - 32 km geopotential)
// =============================================================================

TEST_F(AtmosphereTest, StratosphereGradient25km) {
    auto state = atmo.computeStates(zeroVelocity, toGeometric(25000.0));

    EXPECT_GT(state.temperature, 216.65);
    EXPECT_LT(state.temperature, 228.65);
}

TEST_F(AtmosphereTest, StratosphereGradient32km) {
    auto state = atmo.computeStates(zeroVelocity, toGeometric(32000.0));

    EXPECT_NEAR(state.temperature, 228.65, TEMP_TOL);
    EXPECT_NEAR(state.pressure, 868.0, state.pressure * PRESS_REL_TOL);
}

// =============================================================================
// High Altitude Layers (32 - 71 km geopotential)
// =============================================================================

TEST_F(AtmosphereTest, UpperStratosphere47km) {
    auto state = atmo.computeStates(zeroVelocity, toGeometric(47000.0));

    EXPECT_NEAR(state.temperature, 270.65, TEMP_TOL);
    EXPECT_NEAR(state.pressure, 110.9, state.pressure * PRESS_REL_TOL);
}

TEST_F(AtmosphereTest, Mesosphere71km) {
    auto state = atmo.computeStates(zeroVelocity, toGeometric(71000.0));

    EXPECT_NEAR(state.temperature, 214.65, TEMP_TOL);
}

// =============================================================================
// Mach Number and Dynamic Pressure
// =============================================================================

TEST_F(AtmosphereTest, MachNumberSubsonic) {
    Vec3 velocity{170.0, 0.0, 0.0};
    auto state = atmo.computeStates(velocity, 0.0);

    double expectedMach = 170.0 / state.speedOfSound;
    EXPECT_NEAR(state.machNumber, expectedMach, 0.001);
    EXPECT_LT(state.machNumber, 1.0);
}

TEST_F(AtmosphereTest, MachNumberSupersonic) {
    Vec3 velocity{400.0, 0.0, 0.0};
    auto state = atmo.computeStates(velocity, 0.0);

    EXPECT_GT(state.machNumber, 1.0);
}

TEST_F(AtmosphereTest, DynamicPressureFormula) {
    Vec3 velocity{100.0, 0.0, 0.0};
    auto state = atmo.computeStates(velocity, 0.0);

    double expectedQ = 0.5 * state.density * 100.0 * 100.0;
    EXPECT_NEAR(state.dynamicPressure, expectedQ, 0.1);
}

TEST_F(AtmosphereTest, DynamicPressureZeroVelocity) {
    auto state = atmo.computeStates(zeroVelocity, 0.0);

    EXPECT_DOUBLE_EQ(state.dynamicPressure, 0.0);
    EXPECT_DOUBLE_EQ(state.machNumber, 0.0);
}

TEST_F(AtmosphereTest, DynamicPressure3DVelocity) {
    Vec3 velocity{60.0, 80.0, 0.0};  // |V| = 100 m/s
    auto state = atmo.computeStates(velocity, 0.0);

    double expectedQ = 0.5 * state.density * 100.0 * 100.0;
    EXPECT_NEAR(state.dynamicPressure, expectedQ, 0.1);
}

// =============================================================================
// Layer Boundary Continuity (using geometric altitudes)
// =============================================================================

TEST_F(AtmosphereTest, ContinuityAt11km) {
    double h = toGeometric(11000.0);
    auto below = atmo.computeStates(zeroVelocity, h - 1.0);
    auto at    = atmo.computeStates(zeroVelocity, h);
    auto above = atmo.computeStates(zeroVelocity, h + 1.0);

    EXPECT_NEAR(below.temperature, at.temperature, 0.1);
    EXPECT_NEAR(at.temperature, above.temperature, 0.1);
    EXPECT_NEAR(below.pressure, at.pressure, at.pressure * 0.001);
    EXPECT_NEAR(at.pressure, above.pressure, at.pressure * 0.001);
}

TEST_F(AtmosphereTest, ContinuityAt20km) {
    double h = toGeometric(20000.0);
    auto below = atmo.computeStates(zeroVelocity, h - 1.0);
    auto at    = atmo.computeStates(zeroVelocity, h);
    auto above = atmo.computeStates(zeroVelocity, h + 1.0);

    EXPECT_NEAR(below.temperature, at.temperature, 0.1);
    EXPECT_NEAR(at.temperature, above.temperature, 0.1);
}

// =============================================================================
// Physical Consistency
// =============================================================================

TEST_F(AtmosphereTest, PressureDecreasesWithAltitude) {
    auto low  = atmo.computeStates(zeroVelocity, 0.0);
    auto mid  = atmo.computeStates(zeroVelocity, 20000.0);
    auto high = atmo.computeStates(zeroVelocity, 50000.0);

    EXPECT_GT(low.pressure, mid.pressure);
    EXPECT_GT(mid.pressure, high.pressure);
}

TEST_F(AtmosphereTest, DensityDecreasesWithAltitude) {
    auto low  = atmo.computeStates(zeroVelocity, 0.0);
    auto mid  = atmo.computeStates(zeroVelocity, 20000.0);
    auto high = atmo.computeStates(zeroVelocity, 50000.0);

    EXPECT_GT(low.density, mid.density);
    EXPECT_GT(mid.density, high.density);
}

TEST_F(AtmosphereTest, IdealGasLawConsistency) {
    constexpr double R = 287.053;

    for (double h = 0.0; h <= 70000.0; h += 10000.0) {
        auto state = atmo.computeStates(zeroVelocity, h);
        double computedPressure = state.density * R * state.temperature;

        EXPECT_NEAR(state.pressure, computedPressure, state.pressure * 0.001)
            << "Ideal gas law failed at altitude " << h << " m";
    }
}

TEST_F(AtmosphereTest, SpeedOfSoundFormula) {
    constexpr double GAMMA = 1.4;
    constexpr double R = 287.053;

    auto state = atmo.computeStates(zeroVelocity, 10000.0);
    double expectedA = std::sqrt(GAMMA * R * state.temperature);

    EXPECT_NEAR(state.speedOfSound, expectedA, 0.01);
}

// =============================================================================
// Edge Cases
// =============================================================================

TEST_F(AtmosphereTest, VeryHighAltitudeExtrapolates) {
    auto state = atmo.computeStates(zeroVelocity, 80000.0);

    EXPECT_TRUE(std::isfinite(state.temperature));
    EXPECT_TRUE(std::isfinite(state.pressure));
    EXPECT_TRUE(std::isfinite(state.density));
    EXPECT_GT(state.pressure, 0.0);
}

TEST_F(AtmosphereTest, AllOutputsFinite) {
    std::vector<double> altitudes = {0, 5000, 11000, 15000, 20000, 32000, 47000, 51000, 71000};
    Vec3 velocity{200.0, 50.0, 10.0};

    for (double h : altitudes) {
        auto state = atmo.computeStates(velocity, h);

        EXPECT_TRUE(std::isfinite(state.temperature)) << "at h=" << h;
        EXPECT_TRUE(std::isfinite(state.pressure)) << "at h=" << h;
        EXPECT_TRUE(std::isfinite(state.density)) << "at h=" << h;
        EXPECT_TRUE(std::isfinite(state.speedOfSound)) << "at h=" << h;
        EXPECT_TRUE(std::isfinite(state.machNumber)) << "at h=" << h;
        EXPECT_TRUE(std::isfinite(state.dynamicPressure)) << "at h=" << h;
    }
}

}  // namespace trajsim::test
