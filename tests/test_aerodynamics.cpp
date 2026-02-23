#include <gtest/gtest.h>
#include "models/vehicle/aerodynamics.hpp"
#include <cmath>
#include <limits>

namespace trajsim::test {

// =============================================================================
// Test Fixture
// =============================================================================

class AeroTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Default config for most tests
        config = Aerodynamics::Config{
            .refArea = 1.0,
            .refLength = 1.0,
            .machMin = 0.1,
            .machMax = 5.0,
            .machTransition = 0.8
        };

        // 2x2x2 low-speed table (Mach 0.3, 0.6)
        // Columns: Mach, Beta, Alpha, Cx, Cy, Cz, CMx, CMy, CMz
        lowSpeedTable = {
            {0.3, -5.0, -5.0,  0.10, -0.01, -0.05,  0.001, -0.010,  0.002},
            {0.3, -5.0,  5.0,  0.12, -0.01,  0.05,  0.001,  0.010,  0.002},
            {0.3,  5.0, -5.0,  0.10,  0.01, -0.05, -0.001, -0.010, -0.002},
            {0.3,  5.0,  5.0,  0.12,  0.01,  0.05, -0.001,  0.010, -0.002},
            {0.6, -5.0, -5.0,  0.15, -0.02, -0.08,  0.002, -0.015,  0.003},
            {0.6, -5.0,  5.0,  0.18, -0.02,  0.08,  0.002,  0.015,  0.003},
            {0.6,  5.0, -5.0,  0.15,  0.02, -0.08, -0.002, -0.015, -0.003},
            {0.6,  5.0,  5.0,  0.18,  0.02,  0.08, -0.002,  0.015, -0.003},
        };

        // 2x2x2 high-speed table (Mach 1.2, 2.5)
        highSpeedTable = {
            {1.2, -5.0, -5.0,  0.30, -0.03, -0.10,  0.003, -0.020,  0.004},
            {1.2, -5.0,  5.0,  0.35, -0.03,  0.10,  0.003,  0.020,  0.004},
            {1.2,  5.0, -5.0,  0.30,  0.03, -0.10, -0.003, -0.020, -0.004},
            {1.2,  5.0,  5.0,  0.35,  0.03,  0.10, -0.003,  0.020, -0.004},
            {2.5, -5.0, -5.0,  0.25, -0.02, -0.08,  0.002, -0.018,  0.003},
            {2.5, -5.0,  5.0,  0.28, -0.02,  0.08,  0.002,  0.018,  0.003},
            {2.5,  5.0, -5.0,  0.25,  0.02, -0.08, -0.002, -0.018, -0.003},
            {2.5,  5.0,  5.0,  0.28,  0.02,  0.08, -0.002,  0.018, -0.003},
        };
    }


    Aerodynamics::Config config;
    std::vector<std::vector<double>> lowSpeedTable;
    std::vector<std::vector<double>> highSpeedTable;

    static constexpr double TOL = 1e-10;
    static constexpr double LOOSE_TOL = 1e-6;
};

// =============================================================================
// Construction Tests
// =============================================================================

TEST_F(AeroTest, ConstructionWithValidTables) {
    EXPECT_NO_THROW(Aerodynamics(config, highSpeedTable, lowSpeedTable));
}

TEST_F(AeroTest, ConstructionThrowsOnEmptyHighSpeedTable) {
    std::vector<std::vector<double>> empty;
    EXPECT_THROW(
        Aerodynamics(config, empty, lowSpeedTable),
        std::invalid_argument
    );
}

TEST_F(AeroTest, ConstructionThrowsOnEmptyLowSpeedTable) {
    std::vector<std::vector<double>> empty;
    EXPECT_THROW(
        Aerodynamics(config, highSpeedTable, empty),
        std::invalid_argument
    );
}

TEST_F(AeroTest, ConstructionThrowsOnInsufficientColumns) {
    highSpeedTable[0].resize(5);  // Need 9 columns
    EXPECT_THROW(
        Aerodynamics(config, highSpeedTable, lowSpeedTable),
        std::invalid_argument
    );
}

TEST_F(AeroTest, ConstructionThrowsOnNaNInTable) {
    highSpeedTable[3][4] = std::nan("");
    EXPECT_THROW(
        Aerodynamics(config, highSpeedTable, lowSpeedTable),
        std::invalid_argument
    );
}

TEST_F(AeroTest, ConstructionThrowsOnPositiveInfInTable) {
    lowSpeedTable[0][3] = std::numeric_limits<double>::infinity();
    EXPECT_THROW(
        Aerodynamics(config, highSpeedTable, lowSpeedTable),
        std::invalid_argument
    );
}

TEST_F(AeroTest, ConstructionThrowsOnNegativeInfInTable) {
    lowSpeedTable[2][5] = -std::numeric_limits<double>::infinity();
    EXPECT_THROW(
        Aerodynamics(config, highSpeedTable, lowSpeedTable),
        std::invalid_argument
    );
}

TEST_F(AeroTest, ConstructionThrowsOnIncompleteGrid) {
    highSpeedTable.pop_back();  // Remove one row, breaking 2x2x2 grid
    EXPECT_THROW(
        Aerodynamics(config, highSpeedTable, lowSpeedTable),
        std::invalid_argument
    );
}

TEST_F(AeroTest, ConstructionThrowsOnSingleValueAxis) {
    // Table with only one Mach value (needs at least 2 for interpolation)
    std::vector<std::vector<double>> badTable = {
        {0.5, -5.0, -5.0,  0.1, 0.0, 0.0, 0.0, 0.0, 0.0},
        {0.5, -5.0,  5.0,  0.1, 0.0, 0.0, 0.0, 0.0, 0.0},
        {0.5,  5.0, -5.0,  0.1, 0.0, 0.0, 0.0, 0.0, 0.0},
        {0.5,  5.0,  5.0,  0.1, 0.0, 0.0, 0.0, 0.0, 0.0},
    };
    EXPECT_THROW(
        Aerodynamics(config, badTable, lowSpeedTable),
        std::invalid_argument
    );
}

// =============================================================================
// Config Validation Tests
// =============================================================================

TEST_F(AeroTest, ConfigThrowsOnZeroRefArea) {
    config.refArea = 0.0;
    EXPECT_THROW(
        Aerodynamics(config, highSpeedTable, lowSpeedTable),
        std::invalid_argument
    );
}

TEST_F(AeroTest, ConfigThrowsOnNegativeRefArea) {
    config.refArea = -1.0;
    EXPECT_THROW(
        Aerodynamics(config, highSpeedTable, lowSpeedTable),
        std::invalid_argument
    );
}

TEST_F(AeroTest, ConfigThrowsOnZeroRefLength) {
    config.refLength = 0.0;
    EXPECT_THROW(
        Aerodynamics(config, highSpeedTable, lowSpeedTable),
        std::invalid_argument
    );
}

TEST_F(AeroTest, ConfigThrowsOnNegativeRefLength) {
    config.refLength = -0.5;
    EXPECT_THROW(
        Aerodynamics(config, highSpeedTable, lowSpeedTable),
        std::invalid_argument
    );
}

TEST_F(AeroTest, ConfigThrowsOnNegativeMachMin) {
    config.machMin = -0.1;
    EXPECT_THROW(
        Aerodynamics(config, highSpeedTable, lowSpeedTable),
        std::invalid_argument
    );
}

TEST_F(AeroTest, ConfigThrowsOnMachMaxLessThanMin) {
    config.machMin = 2.0;
    config.machMax = 1.0;
    EXPECT_THROW(
        Aerodynamics(config, highSpeedTable, lowSpeedTable),
        std::invalid_argument
    );
}

TEST_F(AeroTest, ConfigThrowsOnMachMaxEqualToMin) {
    config.machMin = 1.0;
    config.machMax = 1.0;
    EXPECT_THROW(
        Aerodynamics(config, highSpeedTable, lowSpeedTable),
        std::invalid_argument
    );
}

TEST_F(AeroTest, ConfigThrowsOnTransitionBelowMin) {
    config.machTransition = -0.1;
    EXPECT_THROW(
        Aerodynamics(config, highSpeedTable, lowSpeedTable),
        std::invalid_argument
    );
}

TEST_F(AeroTest, ConfigThrowsOnTransitionAboveMax) {
    config.machTransition = 10.0;
    EXPECT_THROW(
        Aerodynamics(config, highSpeedTable, lowSpeedTable),
        std::invalid_argument
    );
}

TEST_F(AeroTest, ConfigAccessorReturnsCorrectValues) {
    Aerodynamics aero(config, highSpeedTable, lowSpeedTable);

    EXPECT_DOUBLE_EQ(aero.getConfig().refArea, config.refArea);
    EXPECT_DOUBLE_EQ(aero.getConfig().refLength, config.refLength);
    EXPECT_DOUBLE_EQ(aero.getConfig().machMin, config.machMin);
    EXPECT_DOUBLE_EQ(aero.getConfig().machMax, config.machMax);
    EXPECT_DOUBLE_EQ(aero.getConfig().machTransition, config.machTransition);
}

// =============================================================================
// Input Validation Tests
// =============================================================================

TEST_F(AeroTest, ComputeThrowsOnNaNMach) {
    Aerodynamics aero(config, highSpeedTable, lowSpeedTable);
    EXPECT_THROW(
        aero.compute(std::nan(""), 0.0, 0.0, 1000.0),
        std::invalid_argument
    );
}

TEST_F(AeroTest, ComputeThrowsOnNaNAlpha) {
    Aerodynamics aero(config, highSpeedTable, lowSpeedTable);
    EXPECT_THROW(
        aero.compute(0.5, std::nan(""), 0.0, 1000.0),
        std::invalid_argument
    );
}

TEST_F(AeroTest, ComputeThrowsOnNaNBeta) {
    Aerodynamics aero(config, highSpeedTable, lowSpeedTable);
    EXPECT_THROW(
        aero.compute(0.5, 0.0, std::nan(""), 1000.0),
        std::invalid_argument
    );
}

TEST_F(AeroTest, ComputeThrowsOnNaNDynamicPressure) {
    Aerodynamics aero(config, highSpeedTable, lowSpeedTable);
    EXPECT_THROW(
        aero.compute(0.5, 0.0, 0.0, std::nan("")),
        std::invalid_argument
    );
}

TEST_F(AeroTest, ComputeThrowsOnNegativeDynamicPressure) {
    Aerodynamics aero(config, highSpeedTable, lowSpeedTable);
    EXPECT_THROW(
        aero.compute(0.5, 0.0, 0.0, -1.0),
        std::invalid_argument
    );
}

TEST_F(AeroTest, ComputeThrowsOnAlphaAbove90) {
    Aerodynamics aero(config, highSpeedTable, lowSpeedTable);
    EXPECT_THROW(
        aero.compute(0.5, 90.1, 0.0, 1000.0),
        std::invalid_argument
    );
}

TEST_F(AeroTest, ComputeThrowsOnAlphaBelowNegative90) {
    Aerodynamics aero(config, highSpeedTable, lowSpeedTable);
    EXPECT_THROW(
        aero.compute(0.5, -90.1, 0.0, 1000.0),
        std::invalid_argument
    );
}

TEST_F(AeroTest, ComputeThrowsOnBetaAbove90) {
    Aerodynamics aero(config, highSpeedTable, lowSpeedTable);
    EXPECT_THROW(
        aero.compute(0.5, 0.0, 91.0, 1000.0),
        std::invalid_argument
    );
}

TEST_F(AeroTest, ComputeThrowsOnBetaBelowNegative90) {
    Aerodynamics aero(config, highSpeedTable, lowSpeedTable);
    EXPECT_THROW(
        aero.compute(0.5, 0.0, -95.0, 1000.0),
        std::invalid_argument
    );
}

TEST_F(AeroTest, ComputeAcceptsBoundaryAlpha) {
    Aerodynamics aero(config, highSpeedTable, lowSpeedTable);
    EXPECT_NO_THROW(aero.compute(0.5, 90.0, 0.0, 1000.0));
    EXPECT_NO_THROW(aero.compute(0.5, -90.0, 0.0, 1000.0));
}

TEST_F(AeroTest, ComputeAcceptsBoundaryBeta) {
    Aerodynamics aero(config, highSpeedTable, lowSpeedTable);
    EXPECT_NO_THROW(aero.compute(0.5, 0.0, 90.0, 1000.0));
    EXPECT_NO_THROW(aero.compute(0.5, 0.0, -90.0, 1000.0));
}

TEST_F(AeroTest, ComputeAcceptsZeroDynamicPressure) {
    Aerodynamics aero(config, highSpeedTable, lowSpeedTable);
    EXPECT_NO_THROW(aero.compute(0.5, 0.0, 0.0, 0.0));
}

// =============================================================================
// Table Selection Tests
// =============================================================================

TEST_F(AeroTest, UsesLowSpeedTableBelowTransition) {
    config.machTransition = 1.0;
    Aerodynamics aero(config, highSpeedTable, lowSpeedTable);

    // Mach 0.5 < transition, should use lowSpeedTable
    // Low speed table has Cx=0.10 at first corner (mach=0.3, beta=-5, alpha=-5)
    AeroResult result = aero.compute(0.3, -5.0, -5.0, 1000.0);
    EXPECT_NEAR(result.coef.forceCoef.x, 0.10, LOOSE_TOL);
}

TEST_F(AeroTest, UsesHighSpeedTableAtTransition) {
    config.machTransition = 1.0;
    Aerodynamics aero(config, highSpeedTable, lowSpeedTable);

    // Mach 1.2 >= transition, should use highSpeedTable
    // High speed table has Cx=0.30 at first corner (mach=1.2, beta=-5, alpha=-5)
    AeroResult result = aero.compute(1.2, -5.0, -5.0, 1000.0);
    EXPECT_NEAR(result.coef.forceCoef.x, 0.30, LOOSE_TOL);
}

TEST_F(AeroTest, UsesHighSpeedTableAboveTransition) {
    config.machTransition = 1.0;
    Aerodynamics aero(config, highSpeedTable, lowSpeedTable);

    // Mach 2.0 > transition
    AeroResult result = aero.compute(2.0, 0.0, 0.0, 1000.0);

    // Should be interpolating within high-speed table (Mach 1.2-2.5)
    EXPECT_GT(result.coef.forceCoef.x, 0.25);  // Between high-speed values
    EXPECT_LT(result.coef.forceCoef.x, 0.35);
}

// =============================================================================
// Mach Clamping Tests
// =============================================================================

TEST_F(AeroTest, ClampsMachBelowMinimum) {
    config.machMin = 0.2;
    Aerodynamics aero(config, highSpeedTable, lowSpeedTable);

    // Mach -0.5 should be clamped to machMin
    EXPECT_NO_THROW(aero.compute(-0.5, 0.0, 0.0, 1000.0));
}

TEST_F(AeroTest, ClampsMachAboveMaximum) {
    config.machMax = 3.0;
    Aerodynamics aero(config, highSpeedTable, lowSpeedTable);

    // Mach 10.0 should be clamped to machMax
    EXPECT_NO_THROW(aero.compute(10.0, 0.0, 0.0, 1000.0));
}

TEST_F(AeroTest, ClampedMachProducesSameResultAsLimit) {
    Aerodynamics aero(config, highSpeedTable, lowSpeedTable);

    AeroResult atMax = aero.compute(config.machMax, 0.0, 0.0, 1000.0);
    AeroResult aboveMax = aero.compute(config.machMax + 5.0, 0.0, 0.0, 1000.0);

    EXPECT_NEAR(atMax.coef.forceCoef.x, aboveMax.coef.forceCoef.x, TOL);
    EXPECT_NEAR(atMax.coef.forceCoef.y, aboveMax.coef.forceCoef.y, TOL);
    EXPECT_NEAR(atMax.coef.forceCoef.z, aboveMax.coef.forceCoef.z, TOL);
}

// =============================================================================
// Interpolation Accuracy Tests
// =============================================================================

TEST_F(AeroTest, ExactGridPointReturnsTableValue) {
    Aerodynamics aero(config, highSpeedTable, lowSpeedTable);

    // Exact grid point in low-speed table: mach=0.3, beta=-5.0, alpha=-5.0
    // Expected: Cx=0.10, Cy=-0.01, Cz=-0.05
    AeroResult result = aero.compute(0.3, -5.0, -5.0, 1.0);  // q=1 so force = coef

    EXPECT_NEAR(result.coef.forceCoef.x, 0.10, TOL);
    EXPECT_NEAR(result.coef.forceCoef.y, -0.01, TOL);
    EXPECT_NEAR(result.coef.forceCoef.z, -0.05, TOL);
}

TEST_F(AeroTest, MidpointAlphaInterpolation) {
    Aerodynamics aero(config, highSpeedTable, lowSpeedTable);

    // Midpoint in alpha at mach=0.3, beta=-5.0, alpha=0.0
    // Should average alpha=-5 and alpha=5 values
    // Cx: (0.10 + 0.12) / 2 = 0.11
    AeroResult result = aero.compute(0.3, 0.0, -5.0, 1.0);

    EXPECT_NEAR(result.coef.forceCoef.x, 0.11, LOOSE_TOL);
}

TEST_F(AeroTest, MidpointBetaInterpolation) {
    Aerodynamics aero(config, highSpeedTable, lowSpeedTable);

    // Midpoint in beta at mach=0.3, alpha=-5.0, beta=0.0
    // Cy: (-0.01 + 0.01) / 2 = 0.0
    AeroResult result = aero.compute(0.3, -5.0, 0.0, 1.0);

    EXPECT_NEAR(result.coef.forceCoef.y, 0.0, LOOSE_TOL);
}

TEST_F(AeroTest, MidpointMachInterpolation) {
    Aerodynamics aero(config, highSpeedTable, lowSpeedTable);

    // Midpoint in mach at beta=-5.0, alpha=-5.0, mach=0.45
    // Cx: (0.10 + 0.15) / 2 = 0.125
    AeroResult result = aero.compute(0.45, -5.0, -5.0, 1.0);

    EXPECT_NEAR(result.coef.forceCoef.x, 0.125, LOOSE_TOL);
}

TEST_F(AeroTest, TrilinearInterpolationCenter) {
    Aerodynamics aero(config, highSpeedTable, lowSpeedTable);

    // Center point should average all 8 corners
    // mach=0.45, beta=0.0, alpha=0.0
    AeroResult result = aero.compute(0.45, 0.0, 0.0, 1.0);

    // Cx average of all low-speed corners: (0.10+0.12+0.10+0.12+0.15+0.18+0.15+0.18)/8 = 0.1375
    EXPECT_NEAR(result.coef.forceCoef.x, 0.1375, LOOSE_TOL);
}

TEST_F(AeroTest, InterpolationIsContinuous) {
    Aerodynamics aero(config, highSpeedTable, lowSpeedTable);

    // Small steps should produce small changes
    AeroResult r1 = aero.compute(0.40, 0.0, 0.0, 1000.0);
    AeroResult r2 = aero.compute(0.41, 0.0, 0.0, 1000.0);

    double deltaCx = std::abs(r2.coef.forceCoef.x - r1.coef.forceCoef.x);
    EXPECT_LT(deltaCx, 0.01);  // Should be smooth
}

// =============================================================================
// Force and Moment Calculation Tests
// =============================================================================

TEST_F(AeroTest, ForceScalesWithDynamicPressure) {
    Aerodynamics aero(config, highSpeedTable, lowSpeedTable);

    AeroResult r1 = aero.compute(0.5, 0.0, 0.0, 1000.0);
    AeroResult r2 = aero.compute(0.5, 0.0, 0.0, 2000.0);

    EXPECT_NEAR(r2.force.x, r1.force.x * 2.0, TOL);
    EXPECT_NEAR(r2.force.y, r1.force.y * 2.0, TOL);
    EXPECT_NEAR(r2.force.z, r1.force.z * 2.0, TOL);
}

TEST_F(AeroTest, MomentScalesWithDynamicPressure) {
    Aerodynamics aero(config, highSpeedTable, lowSpeedTable);

    AeroResult r1 = aero.compute(0.5, 0.0, 0.0, 1000.0);
    AeroResult r2 = aero.compute(0.5, 0.0, 0.0, 3000.0);

    EXPECT_NEAR(r2.moment.x, r1.moment.x * 3.0, TOL);
    EXPECT_NEAR(r2.moment.y, r1.moment.y * 3.0, TOL);
    EXPECT_NEAR(r2.moment.z, r1.moment.z * 3.0, TOL);
}

TEST_F(AeroTest, ForceScalesWithRefArea) {
    Aerodynamics::Config config2 = config;
    config2.refArea = 2.0;

    Aerodynamics aero1(config, highSpeedTable, lowSpeedTable);
    Aerodynamics aero2(config2, highSpeedTable, lowSpeedTable);

    AeroResult r1 = aero1.compute(0.5, 0.0, 0.0, 1000.0);
    AeroResult r2 = aero2.compute(0.5, 0.0, 0.0, 1000.0);

    EXPECT_NEAR(r2.force.x, r1.force.x * 2.0, TOL);
}
}
