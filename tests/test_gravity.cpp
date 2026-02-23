// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Vincent Ning

/// @file test_gravity.cpp
/// @brief Unit tests for the Gravity J2 oblate Earth model
///
/// Reference values are derived analytically from WGS84 constants:
///   - Equatorial gravity (gravitational only, no centrifugal): μ/Re² * (1+J) ≈ 9.8135 m/s²
///   - Polar gravity: accounts for J2 radial + polar-axis terms        ≈ 9.831  m/s²
///
/// Coordinate frame convention (local launch frame):
///   - Origin: launch site
///   - Y-axis: local vertical (up)
///   - X-Z plane: local horizontal

#include <gtest/gtest.h>
#include <cmath>
#include <numbers>
#include "models/gravity.hpp"

namespace trajsim::test {

// =============================================================================
// Test Fixture
// =============================================================================

class GravityTest : public ::testing::Test {
protected:
    static constexpr double DEG_TO_RAD = std::numbers::pi / 180.0;

    // Geodetic latitudes [rad]
    static constexpr double LAT_EQUATOR = 0.0;
    static constexpr double LAT_POLAR   = std::numbers::pi / 2.0;
    static constexpr double LAT_KSC     = 28.574 * DEG_TO_RAD;  // Kennedy Space Center
    static constexpr double LAT_MID     = 45.0   * DEG_TO_RAD;

    static constexpr double ALT_SEA_LEVEL = 0.0;

    // Tolerances
    static constexpr double GRAV_TOL   = 0.01;  // [m/s²] gravitational magnitude
    static constexpr double RADIUS_TOL = 1.0;   // [m]    geocentric radius
};

// =============================================================================
// getLaunchGeocentricRadius
// =============================================================================

TEST_F(GravityTest, EquatorialRadiusMatchesEquatorialConstant) {
    // At the equator (geocentric = geodetic = 0), the ellipsoid radius equals Re.
    Gravity g(LAT_EQUATOR, ALT_SEA_LEVEL);

    EXPECT_NEAR(g.getLaunchGeocentricRadius(), EarthConstants::RADIUS_OF_EQUATOR, RADIUS_TOL);
}

TEST_F(GravityTest, PolarRadiusMatchesSemiMinorAxis) {
    // At the poles (geocentric latitude = 90°), the ellipsoid radius equals b.
    Gravity g(LAT_POLAR, ALT_SEA_LEVEL);

    EXPECT_NEAR(g.getLaunchGeocentricRadius(), EarthConstants::SEMI_MINOR_AXIS, RADIUS_TOL);
}

TEST_F(GravityTest, MidLatitudeRadiusBetweenEquatorialAndPolar) {
    Gravity g(LAT_MID, ALT_SEA_LEVEL);
    double r = g.getLaunchGeocentricRadius();

    EXPECT_GT(r, EarthConstants::SEMI_MINOR_AXIS);
    EXPECT_LT(r, EarthConstants::RADIUS_OF_EQUATOR);
}

TEST_F(GravityTest, AltitudeAddsDirectlyToGeocentricRadius) {
    constexpr double altitude = 100000.0;  // 100 km
    Gravity gSL(LAT_EQUATOR, ALT_SEA_LEVEL);
    Gravity gH (LAT_EQUATOR, altitude);

    EXPECT_NEAR(
        gH.getLaunchGeocentricRadius() - gSL.getLaunchGeocentricRadius(),
        altitude,
        RADIUS_TOL
    );
}

TEST_F(GravityTest, GeoCentricRadiusPositiveAtAllLatitudes) {
    const std::vector<double> lats = {
        LAT_EQUATOR, LAT_KSC, LAT_MID, LAT_POLAR
    };
    for (double lat : lats) {
        Gravity g(lat, ALT_SEA_LEVEL);
        EXPECT_GT(g.getLaunchGeocentricRadius(), 0.0) << "lat_deg=" << lat / DEG_TO_RAD;
    }
}

// =============================================================================
// computeAcceleration - Direction at Launch Origin
// =============================================================================

TEST_F(GravityTest, EquatorialAccelerationPurelyDownward) {
    // At equator with azimuth=0, sinPhi = rHat · polarAxis = 0,
    // so gOmega = 0. Gravity is purely radial (−Y direction in local frame).
    Gravity g(LAT_EQUATOR, ALT_SEA_LEVEL);
    Vec3 acc = g.computeAcceleration(Vec3{0.0, 0.0, 0.0});

    EXPECT_NEAR(acc.x, 0.0, 1e-6);
    EXPECT_NEAR(acc.z, 0.0, 1e-6);
    EXPECT_LT(acc.y, 0.0);  // downward
}

TEST_F(GravityTest, PolarAccelerationPurelyDownward) {
    // At poles, polarAxis = {0,1,0}. Both the radial and polar-axis terms
    // point in the −Y direction; no X or Z component.
    Gravity g(LAT_POLAR, ALT_SEA_LEVEL);
    Vec3 acc = g.computeAcceleration(Vec3{0.0, 0.0, 0.0});

    EXPECT_NEAR(acc.x, 0.0, 1e-6);
    EXPECT_NEAR(acc.z, 0.0, 1e-6);
    EXPECT_LT(acc.y, 0.0);
}

// =============================================================================
// computeAcceleration - Magnitude at Launch Origin
// =============================================================================

TEST_F(GravityTest, EquatorialGravityMagnitude) {
    // At equator, sinPhi = 0:
    //   g = μ/Re² * (1 + J*(1 - 0)) = μ/Re² * (1 + J)
    //   ≈ 9.7976 * 1.001624 ≈ 9.8135 m/s²
    Gravity g(LAT_EQUATOR, ALT_SEA_LEVEL);
    Vec3 acc = g.computeAcceleration(Vec3{0.0, 0.0, 0.0});

    EXPECT_NEAR(acc.mag(), 9.8135, GRAV_TOL);
}

TEST_F(GravityTest, PolarGravityMagnitude) {
    // At poles, the J2 radial correction reduces the radial term but the
    // polar-axis term adds back. Combined effect ≈ 9.831 m/s².
    Gravity g(LAT_POLAR, ALT_SEA_LEVEL);
    Vec3 acc = g.computeAcceleration(Vec3{0.0, 0.0, 0.0});

    EXPECT_NEAR(acc.mag(), 9.831, 0.05);
}

TEST_F(GravityTest, PolarGravityGreaterThanEquatorial) {
    // Poles are closer to Earth's center (oblate spheroid), so gravity is stronger.
    Gravity gEq (LAT_EQUATOR, ALT_SEA_LEVEL);
    Gravity gPol(LAT_POLAR,   ALT_SEA_LEVEL);

    double magEq  = gEq.computeAcceleration(Vec3{0.0, 0.0, 0.0}).mag();
    double magPol = gPol.computeAcceleration(Vec3{0.0, 0.0, 0.0}).mag();

    EXPECT_GT(magPol, magEq);
}

TEST_F(GravityTest, KSCGravityMagnitudeInReasonableRange) {
    // KSC at ~28.6°N should produce gravity between equatorial and polar values.
    Gravity g(LAT_KSC, ALT_SEA_LEVEL);
    double mag = g.computeAcceleration(Vec3{0.0, 0.0, 0.0}).mag();

    EXPECT_GT(mag, 9.78);
    EXPECT_LT(mag, 9.84);
}

// =============================================================================
// computeAcceleration - Altitude Dependence
// =============================================================================

TEST_F(GravityTest, GravityDecreasesMonotonicallyWithAltitude) {
    Gravity g(LAT_EQUATOR, ALT_SEA_LEVEL);

    double g0 = g.computeAcceleration(Vec3{0.0,       0.0, 0.0}).mag();
    double g1 = g.computeAcceleration(Vec3{0.0,  100000.0, 0.0}).mag();
    double g2 = g.computeAcceleration(Vec3{0.0,  500000.0, 0.0}).mag();

    EXPECT_GT(g0, g1);
    EXPECT_GT(g1, g2);
}

TEST_F(GravityTest, GravityFalloffApproximatesInverseSquare) {
    // Far from Earth, J2 effects are negligible and g ∝ 1/r².
    // At r = 2*Re:  g(2Re) / g(Re) ≈ 0.25
    constexpr double Re = EarthConstants::RADIUS_OF_EQUATOR;
    Gravity g(LAT_EQUATOR, ALT_SEA_LEVEL);

    double gSurface = g.computeAcceleration(Vec3{0.0,  0.0, 0.0}).mag();
    double gDoubleR = g.computeAcceleration(Vec3{0.0,  Re,  0.0}).mag();

    EXPECT_NEAR(gDoubleR / gSurface, 0.25, 0.02);
}

TEST_F(GravityTest, HighAltitudeConstructionWeakensLaunchSiteGravity) {
    // Constructing at 100 km altitude means the launch origin is farther out.
    constexpr double alt = 100000.0;
    Gravity gSL(LAT_EQUATOR, ALT_SEA_LEVEL);
    Gravity gH (LAT_EQUATOR, alt);

    double magSL = gSL.computeAcceleration(Vec3{0.0, 0.0, 0.0}).mag();
    double magH  = gH.computeAcceleration(Vec3{0.0, 0.0, 0.0}).mag();

    EXPECT_LT(magH, magSL);
}

// =============================================================================
// computeAcceleration - J2 Perturbation
// =============================================================================

TEST_F(GravityTest, EquatorialGravityExceedsPureNewton) {
    // At equator, sinPhi = 0. The J2 radial term is positive: factor = (1 + J).
    // So J2 increases gravity above the simple Newton's law value.
    constexpr double Re = EarthConstants::RADIUS_OF_EQUATOR;
    constexpr double mu = EarthConstants::GRAVITATIONAL_CONST;
    Gravity g(LAT_EQUATOR, ALT_SEA_LEVEL);

    double gNewton = mu / (Re * Re);
    double gJ2     = g.computeAcceleration(Vec3{0.0, 0.0, 0.0}).mag();

    EXPECT_GT(gJ2, gNewton);
}

TEST_F(GravityTest, GravityMagnitudeIsLatitudeDependent) {
    // J2 breaks spherical symmetry; different latitudes produce different magnitudes.
    Gravity gEq  (LAT_EQUATOR, ALT_SEA_LEVEL);
    Gravity gMid (LAT_MID,     ALT_SEA_LEVEL);
    Gravity gPol (LAT_POLAR,   ALT_SEA_LEVEL);

    double magEq  = gEq.computeAcceleration(Vec3{}).mag();
    double magMid = gMid.computeAcceleration(Vec3{}).mag();
    double magPol = gPol.computeAcceleration(Vec3{}).mag();

    // Gravity increases from equator to pole (monotonically in J2 model).
    EXPECT_LT(magEq, magMid);
    EXPECT_LT(magMid, magPol);
}

// =============================================================================
// computeAcceleration - Finite Output Checks
// =============================================================================

TEST_F(GravityTest, AccelerationFiniteAtAllLatitudes) {
    const std::vector<double> lats = {
        0.0,
        15.0 * DEG_TO_RAD,
        30.0 * DEG_TO_RAD,
        45.0 * DEG_TO_RAD,
        60.0 * DEG_TO_RAD,
        75.0 * DEG_TO_RAD,
        LAT_POLAR
    };

    for (double lat : lats) {
        Gravity g(lat, ALT_SEA_LEVEL);
        Vec3 acc = g.computeAcceleration(Vec3{0.0, 0.0, 0.0});

        EXPECT_TRUE(acc.is_finite()) << "lat_deg=" << lat / DEG_TO_RAD;
        EXPECT_GT(acc.mag(), 0.0)    << "lat_deg=" << lat / DEG_TO_RAD;
    }
}

TEST_F(GravityTest, AccelerationFiniteAtVariousAltitudes) {
    Gravity g(LAT_KSC, ALT_SEA_LEVEL);
    const std::vector<double> alts = {0.0, 1.0e4, 1.0e5, 5.0e5, 1.0e6, 4.0e6};

    for (double h : alts) {
        Vec3 acc = g.computeAcceleration(Vec3{0.0, h, 0.0});

        EXPECT_TRUE(acc.is_finite()) << "h=" << h;
        EXPECT_GT(acc.mag(), 0.0)    << "h=" << h;
    }
}

TEST_F(GravityTest, AccelerationFiniteWithNonZeroAzimuth) {
    const std::vector<double> azimuths = {0.0, std::numbers::pi / 4.0, std::numbers::pi / 2.0, std::numbers::pi};

    for (double az : azimuths) {
        Gravity g(LAT_KSC, ALT_SEA_LEVEL, az);
        Vec3 acc = g.computeAcceleration(Vec3{0.0, 0.0, 0.0});

        EXPECT_TRUE(acc.is_finite()) << "azimuth=" << az;
    }
}

TEST_F(GravityTest, AccelerationFiniteAtOffAxisPosition) {
    Gravity g(LAT_KSC, ALT_SEA_LEVEL);
    const std::vector<Vec3> positions = {
        {  10000.0,  50000.0,  0.0},
        { -10000.0, 100000.0,  5000.0},
        {      0.0, 200000.0, -3000.0},
    };

    for (const auto& pos : positions) {
        Vec3 acc = g.computeAcceleration(pos);
        EXPECT_TRUE(acc.is_finite())
            << "pos=(" << pos.x << "," << pos.y << "," << pos.z << ")";
    }
}

// =============================================================================
// computeAltitude - At Launch Origin
// =============================================================================

TEST_F(GravityTest, AltitudeAtOriginEqualsSeaLevel) {
    // pos={0,0,0} is the launch site itself. Altitude should equal the
    // construction altitude.
    const std::vector<double> lats = {LAT_EQUATOR, LAT_KSC, LAT_MID, LAT_POLAR};

    for (double lat : lats) {
        Gravity g(lat, ALT_SEA_LEVEL);
        double alt = g.computeAltitude(Vec3{0.0, 0.0, 0.0});

        EXPECT_NEAR(alt, ALT_SEA_LEVEL, RADIUS_TOL)
            << "lat_deg=" << lat / DEG_TO_RAD;
    }
}

TEST_F(GravityTest, AltitudeAtOriginMatchesNonZeroConstructionAltitude) {
    constexpr double buildAlt = 50000.0;  // 50 km
    Gravity g(LAT_KSC, buildAlt);

    EXPECT_NEAR(g.computeAltitude(Vec3{0.0, 0.0, 0.0}), buildAlt, RADIUS_TOL);
}

// =============================================================================
// computeAltitude - Monotonicity and Consistency
// =============================================================================

TEST_F(GravityTest, AltitudeIncreasesWithUpwardDisplacement) {
    Gravity g(LAT_EQUATOR, ALT_SEA_LEVEL);

    double alt0 = g.computeAltitude(Vec3{0.0,       0.0, 0.0});
    double alt1 = g.computeAltitude(Vec3{0.0,  50000.0, 0.0});
    double alt2 = g.computeAltitude(Vec3{0.0, 200000.0, 0.0});

    EXPECT_LT(alt0, alt1);
    EXPECT_LT(alt1, alt2);
}

TEST_F(GravityTest, AltitudeChangeApproximatesVerticalDisplacement) {
    // For small displacements, Δaltitude ≈ ΔY (within 1%).
    constexpr double dY = 10000.0;
    Gravity g(LAT_EQUATOR, ALT_SEA_LEVEL);

    double alt0 = g.computeAltitude(Vec3{0.0,  0.0, 0.0});
    double alt1 = g.computeAltitude(Vec3{0.0, dY,  0.0});

    EXPECT_NEAR(alt1 - alt0, dY, dY * 0.01);
}

TEST_F(GravityTest, AltitudePositiveAboveLaunchSurface) {
    Gravity g(LAT_KSC, ALT_SEA_LEVEL);

    EXPECT_GT(g.computeAltitude(Vec3{0.0, 1000.0, 0.0}), 0.0);
}

TEST_F(GravityTest, AltitudeFiniteAtVariousPositions) {
    Gravity g(LAT_KSC, ALT_SEA_LEVEL);
    const std::vector<Vec3> positions = {
        {      0.0,       0.0,     0.0},
        {      0.0,  100000.0,     0.0},
        {  50000.0,   50000.0,     0.0},
        {      0.0,  500000.0, 10000.0},
    };

    for (const auto& pos : positions) {
        double alt = g.computeAltitude(pos);
        EXPECT_TRUE(std::isfinite(alt))
            << "pos=(" << pos.x << "," << pos.y << "," << pos.z << ")";
    }
}

// =============================================================================
// Physical Consistency
// =============================================================================

TEST_F(GravityTest, GravityDecreasesConcurrentlyWithAltitudeIncrease) {
    // Higher altitude <=> weaker gravity. Both metrics should be consistent.
    Gravity g(LAT_MID, ALT_SEA_LEVEL);
    Vec3 pos0{0.0,       0.0, 0.0};
    Vec3 pos1{0.0, 200000.0, 0.0};

    EXPECT_LT(g.computeAltitude(pos0), g.computeAltitude(pos1));
    EXPECT_GT(
        g.computeAcceleration(pos0).mag(),
        g.computeAcceleration(pos1).mag()
    );
}

TEST_F(GravityTest, AzimuthRotationPreservesGravityMagnitudeAtOrigin) {
    // Azimuth only rotates the local horizontal frame.
    // The gravity magnitude at the origin must not change.
    const std::vector<double> azimuths = {0.0, std::numbers::pi / 4.0, std::numbers::pi / 2.0, std::numbers::pi};

    Gravity gRef(LAT_KSC, ALT_SEA_LEVEL, 0.0);
    double magRef = gRef.computeAcceleration(Vec3{0.0, 0.0, 0.0}).mag();

    for (double az : azimuths) {
        Gravity g(LAT_KSC, ALT_SEA_LEVEL, az);
        double mag = g.computeAcceleration(Vec3{0.0, 0.0, 0.0}).mag();

        EXPECT_NEAR(mag, magRef, 1e-6) << "azimuth=" << az;
    }
}

TEST_F(GravityTest, AzimuthRotationPreservesAltitudeAtOrigin) {
    const std::vector<double> azimuths = {0.0, std::numbers::pi / 4.0, std::numbers::pi / 2.0, std::numbers::pi};

    for (double az : azimuths) {
        Gravity g(LAT_KSC, ALT_SEA_LEVEL, az);
        double alt = g.computeAltitude(Vec3{0.0, 0.0, 0.0});

        EXPECT_NEAR(alt, ALT_SEA_LEVEL, RADIUS_TOL) << "azimuth=" << az;
    }
}

// =============================================================================
// EarthConstants Sanity Checks
// =============================================================================

TEST(EarthConstantsTest, J2IsPositive) {
    EXPECT_GT(EarthConstants::J2, 0.0);
}

TEST(EarthConstantsTest, JEqualsThreeHalvesJ2) {
    // J is defined as (3/2)*J2 in configs.hpp.
    EXPECT_NEAR(EarthConstants::J, 1.5 * EarthConstants::J2, 1e-15);
}

TEST(EarthConstantsTest, EquatorialRadiusGreaterThanSemiMinorAxis) {
    EXPECT_GT(EarthConstants::RADIUS_OF_EQUATOR, EarthConstants::SEMI_MINOR_AXIS);
}

TEST(EarthConstantsTest, FlatteningMatchesWGS84) {
    // WGS84: f = 1 / 298.257223563
    EXPECT_NEAR(EarthConstants::FLATTENING, 1.0 / 298.257223563, 1e-15);
}

TEST(EarthConstantsTest, GravitationalConstReasonable) {
    // μ = 3.986005e14 m³/s²
    EXPECT_NEAR(EarthConstants::GRAVITATIONAL_CONST, 3.986005e14, 1e9);
}

TEST(EarthConstantsTest, SecondEccentricityPositive) {
    EXPECT_GT(EarthConstants::SECOND_ECCENTRICITY, 0.0);
}

TEST(EarthConstantsTest, SemiMinorAxisDerivedFromFlattening) {
    // b = Re * (1 - f)
    double expected = EarthConstants::RADIUS_OF_EQUATOR * (1.0 - EarthConstants::FLATTENING);
    EXPECT_NEAR(EarthConstants::SEMI_MINOR_AXIS, expected, 1.0);
}

}  // namespace trajsim::test
