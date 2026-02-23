// SPDX-License-Identifier: MIT
// Copyright (c) 2024 Vincent Ning

/// @file test_vec3.cpp
/// @brief Unit tests for Vec3

#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include "core/vec3.hpp"

namespace trajsim {
namespace {

constexpr double EPS = 1e-10;
constexpr double PI = 3.14159265358979323846;

// =============================================================================
// Construction Tests
// =============================================================================

TEST(Vec3Test, DefaultConstructorIsZero) {
    Vec3 v;
    EXPECT_DOUBLE_EQ(v.x, 0.0);
    EXPECT_DOUBLE_EQ(v.y, 0.0);
    EXPECT_DOUBLE_EQ(v.z, 0.0);
}

TEST(Vec3Test, ComponentConstructor) {
    Vec3 v{1.0, 2.0, 3.0};
    EXPECT_DOUBLE_EQ(v.x, 1.0);
    EXPECT_DOUBLE_EQ(v.y, 2.0);
    EXPECT_DOUBLE_EQ(v.z, 3.0);
}

TEST(Vec3Test, CopyConstructor) {
    Vec3 a{1.0, 2.0, 3.0};
    Vec3 b = a;
    EXPECT_DOUBLE_EQ(b.x, 1.0);
    EXPECT_DOUBLE_EQ(b.y, 2.0);
    EXPECT_DOUBLE_EQ(b.z, 3.0);
}

TEST(Vec3Test, CopyAssignment) {
    Vec3 a{1.0, 2.0, 3.0};
    Vec3 b;
    b = a;
    EXPECT_DOUBLE_EQ(b.x, 1.0);
    EXPECT_DOUBLE_EQ(b.y, 2.0);
    EXPECT_DOUBLE_EQ(b.z, 3.0);
}

// =============================================================================
// Static Factory Tests
// =============================================================================

TEST(Vec3Test, ZeroFactory) {
    Vec3 v = Vec3::zero();
    EXPECT_DOUBLE_EQ(v.x, 0.0);
    EXPECT_DOUBLE_EQ(v.y, 0.0);
    EXPECT_DOUBLE_EQ(v.z, 0.0);
}

TEST(Vec3Test, UnitXFactory) {
    Vec3 v = Vec3::unit_x();
    EXPECT_DOUBLE_EQ(v.x, 1.0);
    EXPECT_DOUBLE_EQ(v.y, 0.0);
    EXPECT_DOUBLE_EQ(v.z, 0.0);
    EXPECT_DOUBLE_EQ(v.mag(), 1.0);
}

TEST(Vec3Test, UnitYFactory) {
    Vec3 v = Vec3::unit_y();
    EXPECT_DOUBLE_EQ(v.x, 0.0);
    EXPECT_DOUBLE_EQ(v.y, 1.0);
    EXPECT_DOUBLE_EQ(v.z, 0.0);
    EXPECT_DOUBLE_EQ(v.mag(), 1.0);
}

TEST(Vec3Test, UnitZFactory) {
    Vec3 v = Vec3::unit_z();
    EXPECT_DOUBLE_EQ(v.x, 0.0);
    EXPECT_DOUBLE_EQ(v.y, 0.0);
    EXPECT_DOUBLE_EQ(v.z, 1.0);
    EXPECT_DOUBLE_EQ(v.mag(), 1.0);
}

// =============================================================================
// Element Access Tests
// =============================================================================

TEST(Vec3Test, IndexedAccessRead) {
    Vec3 v{1.0, 2.0, 3.0};
    EXPECT_DOUBLE_EQ(v[0], 1.0);
    EXPECT_DOUBLE_EQ(v[1], 2.0);
    EXPECT_DOUBLE_EQ(v[2], 3.0);
}

TEST(Vec3Test, IndexedAccessWrite) {
    Vec3 v;
    v[0] = 1.0;
    v[1] = 2.0;
    v[2] = 3.0;
    EXPECT_DOUBLE_EQ(v.x, 1.0);
    EXPECT_DOUBLE_EQ(v.y, 2.0);
    EXPECT_DOUBLE_EQ(v.z, 3.0);
}

TEST(Vec3Test, IndexedAccessConst) {
    const Vec3 v{1.0, 2.0, 3.0};
    EXPECT_DOUBLE_EQ(v[0], 1.0);
    EXPECT_DOUBLE_EQ(v[1], 2.0);
    EXPECT_DOUBLE_EQ(v[2], 3.0);
}

// =============================================================================
// Arithmetic Operator Tests
// =============================================================================

TEST(Vec3Test, Addition) {
    Vec3 a{1.0, 2.0, 3.0};
    Vec3 b{4.0, 5.0, 6.0};
    Vec3 c = a + b;
    EXPECT_DOUBLE_EQ(c.x, 5.0);
    EXPECT_DOUBLE_EQ(c.y, 7.0);
    EXPECT_DOUBLE_EQ(c.z, 9.0);
}

TEST(Vec3Test, Subtraction) {
    Vec3 a{4.0, 5.0, 6.0};
    Vec3 b{1.0, 2.0, 3.0};
    Vec3 c = a - b;
    EXPECT_DOUBLE_EQ(c.x, 3.0);
    EXPECT_DOUBLE_EQ(c.y, 3.0);
    EXPECT_DOUBLE_EQ(c.z, 3.0);
}

TEST(Vec3Test, ScalarMultiplication) {
    Vec3 v{1.0, 2.0, 3.0};
    Vec3 r = v * 2.0;
    EXPECT_DOUBLE_EQ(r.x, 2.0);
    EXPECT_DOUBLE_EQ(r.y, 4.0);
    EXPECT_DOUBLE_EQ(r.z, 6.0);
}

TEST(Vec3Test, ScalarMultiplicationReverse) {
    Vec3 v{1.0, 2.0, 3.0};
    Vec3 r = 2.0 * v;
    EXPECT_DOUBLE_EQ(r.x, 2.0);
    EXPECT_DOUBLE_EQ(r.y, 4.0);
    EXPECT_DOUBLE_EQ(r.z, 6.0);
}

TEST(Vec3Test, ScalarDivision) {
    Vec3 v{2.0, 4.0, 6.0};
    Vec3 r = v / 2.0;
    EXPECT_DOUBLE_EQ(r.x, 1.0);
    EXPECT_DOUBLE_EQ(r.y, 2.0);
    EXPECT_DOUBLE_EQ(r.z, 3.0);
}

TEST(Vec3Test, Negation) {
    Vec3 v{1.0, -2.0, 3.0};
    Vec3 r = -v;
    EXPECT_DOUBLE_EQ(r.x, -1.0);
    EXPECT_DOUBLE_EQ(r.y, 2.0);
    EXPECT_DOUBLE_EQ(r.z, -3.0);
}

TEST(Vec3Test, DoubleNegationIsIdentity) {
    Vec3 v{1.0, 2.0, 3.0};
    Vec3 r = -(-v);
    EXPECT_DOUBLE_EQ(r.x, v.x);
    EXPECT_DOUBLE_EQ(r.y, v.y);
    EXPECT_DOUBLE_EQ(r.z, v.z);
}

// =============================================================================
// Compound Assignment Tests
// =============================================================================

TEST(Vec3Test, AddAssign) {
    Vec3 v{1.0, 2.0, 3.0};
    v += Vec3{1.0, 1.0, 1.0};
    EXPECT_DOUBLE_EQ(v.x, 2.0);
    EXPECT_DOUBLE_EQ(v.y, 3.0);
    EXPECT_DOUBLE_EQ(v.z, 4.0);
}

TEST(Vec3Test, ChainedAssignment) {
    Vec3 a{1.0, 1.0, 1.0};
    Vec3 b{1.0, 1.0, 1.0};
    Vec3 c{1.0, 1.0, 1.0};
    
    // (a += b) should return a reference to a, which is then added to c
    (c += a) += b; 
    
    EXPECT_DOUBLE_EQ(c.x, 3.0); // 1 + 1 + 1
    EXPECT_DOUBLE_EQ(c.y, 3.0);
    EXPECT_DOUBLE_EQ(c.z, 3.0);
}

TEST(Vec3Test, SubtractAssign) {
    Vec3 v{2.0, 3.0, 4.0};
    v -= Vec3{1.0, 1.0, 1.0};
    EXPECT_DOUBLE_EQ(v.x, 1.0);
    EXPECT_DOUBLE_EQ(v.y, 2.0);
    EXPECT_DOUBLE_EQ(v.z, 3.0);
}

TEST(Vec3Test, MultiplyAssign) {
    Vec3 v{1.0, 2.0, 3.0};
    v *= 2.0;
    EXPECT_DOUBLE_EQ(v.x, 2.0);
    EXPECT_DOUBLE_EQ(v.y, 4.0);
    EXPECT_DOUBLE_EQ(v.z, 6.0);
}

TEST(Vec3Test, DivideAssign) {
    Vec3 v{2.0, 4.0, 6.0};
    v /= 2.0;
    EXPECT_DOUBLE_EQ(v.x, 1.0);
    EXPECT_DOUBLE_EQ(v.y, 2.0);
    EXPECT_DOUBLE_EQ(v.z, 3.0);
}

// =============================================================================
// Comparison Tests
// =============================================================================

TEST(Vec3Test, EqualityTrue) {
    Vec3 a{1.0, 2.0, 3.0};
    Vec3 b{1.0, 2.0, 3.0};
    EXPECT_TRUE(a == b);
    EXPECT_FALSE(a != b);
}

TEST(Vec3Test, EqualityFalse) {
    Vec3 a{1.0, 2.0, 3.0};
    Vec3 b{1.0, 2.0, 3.001};
    EXPECT_FALSE(a == b);
    EXPECT_TRUE(a != b);
}

TEST(Vec3Test, NearEqualWithinTolerance) {
    Vec3 a{1.0, 2.0, 3.0};
    Vec3 b{1.0 + 1e-11, 2.0, 3.0};
    EXPECT_TRUE(near_equal(a, b, 1e-10));
}

TEST(Vec3Test, NearEqualOutsideTolerance) {
    Vec3 a{1.0, 2.0, 3.0};
    Vec3 b{1.1, 2.0, 3.0};
    EXPECT_FALSE(near_equal(a, b, 1e-10));
}

// =============================================================================
// Magnitude Tests
// =============================================================================

TEST(Vec3Test, MagnitudeZeroVector) {
    Vec3 v;
    EXPECT_DOUBLE_EQ(v.mag(), 0.0);
    EXPECT_DOUBLE_EQ(v.mag_sq(), 0.0);
}

TEST(Vec3Test, MagnitudeUnitVectors) {
    EXPECT_DOUBLE_EQ(Vec3::unit_x().mag(), 1.0);
    EXPECT_DOUBLE_EQ(Vec3::unit_y().mag(), 1.0);
    EXPECT_DOUBLE_EQ(Vec3::unit_z().mag(), 1.0);
}

TEST(Vec3Test, MagnitudePythagorean) {
    Vec3 v{3.0, 4.0, 0.0};
    EXPECT_DOUBLE_EQ(v.mag(), 5.0);
    EXPECT_DOUBLE_EQ(v.mag_sq(), 25.0);
}

TEST(Vec3Test, MagnitudeSquaredAvoidsSqrt) {
    // mag_sq should be significantly faster for comparisons
    Vec3 v{1.0, 2.0, 3.0};
    double expected = 1.0 + 4.0 + 9.0;
    EXPECT_DOUBLE_EQ(v.mag_sq(), expected);
}

// =============================================================================
// Normalization Tests
// =============================================================================

TEST(Vec3Test, NormalizedUnitMagnitude) {
    Vec3 v{3.0, 4.0, 0.0};
    Vec3 n = v.normalized();
    EXPECT_NEAR(n.mag(), 1.0, EPS);
}

TEST(Vec3Test, NormalizedPreservesDirection) {
    Vec3 v{3.0, 4.0, 0.0};
    Vec3 n = v.normalized();
    EXPECT_DOUBLE_EQ(n.x, 0.6);
    EXPECT_DOUBLE_EQ(n.y, 0.8);
    EXPECT_DOUBLE_EQ(n.z, 0.0);
}

TEST(Vec3Test, NormalizedZeroVectorReturnsZero) {
    Vec3 v;
    Vec3 n = v.normalized();
    EXPECT_DOUBLE_EQ(n.x, 0.0);
    EXPECT_DOUBLE_EQ(n.y, 0.0);
    EXPECT_DOUBLE_EQ(n.z, 0.0);
}

TEST(Vec3Test, NormalizedVerySmallVectorReturnsZero) {
    Vec3 v{1e-320, 0.0, 0.0};
    Vec3 n = v.normalized();
    EXPECT_TRUE(n.is_zero());
}

// =============================================================================
// Utility Method Tests
// =============================================================================

TEST(Vec3Test, IsZeroTrue) {
    Vec3 v;
    EXPECT_TRUE(v.is_zero());
}

TEST(Vec3Test, IsZeroFalse) {
    Vec3 v{0.1, 0.0, 0.0};
    EXPECT_FALSE(v.is_zero());
}

TEST(Vec3Test, IsZeroWithTolerance) {
    Vec3 v{1e-11, 0.0, 0.0};
    EXPECT_TRUE(v.is_zero(1e-10));
    EXPECT_FALSE(v.is_zero(1e-12));
}

TEST(Vec3Test, HasNaN) {
    Vec3 normal{1.0, 2.0, 3.0};
    EXPECT_FALSE(normal.has_nan());

    Vec3 nan_x{std::nan(""), 0.0, 0.0};
    EXPECT_TRUE(nan_x.has_nan());

    Vec3 nan_y{0.0, std::nan(""), 0.0};
    EXPECT_TRUE(nan_y.has_nan());

    Vec3 nan_z{0.0, 0.0, std::nan("")};
    EXPECT_TRUE(nan_z.has_nan());
}

TEST(Vec3Test, HasInf) {
    Vec3 normal{1.0, 2.0, 3.0};
    EXPECT_FALSE(normal.has_inf());

    double inf = std::numeric_limits<double>::infinity();
    Vec3 inf_x{inf, 0.0, 0.0};
    EXPECT_TRUE(inf_x.has_inf());

    Vec3 neg_inf{-inf, 0.0, 0.0};
    EXPECT_TRUE(neg_inf.has_inf());
}

TEST(Vec3Test, IsFinite) {
    Vec3 normal{1.0, 2.0, 3.0};
    EXPECT_TRUE(normal.is_finite());

    Vec3 nan_vec{std::nan(""), 0.0, 0.0};
    EXPECT_FALSE(nan_vec.is_finite());

    Vec3 inf_vec{std::numeric_limits<double>::infinity(), 0.0, 0.0};
    EXPECT_FALSE(inf_vec.is_finite());
}

// =============================================================================
// Dot Product Tests
// =============================================================================

TEST(Vec3Test, DotProductPerpendicular) {
    Vec3 a = Vec3::unit_x();
    Vec3 b = Vec3::unit_y();
    EXPECT_DOUBLE_EQ(dot(a, b), 0.0);
}

TEST(Vec3Test, DotProductParallel) {
    Vec3 a{1.0, 0.0, 0.0};
    Vec3 b{2.0, 0.0, 0.0};
    EXPECT_DOUBLE_EQ(dot(a, b), 2.0);
}

TEST(Vec3Test, DotProductAntiParallel) {
    Vec3 a{1.0, 0.0, 0.0};
    Vec3 b{-1.0, 0.0, 0.0};
    EXPECT_DOUBLE_EQ(dot(a, b), -1.0);
}

TEST(Vec3Test, DotProductGeneral) {
    Vec3 a{1.0, 2.0, 3.0};
    Vec3 b{4.0, 5.0, 6.0};
    EXPECT_DOUBLE_EQ(dot(a, b), 32.0);  // 4 + 10 + 18
}

TEST(Vec3Test, DotProductCommutative) {
    Vec3 a{1.0, 2.0, 3.0};
    Vec3 b{4.0, 5.0, 6.0};
    EXPECT_DOUBLE_EQ(dot(a, b), dot(b, a));
}

TEST(Vec3Test, DotProductWithSelfIsMagSquared) {
    Vec3 v{1.0, 2.0, 3.0};
    EXPECT_DOUBLE_EQ(dot(v, v), v.mag_sq());
}

// =============================================================================
// Cross Product Tests
// =============================================================================

TEST(Vec3Test, CrossProductXY) {
    Vec3 x = Vec3::unit_x();
    Vec3 y = Vec3::unit_y();
    Vec3 z = cross(x, y);
    EXPECT_DOUBLE_EQ(z.x, 0.0);
    EXPECT_DOUBLE_EQ(z.y, 0.0);
    EXPECT_DOUBLE_EQ(z.z, 1.0);
}

TEST(Vec3Test, CrossProductYZ) {
    Vec3 y = Vec3::unit_y();
    Vec3 z = Vec3::unit_z();
    Vec3 x = cross(y, z);
    EXPECT_DOUBLE_EQ(x.x, 1.0);
    EXPECT_DOUBLE_EQ(x.y, 0.0);
    EXPECT_DOUBLE_EQ(x.z, 0.0);
}

TEST(Vec3Test, CrossProductZX) {
    Vec3 z = Vec3::unit_z();
    Vec3 x = Vec3::unit_x();
    Vec3 y = cross(z, x);
    EXPECT_DOUBLE_EQ(y.x, 0.0);
    EXPECT_DOUBLE_EQ(y.y, 1.0);
    EXPECT_DOUBLE_EQ(y.z, 0.0);
}

TEST(Vec3Test, CrossProductAntiCommutative) {
    Vec3 a{1.0, 2.0, 3.0};
    Vec3 b{4.0, 5.0, 6.0};
    Vec3 axb = cross(a, b);
    Vec3 bxa = cross(b, a);
    EXPECT_NEAR(axb.x, -bxa.x, EPS);
    EXPECT_NEAR(axb.y, -bxa.y, EPS);
    EXPECT_NEAR(axb.z, -bxa.z, EPS);
}

TEST(Vec3Test, CrossProductWithSelfIsZero) {
    Vec3 v{1.0, 2.0, 3.0};
    Vec3 c = cross(v, v);
    EXPECT_TRUE(c.is_zero());
}

TEST(Vec3Test, CrossProductPerpendicularToInputs) {
    Vec3 a{1.0, 2.0, 3.0};
    Vec3 b{4.0, 5.0, 6.0};
    Vec3 c = cross(a, b);
    EXPECT_NEAR(dot(c, a), 0.0, EPS);
    EXPECT_NEAR(dot(c, b), 0.0, EPS);
}

// =============================================================================
// Angle Between Tests
// =============================================================================

TEST(Vec3Test, AngleBetweenParallel) {
    Vec3 a{1.0, 0.0, 0.0};
    Vec3 b{2.0, 0.0, 0.0};
    EXPECT_NEAR(angle_between(a, b), 0.0, EPS);
}

TEST(Vec3Test, AngleBetweenPerpendicular) {
    Vec3 a = Vec3::unit_x();
    Vec3 b = Vec3::unit_y();
    EXPECT_NEAR(angle_between(a, b), PI / 2.0, EPS);
}

TEST(Vec3Test, AngleBetweenAntiParallel) {
    Vec3 a{1.0, 0.0, 0.0};
    Vec3 b{-1.0, 0.0, 0.0};
    EXPECT_NEAR(angle_between(a, b), PI, EPS);
}

TEST(Vec3Test, AngleBetween45Degrees) {
    Vec3 a{1.0, 0.0, 0.0};
    Vec3 b{1.0, 1.0, 0.0};
    EXPECT_NEAR(angle_between(a, b), PI / 4.0, EPS);
}

// =============================================================================
// Projection Tests
// =============================================================================

TEST(Vec3Test, ProjectOntoParallel) {
    Vec3 a{3.0, 0.0, 0.0};
    Vec3 b{1.0, 0.0, 0.0};
    Vec3 p = project(a, b);
    EXPECT_NEAR(p.x, 3.0, EPS);
    EXPECT_NEAR(p.y, 0.0, EPS);
    EXPECT_NEAR(p.z, 0.0, EPS);
}

TEST(Vec3Test, ProjectOntoPerpendicular) {
    Vec3 a{0.0, 3.0, 0.0};
    Vec3 b{1.0, 0.0, 0.0};
    Vec3 p = project(a, b);
    EXPECT_TRUE(p.is_zero());
}

TEST(Vec3Test, ProjectGeneral) {
    Vec3 a{1.0, 1.0, 0.0};
    Vec3 b{1.0, 0.0, 0.0};
    Vec3 p = project(a, b);
    EXPECT_NEAR(p.x, 1.0, EPS);
    EXPECT_NEAR(p.y, 0.0, EPS);
    EXPECT_NEAR(p.z, 0.0, EPS);
}

// =============================================================================
// Rejection Tests
// =============================================================================

TEST(Vec3Test, RejectFromPerpendicular) {
    Vec3 a{0.0, 3.0, 0.0};
    Vec3 b{1.0, 0.0, 0.0};
    Vec3 r = reject(a, b);
    EXPECT_NEAR(r.x, 0.0, EPS);
    EXPECT_NEAR(r.y, 3.0, EPS);
    EXPECT_NEAR(r.z, 0.0, EPS);
}

TEST(Vec3Test, ProjectPlusRejectEqualsOriginal) {
    Vec3 a{1.0, 2.0, 3.0};
    Vec3 b{4.0, 5.0, 0.0};
    Vec3 p = project(a, b);
    Vec3 r = reject(a, b);
    Vec3 sum = p + r;
    EXPECT_NEAR(sum.x, a.x, EPS);
    EXPECT_NEAR(sum.y, a.y, EPS);
    EXPECT_NEAR(sum.z, a.z, EPS);
}

// =============================================================================
// Linear Interpolation Tests
// =============================================================================

TEST(Vec3Test, LerpAtZero) {
    Vec3 a{0.0, 0.0, 0.0};
    Vec3 b{1.0, 2.0, 3.0};
    Vec3 r = lerp(a, b, 0.0);
    EXPECT_DOUBLE_EQ(r.x, a.x);
    EXPECT_DOUBLE_EQ(r.y, a.y);
    EXPECT_DOUBLE_EQ(r.z, a.z);
}

TEST(Vec3Test, LerpAtOne) {
    Vec3 a{0.0, 0.0, 0.0};
    Vec3 b{1.0, 2.0, 3.0};
    Vec3 r = lerp(a, b, 1.0);
    EXPECT_DOUBLE_EQ(r.x, b.x);
    EXPECT_DOUBLE_EQ(r.y, b.y);
    EXPECT_DOUBLE_EQ(r.z, b.z);
}

TEST(Vec3Test, LerpAtHalf) {
    Vec3 a{0.0, 0.0, 0.0};
    Vec3 b{2.0, 4.0, 6.0};
    Vec3 r = lerp(a, b, 0.5);
    EXPECT_DOUBLE_EQ(r.x, 1.0);
    EXPECT_DOUBLE_EQ(r.y, 2.0);
    EXPECT_DOUBLE_EQ(r.z, 3.0);
}

// =============================================================================
// Edge Case Tests
// =============================================================================

TEST(Vec3Test, VeryLargeValues) {
    double large = 1e100;
    Vec3 v{large, large, large};
    EXPECT_TRUE(v.is_finite());
    EXPECT_FALSE(v.has_inf());
}

TEST(Vec3Test, VerySmallValues) {
    double small = 1e-100;
    Vec3 v{small, small, small};
    EXPECT_TRUE(v.is_finite());
    EXPECT_GT(v.mag(), 0.0);
}

TEST(Vec3Test, MixedSignValues) {
    Vec3 v{-1.0, 2.0, -3.0};
    EXPECT_DOUBLE_EQ(v.mag_sq(), 14.0);
}

}  // namespace
}  // namespace trajsim