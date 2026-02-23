// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Vincent Ning

/// @file test_mat3.cpp
/// @brief Unit tests for Mat3
/// @author Vincent Ning
/// @date 2025-12-06

#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include "core/mat3.hpp"

namespace trajsim {
namespace { //anonymous namespace for access restriction 

constexpr double EPS = 1e-10;
constexpr double PI = 3.14159265358979323846;

// =============================================================================
// Construction Tests
// =============================================================================

TEST(Mat3Test, DefaultConstructorIsZero) {
    Mat3 m;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            EXPECT_DOUBLE_EQ(m(i, j), 0.0);
}

TEST(Mat3Test, ElementConstructor) {
    Mat3 m{1, 2, 3,
           4, 5, 6,
           7, 8, 9};
    EXPECT_DOUBLE_EQ(m(0, 0), 1);
    EXPECT_DOUBLE_EQ(m(0, 1), 2);
    EXPECT_DOUBLE_EQ(m(0, 2), 3);
    EXPECT_DOUBLE_EQ(m(1, 0), 4);
    EXPECT_DOUBLE_EQ(m(1, 1), 5);
    EXPECT_DOUBLE_EQ(m(1, 2), 6);
    EXPECT_DOUBLE_EQ(m(2, 0), 7);
    EXPECT_DOUBLE_EQ(m(2, 1), 8);
    EXPECT_DOUBLE_EQ(m(2, 2), 9);
}

TEST(Mat3Test, CopyConstructor) {
    Mat3 a{1, 2, 3, 4, 5, 6, 7, 8, 9};
    Mat3 b = a;
    EXPECT_TRUE(near_equal(a, b));
}

TEST(Mat3Test, ParenthesesAccessRead) {
    Mat3 m{1, 2, 3, 4, 5, 6, 7, 8, 9};
    EXPECT_DOUBLE_EQ(m(0, 0), 1);
    EXPECT_DOUBLE_EQ(m(1, 2), 6);
    EXPECT_DOUBLE_EQ(m(2, 1), 8);
}

TEST(Mat3Test, ParenthesesAccessWrite) {
    Mat3 m;
    m(1, 2) = 5.0;
    EXPECT_DOUBLE_EQ(m(1, 2), 5.0);
}

TEST(Mat3Test, ConstAccess) {
    const Mat3 m{1, 2, 3, 4, 5, 6, 7, 8, 9};
    EXPECT_DOUBLE_EQ(m(0, 0), 1);
    EXPECT_DOUBLE_EQ(m(2, 2), 9);
}

TEST(Mat3Test, RowAccess) {
    Mat3 m{1, 2, 3, 4, 5, 6, 7, 8, 9};
    Vec3 r0 = m.row(0);
    Vec3 r1 = m.row(1);
    Vec3 r2 = m.row(2);
    
    EXPECT_TRUE(near_equal(r0, Vec3{1, 2, 3}));
    EXPECT_TRUE(near_equal(r1, Vec3{4, 5, 6}));
    EXPECT_TRUE(near_equal(r2, Vec3{7, 8, 9}));
}

TEST(Mat3Test, ColAccess) {
    Mat3 m{1, 2, 3, 4, 5, 6, 7, 8, 9};
    Vec3 c0 = m.col(0);
    Vec3 c1 = m.col(1);
    Vec3 c2 = m.col(2);
    
    EXPECT_TRUE(near_equal(c0, Vec3{1, 4, 7}));
    EXPECT_TRUE(near_equal(c1, Vec3{2, 5, 8}));
    EXPECT_TRUE(near_equal(c2, Vec3{3, 6, 9}));
}

// =============================================================================
// Static Factory Tests
// =============================================================================

TEST(Mat3Test, ZeroFactory) {
    Mat3 m = Mat3::zero();
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            EXPECT_DOUBLE_EQ(m(i, j), 0.0);
}

TEST(Mat3Test, IdentityFactory) {
    Mat3 I = Mat3::identity();
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            double expected = (i == j) ? 1.0 : 0.0;
            EXPECT_DOUBLE_EQ(I(i, j), expected);
        }
    }
}

TEST(Mat3Test, DiagonalFactory) {
    Mat3 d = Mat3::diagonal(2, 3, 4);
    EXPECT_DOUBLE_EQ(d(0, 0), 2);
    EXPECT_DOUBLE_EQ(d(1, 1), 3);
    EXPECT_DOUBLE_EQ(d(2, 2), 4);
    // Off-diagonal should be zero
    EXPECT_DOUBLE_EQ(d(0, 1), 0);
    EXPECT_DOUBLE_EQ(d(1, 2), 0);
    EXPECT_TRUE(d.is_diagonal());
}

TEST(Mat3Test, SkewFactory) {
    Vec3 v{1, 2, 3};
    Mat3 S = Mat3::skew(v);
    
    // Verify skew-symmetric: S^T = -S
    EXPECT_TRUE(near_equal(S.transposed(), -S));
    
    // Verify S * u = v × u
    Vec3 u{4, 5, 6};
    Vec3 cross_result = cross(v, u);
    Vec3 skew_result = S * u;
    EXPECT_TRUE(near_equal(cross_result, skew_result));
}

// =============================================================================
// Rotation Factory Tests
// =============================================================================

TEST(Mat3Test, RotationXIsRotation) {
    Mat3 R = Mat3::rotation_x(0.5);
    EXPECT_TRUE(R.is_rotation());
}

TEST(Mat3Test, RotationYIsRotation) {
    Mat3 R = Mat3::rotation_y(0.5);
    EXPECT_TRUE(R.is_rotation());
}

TEST(Mat3Test, RotationZIsRotation) {
    Mat3 R = Mat3::rotation_z(0.5);
    EXPECT_TRUE(R.is_rotation());
}

TEST(Mat3Test, RotationXBy90Degrees) {
    Mat3 R = Mat3::rotation_x(PI / 2);
    Vec3 v{0, 1, 0};  // Y-axis
    Vec3 result = R * v;
    // Passive convention: Y -> -Z
    EXPECT_NEAR(result.x, 0, EPS);
    EXPECT_NEAR(result.y, 0, EPS);
    EXPECT_NEAR(result.z, -1, EPS);
}

TEST(Mat3Test, RotationYBy90Degrees) {
    Mat3 R = Mat3::rotation_y(PI / 2);
    Vec3 v{0, 0, 1};  // Z-axis
    Vec3 result = R * v;
    // Passive convention: Z -> -X
    EXPECT_NEAR(result.x, -1, EPS);
    EXPECT_NEAR(result.y, 0, EPS);
    EXPECT_NEAR(result.z, 0, EPS);
}

TEST(Mat3Test, RotationZBy90Degrees) {
    Mat3 R = Mat3::rotation_z(PI / 2);
    Vec3 v{1, 0, 0};  // X-axis
    Vec3 result = R * v;
    // Passive convention: X -> -Y
    EXPECT_NEAR(result.x, 0, EPS);
    EXPECT_NEAR(result.y, -1, EPS);
    EXPECT_NEAR(result.z, 0, EPS);
}

TEST(Mat3Test, RotationBy360DegreesIsIdentity) {
    Mat3 Rx = Mat3::rotation_x(2 * PI);
    Mat3 Ry = Mat3::rotation_y(2 * PI);
    Mat3 Rz = Mat3::rotation_z(2 * PI);
    Mat3 I = Mat3::identity();
    
    EXPECT_TRUE(near_equal(Rx, I, 1e-9));
    EXPECT_TRUE(near_equal(Ry, I, 1e-9));
    EXPECT_TRUE(near_equal(Rz, I, 1e-9));
}

TEST(Mat3Test, ZeroRotationIsIdentity) {
    EXPECT_TRUE(near_equal(Mat3::rotation_x(0), Mat3::identity()));
    EXPECT_TRUE(near_equal(Mat3::rotation_y(0), Mat3::identity()));
    EXPECT_TRUE(near_equal(Mat3::rotation_z(0), Mat3::identity()));
}

TEST(Mat3Test, RotationPreservesVectorLength) {
    Mat3 R = Mat3::rotation_x(0.3) * Mat3::rotation_y(0.5) * Mat3::rotation_z(0.7);
    Vec3 v{3, 4, 5};
    Vec3 result = R * v;
    EXPECT_NEAR(v.mag(), result.mag(), EPS);
}
// =============================================================================
// Arithmetic Operator Tests
// =============================================================================

TEST(Mat3Test, IdentityTimesVector) {
    Mat3 I = Mat3::identity();
    Vec3 v{1, 2, 3};
    Vec3 result = I * v;
    EXPECT_TRUE(near_equal(result, v));
}

TEST(Mat3Test, MatrixTimesVector) {
    Mat3 m{1, 2, 3,
           4, 5, 6,
           7, 8, 9};
    Vec3 v{1, 1, 1};
    Vec3 result = m * v;
    EXPECT_DOUBLE_EQ(result.x, 6);   // 1+2+3
    EXPECT_DOUBLE_EQ(result.y, 15);  // 4+5+6
    EXPECT_DOUBLE_EQ(result.z, 24);  // 7+8+9
}

TEST(Mat3Test, IdentityTimesMatrix) {
    Mat3 I = Mat3::identity();
    Mat3 m{1, 2, 3, 4, 5, 6, 7, 8, 9};
    EXPECT_TRUE(near_equal(I * m, m));
    EXPECT_TRUE(near_equal(m * I, m));
}

TEST(Mat3Test, MatrixMultiplicationAssociative) {
    Mat3 A{1, 2, 0, 0, 1, 1, 2, 0, 1};
    Mat3 B{0, 1, 2, 1, 0, 1, 1, 2, 0};
    Mat3 C{1, 1, 1, 2, 1, 0, 0, 1, 2};
    
    Mat3 AB_C = (A * B) * C;
    Mat3 A_BC = A * (B * C);
    EXPECT_TRUE(near_equal(AB_C, A_BC));
}

TEST(Mat3Test, Addition) {
    Mat3 a{1, 2, 3, 4, 5, 6, 7, 8, 9};
    Mat3 b{9, 8, 7, 6, 5, 4, 3, 2, 1};
    Mat3 c = a + b;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            EXPECT_DOUBLE_EQ(c(i, j), 10.0);
}

TEST(Mat3Test, Subtraction) {
    Mat3 a{1, 2, 3, 4, 5, 6, 7, 8, 9};
    Mat3 b = a - a;
    EXPECT_TRUE(near_equal(b, Mat3::zero()));
}

TEST(Mat3Test, ScalarMultiplication) {
    Mat3 m{1, 2, 3, 4, 5, 6, 7, 8, 9};
    Mat3 result = m * 2.0;
    EXPECT_DOUBLE_EQ(result(0, 0), 2);
    EXPECT_DOUBLE_EQ(result(1, 1), 10);
    EXPECT_DOUBLE_EQ(result(2, 2), 18);
}

TEST(Mat3Test, ScalarMultiplicationCommutative) {
    Mat3 m{1, 2, 3, 4, 5, 6, 7, 8, 9};
    EXPECT_TRUE(near_equal(m * 2.0, 2.0 * m));
}

TEST(Mat3Test, ScalarDivision) {
    Mat3 m{2, 4, 6, 8, 10, 12, 14, 16, 18};
    Mat3 result = m / 2.0;
    EXPECT_DOUBLE_EQ(result(0, 0), 1);
    EXPECT_DOUBLE_EQ(result(2, 2), 9);
}

TEST(Mat3Test, Negation) {
    Mat3 m{1, -2, 3, -4, 5, -6, 7, -8, 9};
    Mat3 neg = -m;
    EXPECT_DOUBLE_EQ(neg(0, 0), -1);
    EXPECT_DOUBLE_EQ(neg(0, 1), 2);
    EXPECT_DOUBLE_EQ(neg(1, 0), 4);
}
// =============================================================================
// Compound Assignment Tests
// =============================================================================

TEST(Mat3Test, AddAssign) {
    Mat3 a{1, 2, 3, 4, 5, 6, 7, 8, 9};
    Mat3 b{1, 1, 1, 1, 1, 1, 1, 1, 1};
    a += b;
    EXPECT_DOUBLE_EQ(a(0, 0), 2);
    EXPECT_DOUBLE_EQ(a(1, 1), 6);
}

TEST(Mat3Test, SubtractAssign) {
    Mat3 a{2, 3, 4, 5, 6, 7, 8, 9, 10};
    Mat3 b{1, 1, 1, 1, 1, 1, 1, 1, 1};
    a -= b;
    EXPECT_DOUBLE_EQ(a(0, 0), 1);
    EXPECT_DOUBLE_EQ(a(2, 2), 9);
}

TEST(Mat3Test, ScalarMultiplyAssign) {
    Mat3 m{1, 2, 3, 4, 5, 6, 7, 8, 9};
    m *= 2.0;
    EXPECT_DOUBLE_EQ(m(0, 0), 2);
    EXPECT_DOUBLE_EQ(m(2, 2), 18);
}

TEST(Mat3Test, ScalarDivideAssign) {
    Mat3 m{2, 4, 6, 8, 10, 12, 14, 16, 18};
    m /= 2.0;
    EXPECT_DOUBLE_EQ(m(0, 0), 1);
    EXPECT_DOUBLE_EQ(m(2, 2), 9);
}

TEST(Mat3Test, MatrixMultiplyAssign) {
    Mat3 R = Mat3::rotation_z(0.3);
    Mat3 R_copy = R;
    R *= R.transposed();
    EXPECT_TRUE(near_equal(R, Mat3::identity()));
}

// =============================================================================
// Validation Tests
// =============================================================================

TEST(Mat3Test, HasNaN) {
    Mat3 normal{1, 2, 3, 4, 5, 6, 7, 8, 9};
    EXPECT_FALSE(normal.has_nan());
    
    Mat3 with_nan{std::nan(""), 2, 3, 4, 5, 6, 7, 8, 9};
    EXPECT_TRUE(with_nan.has_nan());
}

TEST(Mat3Test, HasInf) {
    Mat3 normal{1, 2, 3, 4, 5, 6, 7, 8, 9};
    EXPECT_FALSE(normal.has_inf());
    
    double inf = std::numeric_limits<double>::infinity();
    Mat3 with_inf{inf, 2, 3, 4, 5, 6, 7, 8, 9};
    EXPECT_TRUE(with_inf.has_inf());
}

TEST(Mat3Test, IsFinite) {
    Mat3 normal{1, 2, 3, 4, 5, 6, 7, 8, 9};
    EXPECT_TRUE(normal.is_finite());
    
    Mat3 with_nan{std::nan(""), 2, 3, 4, 5, 6, 7, 8, 9};
    EXPECT_FALSE(with_nan.is_finite());
}

TEST(Mat3Test, IsOrthogonal) {
    Mat3 R = Mat3::rotation_z(0.5);
    EXPECT_TRUE(R.is_orthogonal());
    
    Mat3 not_orthogonal{1, 2, 3, 4, 5, 6, 7, 8, 9};
    EXPECT_FALSE(not_orthogonal.is_orthogonal());
}

TEST(Mat3Test, IsRotation) {
    Mat3 R = Mat3::rotation_x(0.3) * Mat3::rotation_y(0.5);
    EXPECT_TRUE(R.is_rotation());
    
    // Reflection: orthogonal but det = -1
    Mat3 reflection{-1, 0, 0, 0, 1, 0, 0, 0, 1};
    EXPECT_TRUE(reflection.is_orthogonal());
    EXPECT_FALSE(reflection.is_rotation());
}

TEST(Mat3Test, IsSymmetric) {
    Mat3 symmetric{1, 2, 3, 2, 4, 5, 3, 5, 6};
    EXPECT_TRUE(symmetric.is_symmetric());
    
    Mat3 not_symmetric{1, 2, 3, 4, 5, 6, 7, 8, 9};
    EXPECT_FALSE(not_symmetric.is_symmetric());
}

TEST(Mat3Test, IsDiagonal) {
    Mat3 d = Mat3::diagonal(1, 2, 3);
    EXPECT_TRUE(d.is_diagonal());
    
    Mat3 I = Mat3::identity();
    EXPECT_TRUE(I.is_diagonal());
    
    Mat3 not_diagonal{1, 2, 0, 0, 1, 0, 0, 0, 1};
    EXPECT_FALSE(not_diagonal.is_diagonal());
}

// =============================================================================
// Near Equal Tests
// =============================================================================

TEST(Mat3Test, NearEqualIdentical) {
    Mat3 a{1, 2, 3, 4, 5, 6, 7, 8, 9};
    Mat3 b{1, 2, 3, 4, 5, 6, 7, 8, 9};
    EXPECT_TRUE(near_equal(a, b));
}

TEST(Mat3Test, NearEqualWithinTolerance) {
    Mat3 a{1, 2, 3, 4, 5, 6, 7, 8, 9};
    Mat3 b{1 + 1e-11, 2, 3, 4, 5, 6, 7, 8, 9};
    EXPECT_TRUE(near_equal(a, b, 1e-10));
}

TEST(Mat3Test, NearEqualOutsideTolerance) {
    Mat3 a{1, 2, 3, 4, 5, 6, 7, 8, 9};
    Mat3 b{1.1, 2, 3, 4, 5, 6, 7, 8, 9};
    EXPECT_FALSE(near_equal(a, b, 1e-10));
}
}
}