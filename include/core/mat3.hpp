#pragma once
// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Vincent Ning

/// @file mat3.hpp
/// @brief 3x3 matrix type for aerospace computations
///
/// Row-major storage: data[row][column].
/// Matrix-vector multiplication: v' = M * v (column vector convention).
///
/// ## Rotation Convention
/// Rotation matrices use passive (alias) convention:
/// - Positive angle = clockwise rotation when looking along the axis
/// - R * v transforms vector v from the rotated frame to the original frame
///
/// ## Usage
/// @code
///     Mat3 R = Mat3::rotation_z(0.5);
///     Vec3 v{1, 0, 0};
///     Vec3 rotated = R * v;
/// @endcode
///
/// ## Thread Safety
/// No shared state. Operations on separate instances are thread-safe.
///
/// @author Vincent Ning
/// @date 2025-12-06

#include <cassert>
#include <cmath>
#include <iostream>
#include "vec3.hpp"

namespace trajsim {

struct Mat3 {
    double data[3][3];

    // =========================================================================
    // Construction
    // =========================================================================

    /// @brief Default constructor. Initializes to zero matrix.
    constexpr Mat3() noexcept : data{{0, 0, 0}, {0, 0, 0}, {0, 0, 0}} {}

    /// @brief Construct from individual elements (row-major order).
    constexpr Mat3(double m00, double m01, double m02,
                   double m10, double m11, double m12,
                   double m20, double m21, double m22) noexcept
        : data{{m00, m01, m02},
               {m10, m11, m12},
               {m20, m21, m22}} {}

    // =========================================================================
    // Element Access
    // =========================================================================

    /// @brief Access element by row and column index.
    [[nodiscard]] constexpr double& operator()(int i, int j) noexcept {
        assert(i >= 0 && i < 3 && j >= 0 && j < 3 && "Mat3 index out of range");
        return data[i][j];
    }

    /// @brief Access element by row and column index (const).
    [[nodiscard]] constexpr double operator()(int i, int j) const noexcept {
        assert(i >= 0 && i < 3 && j >= 0 && j < 3 && "Mat3 index out of range");
        return data[i][j];
    }

    /// @brief Get row as Vec3.
    [[nodiscard]] constexpr Vec3 row(int i) const noexcept {
        assert(i >= 0 && i < 3 && "Mat3 row index out of range");
        return Vec3{data[i][0], data[i][1], data[i][2]};
    }

    /// @brief Get column as Vec3.
    [[nodiscard]] constexpr Vec3 col(int j) const noexcept {
        assert(j >= 0 && j < 3 && "Mat3 column index out of range");
        return Vec3{data[0][j], data[1][j], data[2][j]};
    }

    // =========================================================================
    // Static Factory Methods
    // =========================================================================

    /// @brief Returns the zero matrix.
    [[nodiscard]] static constexpr Mat3 zero() noexcept {
        return Mat3{};
    }

    /// @brief Returns the identity matrix.
    [[nodiscard]] static constexpr Mat3 identity() noexcept {
        return Mat3{
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0
        };
    }

    /// @brief Rotation matrix about X-axis (roll).
    /// @param angle Angle in radians
    /// @note Uses passive convention
    [[nodiscard]] static Mat3 rotation_x(double angle) noexcept {
        const double c = std::cos(angle);
        const double s = std::sin(angle);
        return Mat3{
            1.0, 0.0, 0.0,
            0.0,   c,   s,
            0.0,  -s,   c
        };
    }

    /// @brief Rotation matrix about Y-axis (pitch).
    /// @param angle Angle in radians
    /// @note Uses passive convention
    [[nodiscard]] static Mat3 rotation_y(double angle) noexcept {
        const double c = std::cos(angle);
        const double s = std::sin(angle);
        return Mat3{
              c, 0.0,  -s,
            0.0, 1.0, 0.0,
              s, 0.0,   c
        };
    }

    /// @brief Rotation matrix about Z-axis (yaw).
    /// @param angle Angle in radians
    /// @note Uses passive convention
    [[nodiscard]] static Mat3 rotation_z(double angle) noexcept {
        const double c = std::cos(angle);
        const double s = std::sin(angle);
        return Mat3{
              c,   s, 0.0,
             -s,   c, 0.0,
            0.0, 0.0, 1.0
        };
    }

    /// @brief Create diagonal matrix.
    [[nodiscard]] static constexpr Mat3 diagonal(double d0, double d1, double d2) noexcept {
        return Mat3{
             d0, 0.0, 0.0,
            0.0,  d1, 0.0,
            0.0, 0.0,  d2
        };
    }

    /// @brief Create skew-symmetric matrix from vector.
    /// Satisfies: skew(v) * u = v × u
    [[nodiscard]] static constexpr Mat3 skew(const Vec3& v) noexcept {
        return Mat3{
             0.0, -v.z,  v.y,
             v.z,  0.0, -v.x,
            -v.y,  v.x,  0.0
        };
    }

    // =========================================================================
    // Arithmetic Operators
    // =========================================================================

    /// @brief Matrix-vector multiplication.
    [[nodiscard]] constexpr Vec3 operator*(const Vec3& v) const noexcept {
        return Vec3{
            data[0][0] * v.x + data[0][1] * v.y + data[0][2] * v.z,
            data[1][0] * v.x + data[1][1] * v.y + data[1][2] * v.z,
            data[2][0] * v.x + data[2][1] * v.y + data[2][2] * v.z
        };
    }

    /// @brief Matrix-matrix multiplication.
    [[nodiscard]] constexpr Mat3 operator*(const Mat3& o) const noexcept {
        Mat3 result{};
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                for (int k = 0; k < 3; ++k) {
                    result.data[i][j] += data[i][k] * o.data[k][j];
                }
            }
        }
        return result;
    }

    /// @brief Matrix addition.
    [[nodiscard]] constexpr Mat3 operator+(const Mat3& o) const noexcept {
        return Mat3{
            data[0][0] + o.data[0][0], data[0][1] + o.data[0][1], data[0][2] + o.data[0][2],
            data[1][0] + o.data[1][0], data[1][1] + o.data[1][1], data[1][2] + o.data[1][2],
            data[2][0] + o.data[2][0], data[2][1] + o.data[2][1], data[2][2] + o.data[2][2]
        };
    }

    /// @brief Matrix subtraction.
    [[nodiscard]] constexpr Mat3 operator-(const Mat3& o) const noexcept {
        return Mat3{
            data[0][0] - o.data[0][0], data[0][1] - o.data[0][1], data[0][2] - o.data[0][2],
            data[1][0] - o.data[1][0], data[1][1] - o.data[1][1], data[1][2] - o.data[1][2],
            data[2][0] - o.data[2][0], data[2][1] - o.data[2][1], data[2][2] - o.data[2][2]
        };
    }

    /// @brief Scalar multiplication.
    [[nodiscard]] constexpr Mat3 operator*(double s) const noexcept {
        return Mat3{
            data[0][0]*s, data[0][1]*s, data[0][2]*s,
            data[1][0]*s, data[1][1]*s, data[1][2]*s,
            data[2][0]*s, data[2][1]*s, data[2][2]*s
        };
    }

    /// @brief Scalar division.
    [[nodiscard]] constexpr Mat3 operator/(double s) const noexcept {
        assert(s != 0.0 && "Mat3 division by zero");
        return *this * (1.0 / s);
    }

    /// @brief Unary negation.
    [[nodiscard]] constexpr Mat3 operator-() const noexcept {
        return *this * -1.0;
    }

    // =========================================================================
    // Compound Assignment Operators
    // =========================================================================

    /// @brief Add and assign.
    constexpr Mat3& operator+=(const Mat3& o) noexcept {
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                data[i][j] += o.data[i][j];
        return *this;
    }

    /// @brief Subtract and assign.
    constexpr Mat3& operator-=(const Mat3& o) noexcept {
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                data[i][j] -= o.data[i][j];
        return *this;
    }

    /// @brief Multiply by scalar and assign.
    constexpr Mat3& operator*=(double s) noexcept {
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                data[i][j] *= s;
        return *this;
    }

    /// @brief Divide by scalar and assign.
    constexpr Mat3& operator/=(double s) noexcept {
        assert(s != 0.0 && "Mat3 division by zero");
        return *this *= (1.0 / s);
    }

    /// @brief Multiply by matrix and assign.
    constexpr Mat3& operator*=(const Mat3& o) noexcept {
        *this = *this * o;
        return *this;
    }

    // =========================================================================
    // Matrix Operations
    // =========================================================================

    /// @brief Transpose matrix.
    [[nodiscard]] constexpr Mat3 transposed() const noexcept {
        return Mat3{
            data[0][0], data[1][0], data[2][0],
            data[0][1], data[1][1], data[2][1],
            data[0][2], data[1][2], data[2][2]
        };
    }

    /// @brief Compute determinant.
    [[nodiscard]] constexpr double det() const noexcept {
        return data[0][0] * (data[1][1]*data[2][2] - data[1][2]*data[2][1])
             - data[0][1] * (data[1][0]*data[2][2] - data[1][2]*data[2][0])
             + data[0][2] * (data[1][0]*data[2][1] - data[1][1]*data[2][0]);
    }

    /// @brief Compute trace (sum of diagonal).
    [[nodiscard]] constexpr double trace() const noexcept {
        return data[0][0] + data[1][1] + data[2][2];
    }

    /// @brief Compute matrix inverse.
    /// @pre Matrix must be non-singular (det != 0)
    [[nodiscard]] Mat3 inverse() const noexcept {
        // Cofactors of first row (reused for determinant)
        const double c00 = data[1][1]*data[2][2] - data[1][2]*data[2][1];
        const double c01 = data[1][2]*data[2][0] - data[1][0]*data[2][2];
        const double c02 = data[1][0]*data[2][1] - data[1][1]*data[2][0];
        
        // Determinant via cofactor expansion along first row
        const double d = data[0][0]*c00 + data[0][1]*c01 + data[0][2]*c02;
        assert(std::abs(d) > 1e-15 && "Cannot invert singular matrix");
        
        const double inv_d = 1.0 / d;
        
        // Remaining cofactors (for adjugate matrix)
        const double c10 = data[0][2]*data[2][1] - data[0][1]*data[2][2];
        const double c11 = data[0][0]*data[2][2] - data[0][2]*data[2][0];
        const double c12 = data[0][1]*data[2][0] - data[0][0]*data[2][1];
        const double c20 = data[0][1]*data[1][2] - data[0][2]*data[1][1];
        const double c21 = data[0][2]*data[1][0] - data[0][0]*data[1][2];
        const double c22 = data[0][0]*data[1][1] - data[0][1]*data[1][0];
        
        // Inverse = adjugate / det = cofactor^T / det
        return Mat3{
            c00 * inv_d, c10 * inv_d, c20 * inv_d,
            c01 * inv_d, c11 * inv_d, c21 * inv_d,
            c02 * inv_d, c12 * inv_d, c22 * inv_d
        };
    }

    // =========================================================================
    // Validation Methods
    // =========================================================================

    /// @brief Check if any element is NaN.
    [[nodiscard]] bool has_nan() const noexcept {
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                if (std::isnan(data[i][j])) return true;
        return false;
    }

    /// @brief Check if any element is infinite.
    [[nodiscard]] bool has_inf() const noexcept {
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                if (std::isinf(data[i][j])) return true;
        return false;
    }

    /// @brief Check if all elements are finite.
    [[nodiscard]] bool is_finite() const noexcept {
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                if (!std::isfinite(data[i][j])) return false;
        return true;
    }

    /// @brief Check if matrix is orthogonal (R^T * R ≈ I).
    [[nodiscard]] bool is_orthogonal(double eps = 1e-10) const noexcept {
        Mat3 check = transposed() * (*this);
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                double expected = (i == j) ? 1.0 : 0.0;
                if (std::abs(check.data[i][j] - expected) > eps)
                    return false;
            }
        }
        return true;
    }

    /// @brief Check if matrix is a proper rotation (orthogonal with det = +1).
    [[nodiscard]] bool is_rotation(double eps = 1e-10) const noexcept {
        return is_orthogonal(eps) && std::abs(det() - 1.0) < eps;
    }

    /// @brief Check if matrix is symmetric (M = M^T).
    [[nodiscard]] bool is_symmetric(double eps = 1e-10) const noexcept {
        return std::abs(data[0][1] - data[1][0]) < eps &&
               std::abs(data[0][2] - data[2][0]) < eps &&
               std::abs(data[1][2] - data[2][1]) < eps;
    }

    /// @brief Check if matrix is diagonal.
    [[nodiscard]] bool is_diagonal(double eps = 1e-10) const noexcept {
        return std::abs(data[0][1]) < eps && std::abs(data[0][2]) < eps &&
               std::abs(data[1][0]) < eps && std::abs(data[1][2]) < eps &&
               std::abs(data[2][0]) < eps && std::abs(data[2][1]) < eps;
    }
};

// =============================================================================
// Free Functions
// =============================================================================

/// @brief Scalar * Matrix multiplication.
[[nodiscard]] inline constexpr Mat3 operator*(double s, const Mat3& m) noexcept {
    return m * s;
}

/// @brief Approximate equality with tolerance.
[[nodiscard]] inline bool near_equal(const Mat3& a, const Mat3& b, 
                                      double eps = 1e-10) noexcept {
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            if (std::abs(a(i, j) - b(i, j)) > eps)
                return false;
    return true;
}

inline std::ostream& operator<<(std::ostream& os, const Mat3& m) {
    os << "[" << m(0,0) << ", " << m(0,1) << ", " << m(0,2) << "]\n"
       << "[" << m(1,0) << ", " << m(1,1) << ", " << m(1,2) << "]\n"
       << "[" << m(2,0) << ", " << m(2,1) << ", " << m(2,2) << "]";
    return os;
}

}  // namespace trajsim