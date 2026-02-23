// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Vincent Ning

/// @file vec3.hpp
/// @brief Three-dimensional vector type for aerospace computations
/// @author Vincent Ning
/// @date 2025

#pragma once

#include <cassert>
#include <cmath>
#include <limits>

namespace trajsim {

/// @brief Three-dimensional vector for position, velocity, force, etc.
///
/// This is the fundamental vector type used throughout TrajSim.
///
/// ## Units Convention
/// Vec3 is unit-agnostic. The calling code is responsible for ensuring
/// consistent units. By convention, SI base units are used:
/// - Position: meters [m]
/// - Velocity: meters per second [m/s]
/// - Force: Newtons [N]
/// - Angles: radians [rad]
///
/// ## Example Usage
/// @code
///     Vec3 position{100.0, 200.0, 300.0};
///     Vec3 velocity{10.0, 0.0, 0.0};
///     
///     Vec3 new_pos = position + velocity * dt;
///     double speed = velocity.mag();
///     Vec3 direction = velocity.normalized();
/// @endcode
///
/// ## Thread Safety
/// Vec3 is trivially copyable and has no shared state. Operations on
/// separate instances are thread-safe. Operations on the same instance
/// require external synchronization.
struct Vec3 {
    double x;  ///< X component
    double y;  ///< Y component
    double z;  ///< Z component

    // =========================================================================
    // Construction
    // =========================================================================

    /// @brief Default constructor. Initializes to zero vector.
    constexpr Vec3() noexcept : x(0.0), y(0.0), z(0.0) {}

    /// @brief Construct from components.
    /// @param x_in X component
    /// @param y_in Y component
    /// @param z_in Z component
    constexpr Vec3(double x_in, double y_in, double z_in) noexcept
        : x(x_in), y(y_in), z(z_in) {}

    // =========================================================================
    // Static Factory Methods
    // =========================================================================

    /// @brief Returns the zero vector (0, 0, 0).
    [[nodiscard]] static constexpr Vec3 zero() noexcept {
        return Vec3{0.0, 0.0, 0.0};
    }

    /// @brief Returns the unit X vector (1, 0, 0).
    [[nodiscard]] static constexpr Vec3 unit_x() noexcept {
        return Vec3{1.0, 0.0, 0.0};
    }

    /// @brief Returns the unit Y vector (0, 1, 0).
    [[nodiscard]] static constexpr Vec3 unit_y() noexcept {
        return Vec3{0.0, 1.0, 0.0};
    }

    /// @brief Returns the unit Z vector (0, 0, 1).
    [[nodiscard]] static constexpr Vec3 unit_z() noexcept {
        return Vec3{0.0, 0.0, 1.0};
    }

    // =========================================================================
    // Element Access
    // =========================================================================

    /// @brief Access element by index (mutable).
    /// @param i Index (0=x, 1=y, 2=z)
    /// @return Reference to the component
    /// @pre i must be in range [0, 2]
    [[nodiscard]] constexpr double& operator[](int i) noexcept {
        assert(i >= 0 && i <= 2 && "Vec3 index out of range");
        if (i == 0) return x;
        if (i == 1) return y;
        return z;
    }

    /// @brief Access element by index (const).
    /// @param i Index (0=x, 1=y, 2=z)
    /// @return Copy of the component value
    /// @pre i must be in range [0, 2]
    [[nodiscard]] constexpr double operator[](int i) const noexcept {
        assert(i >= 0 && i <= 2 && "Vec3 index out of range");
        if (i == 0) return x;
        if (i == 1) return y;
        return z;
    }

    // =========================================================================
    // Arithmetic Operators
    // =========================================================================

    /// @brief Vector addition.
    [[nodiscard]] constexpr Vec3 operator+(const Vec3& o) const noexcept {
        return Vec3{x + o.x, y + o.y, z + o.z};
    }

    /// @brief Vector subtraction.
    [[nodiscard]] constexpr Vec3 operator-(const Vec3& o) const noexcept {
        return Vec3{x - o.x, y - o.y, z - o.z};
    }

    /// @brief Scalar multiplication (Vec3 * scalar).
    [[nodiscard]] constexpr Vec3 operator*(double s) const noexcept {
        return Vec3{x * s, y * s, z * s};
    }

    /// @brief Scalar division.
    /// @param s Scalar divisor (must not be zero)
    /// @pre s != 0
    [[nodiscard]] constexpr Vec3 operator/(double s) const noexcept {
        assert(s != 0.0 && "Vec3 division by zero");
        return Vec3{x / s, y / s, z / s};
    }

    /// @brief Unary negation.
    [[nodiscard]] constexpr Vec3 operator-() const noexcept {
        return Vec3{-x, -y, -z};
    }

    // =========================================================================
    // Compound Assignment Operators
    // =========================================================================

    /// @brief Add and assign.
    constexpr Vec3& operator+=(const Vec3& o) noexcept {
        x += o.x;
        y += o.y;
        z += o.z;
        return *this;
    }

    /// @brief Subtract and assign.
    constexpr Vec3& operator-=(const Vec3& o) noexcept {
        x -= o.x;
        y -= o.y;
        z -= o.z;
        return *this;
    }

    /// @brief Multiply by scalar and assign.
    constexpr Vec3& operator*=(double s) noexcept {
        x *= s;
        y *= s;
        z *= s;
        return *this;
    }

    /// @brief Divide by scalar and assign.
    /// @pre s != 0
    constexpr Vec3& operator/=(double s) noexcept {
        assert(s != 0.0 && "Vec3 division by zero");
        x /= s;
        y /= s;
        z /= s;
        return *this;
    }

    // =========================================================================
    // Comparison Operators
    // =========================================================================

    /// @brief Exact equality comparison.
    /// @warning Use near_equal() for floating-point comparisons in physics code.
    [[nodiscard]] constexpr bool operator==(const Vec3& o) const noexcept {
        return x == o.x && y == o.y && z == o.z;
    }

    /// @brief Exact inequality comparison.
    [[nodiscard]] constexpr bool operator!=(const Vec3& o) const noexcept {
        return !(*this == o);
    }

    // =========================================================================
    // Magnitude Operations
    // =========================================================================

    /// @brief Compute the Euclidean magnitude (L2 norm).
    /// @return ||v|| = sqrt(x² + y² + z²)
    [[nodiscard]] double mag() const noexcept {
        return std::sqrt(x * x + y * y + z * z);
    }

    /// @brief Compute the squared magnitude.
    /// 
    /// Use this instead of mag() when comparing magnitudes, as it avoids
    /// the sqrt computation.
    /// @return x² + y² + z²
    [[nodiscard]] constexpr double mag_sq() const noexcept {
        return x * x + y * y + z * z;
    }

    /// @brief Return a unit vector in the same direction.
    /// 
    /// If the vector is zero (or nearly zero), returns the zero vector
    /// rather than producing NaN or infinity.
    /// @return Unit vector, or zero vector if magnitude is negligible
    [[nodiscard]] Vec3 normalized() const noexcept {
        const double m = mag();
        if (m < std::numeric_limits<double>::epsilon()) {
            return Vec3::zero();
        }
        return *this / m;
    }

    /// @brief Check if this is approximately a zero vector.
    /// @param eps Tolerance for comparison
    /// @return true if magnitude squared is less than eps²
    [[nodiscard]] constexpr bool is_zero(double eps = 1e-10) const noexcept {
        return mag_sq() < eps * eps;
    }

    /// @brief Check if this vector contains any NaN components.
    [[nodiscard]] bool has_nan() const noexcept {
        return std::isnan(x) || std::isnan(y) || std::isnan(z);
    }

    /// @brief Check if this vector contains any infinite components.
    [[nodiscard]] bool has_inf() const noexcept {
        return std::isinf(x) || std::isinf(y) || std::isinf(z);
    }

    /// @brief Check if all components are finite (not NaN or Inf).
    [[nodiscard]] bool is_finite() const noexcept {
        return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
    }
};

// =============================================================================
// Free Functions
// =============================================================================

/// @brief Compute the dot product of two vectors.
/// @param a First vector
/// @param b Second vector
/// @return a · b = ax*bx + ay*by + az*bz
[[nodiscard]] inline constexpr double dot(const Vec3& a, const Vec3& b) noexcept {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

/// @brief Compute the cross product of two vectors.
/// 
/// The result follows the right-hand rule: if fingers curl from a to b,
/// thumb points in direction of a × b.
/// @param a First vector
/// @param b Second vector
/// @return a × b
[[nodiscard]] inline constexpr Vec3 cross(const Vec3& a, const Vec3& b) noexcept {
    return Vec3{
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    };
}

/// @brief Scalar multiplication (scalar * Vec3).
/// 
/// Enables natural syntax: `Vec3 result = 2.0 * velocity;`
/// @param s Scalar multiplier
/// @param v Vector
/// @return Scaled vector
[[nodiscard]] inline constexpr Vec3 operator*(double s, const Vec3& v) noexcept {
    return v * s;
}

/// @brief Approximate equality comparison with tolerance.
/// 
/// Two vectors are considered equal if the distance between them is less
/// than the specified tolerance.
/// @param a First vector
/// @param b Second vector
/// @param eps Tolerance (default: 1e-10)
/// @return true if ||a - b|| < eps
[[nodiscard]] inline bool near_equal(const Vec3& a, const Vec3& b, 
                                      double eps = 1e-10) noexcept {
    return (a - b).mag_sq() < eps * eps;
}

/// @brief Compute the angle between two vectors.
/// @param a First vector (must be non-zero)
/// @param b Second vector (must be non-zero)
/// @return Angle in radians [0, π]
/// @pre Both vectors must be non-zero
[[nodiscard]] inline double angle_between(const Vec3& a, const Vec3& b) noexcept {
    const double denom = a.mag() * b.mag();
    assert(denom > 0 && "angle_between requires non-zero vectors");
    
    // Clamp to [-1, 1] to handle numerical errors
    const double cos_angle = dot(a, b) / denom;
    const double clamped = (cos_angle < -1.0) ? -1.0 : (cos_angle > 1.0 ? 1.0 : cos_angle);
    return std::acos(clamped);
}

/// @brief Project vector a onto vector b.
/// @param a Vector to project
/// @param b Vector to project onto (must be non-zero)
/// @return Component of a in direction of b
/// @pre b must be non-zero
[[nodiscard]] inline Vec3 project(const Vec3& a, const Vec3& b) noexcept {
    const double b_mag_sq = b.mag_sq();
    assert(b_mag_sq > 0 && "Cannot project onto zero vector");
    return b * (dot(a, b) / b_mag_sq);
}

/// @brief Compute the component of a perpendicular to b.
/// @param a Vector to decompose
/// @param b Reference direction (must be non-zero)
/// @return Component of a perpendicular to b
/// @pre b must be non-zero
[[nodiscard]] inline Vec3 reject(const Vec3& a, const Vec3& b) noexcept {
    return a - project(a, b);
}

/// @brief Linear interpolation between two vectors.
/// @param a Start vector (t=0)
/// @param b End vector (t=1)
/// @param t Interpolation parameter [0, 1]
/// @return Interpolated vector
[[nodiscard]] inline constexpr Vec3 lerp(const Vec3& a, const Vec3& b, double t) noexcept {
    return a + (b - a) * t;
}

}  // namespace trajsim