//
//  types.hpp
//  FlightSim
//
//  Type-safe wrappers for common data structures
//

#ifndef types_hpp
#define types_hpp

#include <vector>
#include <cmath>

// 3D vector for position, velocity, acceleration, etc.
struct Vec3 {
    double x, y, z;

    // Constructors
    Vec3() : x(0.0), y(0.0), z(0.0) {}
    Vec3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}

    // Construct from std::vector<double>
    explicit Vec3(const std::vector<double>& v) {
        if (v.size() >= 3) {
            x = v[0];
            y = v[1];
            z = v[2];
        } else {
            x = y = z = 0.0;
        }
    }

    // Convert to std::vector<double>
    std::vector<double> to_vector() const {
        return {x, y, z};
    }

    // Magnitude
    double magnitude() const {
        return std::sqrt(x*x + y*y + z*z);
    }

    // Operators
    Vec3 operator+(const Vec3& other) const {
        return Vec3(x + other.x, y + other.y, z + other.z);
    }

    Vec3 operator-(const Vec3& other) const {
        return Vec3(x - other.x, y - other.y, z - other.z);
    }

    Vec3 operator*(double scalar) const {
        return Vec3(x * scalar, y * scalar, z * scalar);
    }

    Vec3 operator/(double scalar) const {
        return Vec3(x / scalar, y / scalar, z / scalar);
    }

    // Dot product
    double dot(const Vec3& other) const {
        return x * other.x + y * other.y + z * other.z;
    }

    // Cross product
    Vec3 cross(const Vec3& other) const {
        return Vec3(
            y * other.z - z * other.y,
            z * other.x - x * other.z,
            x * other.y - y * other.x
        );
    }

    // Array access
    double& operator[](size_t index) {
        switch(index) {
            case 0: return x;
            case 1: return y;
            case 2: return z;
            default: return x; // fallback
        }
    }

    const double& operator[](size_t index) const {
        switch(index) {
            case 0: return x;
            case 1: return y;
            case 2: return z;
            default: return x; // fallback
        }
    }
};

// Scalar * Vec3
inline Vec3 operator*(double scalar, const Vec3& v) {
    return v * scalar;
}

// Euler angles for attitude representation
struct EulerAngles {
    double phi;   // Pitch
    double psi;   // Yaw
    double gamma; // Roll

    // Constructors
    EulerAngles() : phi(0.0), psi(0.0), gamma(0.0) {}
    EulerAngles(double phi_, double psi_, double gamma_) : phi(phi_), psi(psi_), gamma(gamma_) {}

    // Construct from std::vector<double>
    explicit EulerAngles(const std::vector<double>& v) {
        if (v.size() >= 3) {
            phi = v[0];
            psi = v[1];
            gamma = v[2];
        } else {
            phi = psi = gamma = 0.0;
        }
    }

    // Convert to std::vector<double>
    std::vector<double> to_vector() const {
        return {phi, psi, gamma};
    }

    // Array access
    double& operator[](size_t index) {
        switch(index) {
            case 0: return phi;
            case 1: return psi;
            case 2: return gamma;
            default: return phi; // fallback
        }
    }

    const double& operator[](size_t index) const {
        switch(index) {
            case 0: return phi;
            case 1: return psi;
            case 2: return gamma;
            default: return phi; // fallback
        }
    }
};

// IGM (Iterative Guidance Mode) helper structures
struct IGMCoefficients {
    double A;
    double J;
    double S;
    double Q;
    double tau_remaining;
};

struct DeltaVelocity {
    double del_v_xi;
    double del_v_eta;
    double del_v_zeta;
    double del_v;
};

struct SteeringAngles {
    double phi;
    double psi;
};

#endif /* types_hpp */
