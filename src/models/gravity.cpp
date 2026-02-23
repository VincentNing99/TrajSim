#include "models/gravity.hpp"

namespace trajsim {

Vec3 Gravity::computeAcceleration(const Vec3& pos) const {
    // Position vector from Earth's center (in local frame)
    Vec3 rVec = launchGeocentricPosition + pos;
    double r = rVec.mag();
    double r2 = r * r;
    double r4 = r2 * r2;

    // Unit position vector (radial direction)
    Vec3 rHat = rVec / r;

    // Sine of geocentric latitude: sin(φ) = r̂ · ω̂
    double sinPhi = rHat.x * polarAxis.x + rHat.y * polarAxis.y + rHat.z * polarAxis.z;
    double sinPhi2 = sinPhi * sinPhi;

    // J2 gravity model constants
    double mu = EarthConstants::GRAVITATIONAL_CONST;
    double J = EarthConstants::J;
    double Re2 = EarthConstants::RADIUS_OF_EQUATOR * EarthConstants::RADIUS_OF_EQUATOR;

    // Radial component: g_r = -μ/r² * [1 + J*(Re/r)²*(1 - 5sin²φ)]
    double gr = -mu / r2 * (1.0 + J * Re2 * (1.0 - 5.0 * sinPhi2) / r2);

    // Polar axis component: g_ω = -2J*μ*Re²*sinφ / r⁴
    // Combined with radial term, this produces correct J2 Cartesian accelerations
    double gOmega = -2.0 * J * mu * Re2 * sinPhi / r4;

    // Total gravity: radial + polar axis contributions
    return gr * rHat + gOmega * polarAxis;
}

double Gravity::computeAltitude(const Vec3& pos) const {
    // Position vector from Earth's center
    Vec3 rVec = launchGeocentricPosition + pos;
    double r = rVec.mag();

    // Unit position vector
    Vec3 rHat = rVec / r;

    // Sine of geocentric latitude
    double sinPhi = rHat.x * polarAxis.x + rHat.y * polarAxis.y + rHat.z * polarAxis.z;

    // Local Earth radius (reference ellipsoid)
    double e2 = EarthConstants::SECOND_ECCENTRICITY * EarthConstants::SECOND_ECCENTRICITY;
    double Re = EarthConstants::RADIUS_OF_EQUATOR / std::sqrt(1.0 + e2 * sinPhi * sinPhi);

    return r - Re;
}

}  // namespace trajsim