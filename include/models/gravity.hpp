#pragma once

#include "core/vec3.hpp"
#include "core/mat3.hpp"
#include "core/constants.hpp"

namespace trajsim {

/**
 * @brief J2 oblate Earth gravity model
 *
 * Computes gravitational acceleration accounting for Earth's equatorial bulge
 * using the J2 zonal harmonic perturbation. All computations are performed in
 * a launch-site-centered coordinate frame.
 *
 * Coordinate frame:
 *   - Origin: launch site
 *   - Y-axis: local vertical (up)
 *   - X-Z plane: local horizontal, oriented by aiming azimuth
 */
class Gravity {
public:
    /**
     * @brief Construct gravity model for a launch site
     * @param latitude   Geodetic latitude of launch site [rad]
     * @param altitude   Altitude above reference ellipsoid [m]
     * @param aimingAzimuth  Azimuth angle for local frame orientation [rad]
     */
    Gravity(double latitude, double altitude, double aimingAzimuth = 0.0) {
        geodeticLatitude = latitude;
        height = altitude;

        // Convert geodetic to geocentric latitude
        double e2 = EarthConstants::SECOND_ECCENTRICITY * EarthConstants::SECOND_ECCENTRICITY;
        geocentricLatitude = std::atan(std::tan(geodeticLatitude) / (1.0 + e2));

        // Polar axis unit vector in local frame
        double cosLat = std::cos(geodeticLatitude);
        double sinLat = std::sin(geodeticLatitude);
        double cosAz = std::cos(aimingAzimuth);
        double sinAz = std::sin(aimingAzimuth);
        // Polar axis points towards North Pole, unit vector of polar axis in local frame
        polarAxis = { cosAz * cosLat, sinLat, -cosLat * sinAz };

        // Geocentric radius at launch site (on reference ellipsoid + altitude)
        double a = EarthConstants::RADIUS_OF_EQUATOR;
        double b = EarthConstants::SEMI_MINOR_AXIS;
        double sinGc = std::sin(geocentricLatitude);
        double cosGc = std::cos(geocentricLatitude);
        launchGeocentricRadius = a * b / std::sqrt(a * a * sinGc * sinGc + b * b * cosGc * cosGc) + height;

        // Launch site position vector (from Earth center, in local frame)
        double latDiff = geodeticLatitude - geocentricLatitude;
        double sinDiff = std::sin(latDiff);
        double cosDiff = std::cos(latDiff);
        launchGeocentricPosition = {
            -launchGeocentricRadius * sinDiff * cosAz,
             launchGeocentricRadius * cosDiff,
             launchGeocentricRadius * sinAz * sinDiff
        };
    }

    /** @brief Get geocentric radius at launch site [m] */
    double getLaunchGeocentricRadius() const { return launchGeocentricRadius; }

    /** @brief Get geocentric radius vector at launch site in inertial frame [m] */
    Vec3 getLaunchGeocentricRadiusVector() const { return launchGeocentricPosition; }
    /**
     * @brief Compute J2 gravitational acceleration
     * @param pos  Position relative to launch site [m], in local frame
     * @return Gravitational acceleration [m/s²], in local frame
     */
    Vec3 computeAcceleration(const Vec3& pos) const;

    /**
     * @brief Compute altitude above reference ellipsoid
     * @param pos  Position relative to launch site [m], in local frame
     * @return Altitude [m]
     */
    double computeAltitude(const Vec3& pos) const;

    inline Vec3 getPolarAxis() const {return polarAxis;};

private:
    Vec3 launchGeocentricPosition;  ///< Launch site geocentric position in local frame [m]
    Vec3 polarAxis;                 ///< Earth's polar axis unit vector in local frame
    double geodeticLatitude;        ///< Geodetic latitude [rad]
    double geocentricLatitude;      ///< Geocentric latitude [rad]
    double height;                  ///< Launch site altitude [m]
    double launchGeocentricRadius;  ///< Geocentric radius at launch [m]
};

}  // namespace trajsim
