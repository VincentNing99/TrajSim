#pragma once
// SPDX-License-Identifier: MIT

/// @file atmosphere.hpp
/// @brief US Standard Atmosphere 1976 model for aerospace computations
///
/// Implements the US Standard Atmosphere 1976 (NOAA-S/T 76-1562) for
/// computing atmospheric properties as a function of geometric altitude.
///
/// ## Usage
/// @code
///     AtmosphereModel atmo;
///     Vec3 velocityBody{100.0, 0.0, 0.0};
///     AtmosphereState state = atmo.computeStates(velocityBody, 10000.0);
///     double mach = state.machNumber;
/// @endcode
///
/// ## Valid Range
/// Altitude: 0 to 86 km (extrapolates beyond using top layer)
///
/// ## Thread Safety
/// No shared state. Operations on separate instances are thread-safe.
///
/// @author Vincent Ning
/// @date 2025-12
#include "core/vec3.hpp"

namespace trajsim {

// =============================================================================
// Atmospheric State
// =============================================================================

/// @brief Atmospheric state at a given altitude
struct AtmosphereState {
    double temperature;       ///< [K] Static temperature
    double pressure;          ///< [Pa] Static pressure
    double density;           ///< [kg/m³] Air density
    double speedOfSound;    ///< [m/s] Local speed of sound
    double machNumber;       ///< [-] Mach number
    double dynamicPressure;  ///< [Pa] Dynamic pressure (q = ½ρV²)
};

class AtmosphereModel{
public:

    /// @brief Compute atmospheric properties at geometric altitude
    /// @param Velocity in body frame m/s
    /// @param Geocentric altitude
    /// @return Atmospheric state
    [[nodiscard]] AtmosphereState computeStates(const Vec3& vb, double geometricAltitude) const noexcept;

private:
    // =========================================================================
    // Layer Data
    // =========================================================================

    /// @brief Atmospheric layer parameters
    struct LayerParams {
        double baseAltitude;     ///< [m] Geopotential altitude
        double baseTemperature;  ///< [K] Temperature at base
        double lapseRate;        ///< [K/m] Temperature gradient
        double basePressure;     ///< [Pa] Pressure at base
    };

    /// US Standard Atmosphere 1976 layer definitions
    ///        h_b [m]    T_b [K]    L_b [K/m]     P_b [Pa]
    static constexpr LayerParams kLayers[] = {
        {     0.0,       288.15,    -6.5e-3,      101325.0   },  // Troposphere
        { 11000.0,       216.65,     0.0,          22632.1   },  // Tropopause
        { 20000.0,       216.65,     1.0e-3,        5474.88  },  // Stratosphere I
        { 32000.0,       228.65,     2.8e-3,         868.018 },  // Stratosphere II
        { 47000.0,       270.65,     0.0,            110.906 },  // Stratopause
        { 51000.0,       270.65,    -2.8e-3,          66.9387},  // Mesosphere I
        { 71000.0,       214.65,    -2.0e-3,           3.9564},  // Mesosphere II
    };
    // =========================================================================
    // Private Methods
    // =========================================================================

    /// @brief Find atmospheric layer for given altitude
    /// @param geopotentialAltitude Geopotential altitude [m]
    /// @return Layer parameters
    /// @pre geopotentialAltitude >= 0
    [[nodiscard]] const LayerParams findLayer(double geopotentialAltitude) const noexcept;
};

} // namespace trajsim
