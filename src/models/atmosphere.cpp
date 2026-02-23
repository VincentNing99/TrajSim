#include "models/atmosphere.hpp"
#include "core/constants.hpp"
#include <cassert>  // Needed for assert()
#include <cmath>    // Needed for std::exp, std::pow, std::sqrt, std::abs
using namespace trajsim;

/// @brief Computes atmosphere properties
/// @param Velocites in body frame in m/s
/// @param Geometric Altitude in meters
AtmosphereState AtmosphereModel::computeStates(const Vec3& vb, double geometricAltitude) const noexcept
{
    // Convert to geopotential altitude for atmosphere layer lookup
    double geopotentialAltitude = geometricAltitude / (1 + geometricAltitude / EarthConstants::SEMI_MINOR_AXIS);
    const LayerParams layer = findLayer(geopotentialAltitude);

    AtmosphereState result;

    result.temperature = layer.baseTemperature + layer.lapseRate * (geopotentialAltitude - layer.baseAltitude);

    // Pressure (isothermal vs gradient)
    if (std::abs(layer.lapseRate) < 1e-10) {
        result.pressure = layer.basePressure * std::exp(-EarthConstants::GRAVITATIONAL_ACCELERATION * (geopotentialAltitude - layer.baseAltitude)
                        / (AtmosphereConstants::GAS_CONSTANT * layer.baseTemperature));
    } else {
        result.pressure = layer.basePressure * std::pow(result.temperature / layer.baseTemperature,
                        -EarthConstants::GRAVITATIONAL_ACCELERATION / (AtmosphereConstants::GAS_CONSTANT * layer.lapseRate));
    }

    result.density = result.pressure / (AtmosphereConstants::GAS_CONSTANT * result.temperature);
    result.speedOfSound = std::sqrt(AtmosphereConstants::GAMMA * AtmosphereConstants::GAS_CONSTANT * result.temperature);
    // Squared velocity magnitude in body frame
    double vbMag = vb.mag();
    result.dynamicPressure = 0.5 * result.density * vbMag * vbMag;
    result.machNumber = vbMag / result.speedOfSound;

    return result;
}

const AtmosphereModel::LayerParams AtmosphereModel::findLayer(double geopotentialAltitude) const noexcept
{
    constexpr size_t kNumLayers = sizeof(kLayers) / sizeof(kLayers[0]);
    assert(geopotentialAltitude >= 0.0 && "Negative altitude");
    for (size_t i = kNumLayers - 1; i > 0; --i)
    {
        if(geopotentialAltitude >= kLayers[i].baseAltitude)
        {
            return kLayers[i];
        }
    }
    return kLayers[0];
}
