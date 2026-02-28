#pragma once
/// @brief aerodynamics computation module
/// @author Vincent Ning
/// @date 2025-12-06
#include "core/vec3.hpp"
#include <vector>
#include <cmath>
#include <stdexcept>
#include <string>
#include <algorithm>

namespace trajsim {
/// Aerodynamic coefficients in body frame
struct AeroCoefficients {
    Vec3 forceCoef;   ///< Cx, Cy, Cz [-]
    Vec3 momentCoef;  ///< CMx, CMy, CMz [-]
};

/// Result of aerodynamic computation
struct AeroResult {
    Vec3 force;            ///< [N] body frame
    Vec3 moment;           ///< [N·m] body frame
    AeroCoefficients coef;
};

/// Aerodynamic table grid metadata
struct AeroGrid {
    std::vector<double> machVals;
    std::vector<double> betaVals;
    std::vector<double> alphaVals;
    size_t strideMach = 0;
    size_t strideBeta = 0;

    void computeStrides() noexcept{
        strideBeta = alphaVals.size();
        strideMach = betaVals.size() * strideBeta;
    }

    [[nodiscard]] bool isValid() const noexcept {
        return !machVals.empty() && !betaVals.empty() && !alphaVals.empty();
    }
    // Immutable for now, making them static
    static constexpr size_t COL_MACH  = 0;
    static constexpr size_t COL_BETA  = 1;
    static constexpr size_t COL_ALPHA = 2;
    static constexpr size_t COL_CX    = 3;
    static constexpr size_t COL_CY    = 4;
    static constexpr size_t COL_CZ    = 5;
    static constexpr size_t COL_CMX   = 6;
    static constexpr size_t COL_CMY   = 7;
    static constexpr size_t COL_CMZ   = 8;
    static constexpr size_t NUM_COLS  = 9;
};


/// Aerodynamic force and moment model using table lookup with trilinear interpolation.
///
/// Tables must be structured as:
/// - Columns: Mach, Beta, Alpha, Cx, Cy, Cz, CMx, CMy, CMz
/// - Rows sorted by: Mach (slowest), Beta, Alpha (fastest varying)
/// - All combinations of (Mach, Beta, Alpha) must be present (full grid)
///
/// Example usage:
/// @code
///     Aerodynamics::Config config;
///     Aerodynamics aero(config, highSpeedTable, lowSpeedTable);
///     AeroResult result = aero.compute(mach, alpha, beta, q);
/// @endcode
class Aerodynamics {
public:
    struct Config {
        double refArea;        // [m^2]
        double refLength;      // [m]
        double machMin;
        double machMax;
        double machTransition;

        void validate() const {
            if (refArea <= 0)
                throw std::invalid_argument("AeroConfig: refArea must be > 0, got " + std::to_string(refArea));
            if (refLength <= 0)
                throw std::invalid_argument("AeroConfig: refLength must be > 0, got " + std::to_string(refLength));
            if (machMin <= 0)
                throw std::invalid_argument("AeroConfig: machMin must be > 0, got " + std::to_string(machMin));
            if (machMax <= machMin)
                throw std::invalid_argument("AeroConfig: machMax (" + std::to_string(machMax) +
                                            ") must be > machMin (" + std::to_string(machMin) + ")");
            if (machTransition <= machMin || machTransition >= machMax)
                throw std::invalid_argument("AeroConfig: machTransition must be in (machMin, machMax), got " +
                                            std::to_string(machTransition));
        }
    };

    /// Construct aerodynamic model with lookup tables
    /// @param config Model configuration (reference area, length, Mach limits)
    /// @param highSpeedTable Table for Mach >= machTransition
    /// @param lowSpeedTable Table for Mach < machTransition
    /// @throws std::invalid_argument if tables are empty or malformed
    Aerodynamics(const Config& config,
                 const std::vector<std::vector<double>>& highSpeedTable,
                 const std::vector<std::vector<double>>& lowSpeedTable);

    /// Compute aerodynamic forces and moments
    /// @param mach Mach number [-] (clamped to [machMin, machMax])
    /// @param alpha Angle of attack [deg] (must be within ±90°)
    /// @param beta Sideslip angle [deg] (must be within ±90°)
    /// @param dynamicPressure Dynamic pressure [Pa] (must be non-negative)
    /// @return Forces [N], moments [N·m], and coefficients
    /// @throws std::invalid_argument if inputs are NaN or out of range
    [[nodiscard]] AeroResult compute(double mach, double alpha, double beta, double dynamicPressure, double stage) const;

    /// Get current configuration
    [[nodiscard]] const Config& getConfig() const noexcept { return cfg; }

    // Get current reference area
    [[nodiscard]] double getRefArea(double stage) const noexcept { return refArea[static_cast<size_t>(stage - 1)]; }
    // Get current reference length
    [[nodiscard]] double getRefLength(double stage) const noexcept { return refLength[static_cast<size_t>(stage - 1)]; }

private:
    // Methods
    AeroCoefficients interpolate(const AeroGrid& grid,
                                 const std::vector<std::vector<double>>& table,
                                 double mach, double beta, double alpha) const;
    void buildGrid(const std::vector<std::vector<double>>& table,
                    AeroGrid& grid,
                    const std::string& tableName) const;
    void validateTable(const std::vector<std::vector<double>>& table,
                        const std::string& tableName) const;

    // Data
    const Config cfg;
    std::vector<std::vector<double>> highSpeedTable;
    std::vector<std::vector<double>> lowSpeedTable;
    AeroGrid highSpeedGrid;
    AeroGrid lowSpeedGrid;
    std::vector<double> refArea;
    std::vector<double> refLength;
};
} // namespace trajsim
