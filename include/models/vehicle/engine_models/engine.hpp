#pragma once
// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Vincent Ning

/// @file engine.hpp
/// @brief Abstract base class for engine/motor models.

#include <memory>
#include <string>
#include "core/vec3.hpp"
#include "core/types.hpp"

namespace trajsim {

/// @brief Output from an engine computation step.
struct EngineOutput {
    Vec3 force;     ///< Thrust force [N] in body frame
    Vec3 moment;    ///< Thrust moment [N·m] in body frame
    double mdot;    ///< Mass flow rate [kg/s]

    [[nodiscard]] static constexpr EngineOutput zero() noexcept {
        return EngineOutput{{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, 0.0};
    }
};

// =============================================================================
// Engine Model Interface
// =============================================================================

/// @brief Abstract interface for propulsion models.
///
/// Derived classes implement specific engine types (liquid, solid, hybrid).
/// All outputs are in body frame with thrust along +X by convention.
class Engine {
public:
    struct Config {
        std::string id;
        std::string propellant;         ///< "liquid" or "solid"
        std::string nozzleType;         ///< "sea-level" or "vacuum"
        double thrust;              ///< [N]
        double isp;                 ///< [s]
        double mdot;                ///< [kg/s]
        double nozzleExitArea;      ///< [m²]
        double nozzleExitPressure;  ///< [Pascal]
        Vec3 mountOffset;           ///< From CoM [m]
    };

    virtual ~Engine() = default;
    Engine(const Engine&) = delete;
    Engine& operator=(const Engine&) = delete;
    Engine(Engine&&) = default;
    Engine& operator=(Engine&&) = default;

    // =========================================================================
    // Core Interface
    // =========================================================================

    /// @brief Compute thrust and moment for current conditions.
    /// @param throttle Throttle setting [0, 1]
    /// @param burnTime Time since ignition [s]
    /// @param ambientPressure Ambient pressure [Pa]
    /// @param gimbalAngles Gimbal angles (pitch, yaw) [rad]
    /// @return Engine output (force, moment, mass flow)
    [[nodiscard]] virtual EngineOutput computeOutput(
        double throttle,
        double burnTime,
        double ambientPressure,
        const Vec3& gimbalAngles) = 0;

    /// @brief Check if engine can throttle.
    [[nodiscard]] virtual bool isThrottleable() const noexcept = 0;

    /// @brief Check if engine can gimbal.
    [[nodiscard]] virtual bool isGimbaled() const noexcept = 0;

    /// @brief Get engine identifier.
    [[nodiscard]] virtual std::string_view getId() const noexcept = 0;

    // =========================================================================
    // Properties
    // =========================================================================

    /// @brief Thrust at full throttle [N].
    [[nodiscard]] virtual double getThrust() const noexcept = 0;

    /// @brief Specific impulse in vacuum [s].
    [[nodiscard]] virtual double getIsp() const noexcept = 0;

    /// @brief Dry mass of engine [kg].
    [[nodiscard]] virtual double getDryMass() const noexcept = 0;

protected:
    Engine() = default;
};

// =============================================================================
// Factory
// =============================================================================

/// @brief Create engine model from configuration.
/// @param config Engine configuration parameters.
/// @return Owning pointer to engine model.
/// @throws std::runtime_error on invalid config.
[[nodiscard]] std::unique_ptr<Engine> createEngine(const Engine::Config& config);

}  // namespace trajsim
