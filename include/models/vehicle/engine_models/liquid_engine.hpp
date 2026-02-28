#pragma once
// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Vincent Ning

/// @file liquid_engine.hpp
/// @brief Liquid rocket engine model

#include "engine.hpp"
#include <string>
#include <memory>

namespace trajsim {

class LiquidEngine final : public Engine {
public:
    struct Config {
        std::string id;
        double thrust;        ///< [N]
        double isp;           ///< [s]
        double mdot;         ///< [kg/s]
        double nozzleExitArea;  ///< [m²]
        double nozzleExitPressure; ///< [Pascal]
        Vec3 mountOffset;        ///< From CoM [m]
        double dryMass;          ///< [kg]
        double gimbalLimit;      ///< Max gimbal angle [rad]
        double throttleMin;      ///< Minimum throttle [0,1]
    };

    // =========================================================================
    // Construction
    // =========================================================================

    [[nodiscard]] static std::unique_ptr<LiquidEngine> fromConfig(const Config& cfg);

    // =========================================================================
    // Core Interface
    // =========================================================================

    [[nodiscard]] EngineOutput computeOutput(
        double throttle,
        double burnTime,
        double ambientPressure,
        const Vec3& gimbalAngles
    ) override;

    [[nodiscard]] bool isThrottleable() const noexcept override {
        return cfg.throttleMin < 1.0;
    }

    [[nodiscard]] bool isGimbaled() const noexcept override {
        return cfg.gimbalLimit > 0.0;
    }

    [[nodiscard]] std::string_view getId() const noexcept override {
        return cfg.id;
    }

    // =========================================================================
    // Properties
    // =========================================================================

    [[nodiscard]] double getThrust() const noexcept override { return cfg.thrust; }
    [[nodiscard]] double getIsp() const noexcept override { return cfg.isp; }
    [[nodiscard]] double getDryMass() const noexcept override { return cfg.dryMass; }

private:
    explicit LiquidEngine(Config cfg);
    Config cfg;
};

}  // namespace trajsim
