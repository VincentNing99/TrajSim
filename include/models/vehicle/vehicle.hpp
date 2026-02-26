#pragma once
// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Vincent Ning

/// @file vehicle.hpp
/// @brief Vehicle model aggregating aerodynamics and propulsion subsystems.

#include "core/vec3.hpp"
#include "engine_models/engine.hpp"
#include "aerodynamics.hpp"
#include <memory>
#include <vector>

namespace trajsim {

/// @brief Top-level vehicle model owning aerodynamics and engine subsystems.
class Vehicle {
public:
    struct StageConfig {
        int stage;                      ///< Stage number (1-based)
        int numberOfEngine;             ///< Number of engines in this stage
        EngineModel::Config engine;     ///< Per-engine configuration
    };

    struct Stage {
        int stage;
        std::vector<EngineModel> engines;
    };

    struct Config {
        double numberOfStage = 1;
        double mass = 0.0;                      ///< Total initial mass [kg]
        std::vector<StageConfig> stages;        ///< Per-stage engine configs
    };

    /// @brief Construct vehicle with subsystems.
    /// @param config  Vehicle configuration (mass, staging).
    /// @param aero    Aerodynamic model (taken by value, moved in).
    /// @param engine  Engine model (ownership transferred, may be nullptr).
    Vehicle(const Config& config,
            Aerodynamics aero,
            std::unique_ptr<EngineModel> engine)
        : numberOfStage{config.numberOfStage}
        , mass{config.mass}
        , aero{std::move(aero)}
        , engine{std::move(engine)}
    {}

    virtual ~Vehicle() = default;
    Vehicle(const Vehicle&) = delete;
    Vehicle(Vehicle&&) = default;
    Vehicle& operator=(const Vehicle&) = delete;
    Vehicle& operator=(Vehicle&&) = delete;

    [[nodiscard]] double getMass() const noexcept { return mass; }
    [[nodiscard]] double getNumberOfStage() const noexcept { return numberOfStage; }
    [[nodiscard]] const Aerodynamics& getAero() const noexcept { return aero; }
    [[nodiscard]] const EngineModel& getEngine() const { return *engine; }
    [[nodiscard]] EngineModel& getEngine() { return *engine; }

private:
    double numberOfStage;
    double mass;
    Aerodynamics aero;
    std::unique_ptr<EngineModel> engine;
};

}  // namespace trajsim
