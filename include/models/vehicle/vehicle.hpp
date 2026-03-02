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
    struct StageCfg 
    {
        std::vector<double> mass;
        std::vector<int> numberOfEngine;
        std::vector<Engine::Config> engineCfg;
    };

    struct Config {
        int numberOfStage = 1;
        double mass = 0.0;                      ///< Total initial mass [kg]
        double oxidizerMass = 0.0;              ///< Oxidizer mass [kg]
        double fuelMass = 0.0;                  ///< Fuel mass [kg]
        StageCfg stage;
        Aerodynamics::Config aeroCfg;
    };

    /// @brief Construct vehicle with subsystems.
    /// @param config  Vehicle configuration (mass, staging).
    /// @param aero    Aerodynamic model (taken by value, moved in).
    /// @param engine  Engine model (ownership transferred, may be nullptr).
    Vehicle(const Config& config,
            Aerodynamics aero,
            std::unique_ptr<Engine> engine)
        : numberOfStage(config.numberOfStage)
        , mass{config.mass}
        , propellantMass{config.oxidizerMass + config.fuelMass}
        , aero{std::move(aero)}
        , engine{std::move(engine)}
    {
        currentStage = 1;
    }

    virtual ~Vehicle() = default;
    Vehicle(const Vehicle&) = delete;
    Vehicle(Vehicle&&) = default;
    Vehicle& operator=(const Vehicle&) = delete;
    Vehicle& operator=(Vehicle&&) = delete;

    [[nodiscard]] double getMass() const noexcept { return mass; }
    [[nodiscard]] double getNumberOfStage() const noexcept { return numberOfStage; }
    [[nodiscard]] double getPropellantMass() const noexcept { return propellantMass; }
    void consumePropellant(double amount) { propellantMass -= amount; }
    [[nodiscard]] const Aerodynamics& getAero() const noexcept { return aero; }
    [[nodiscard]] const Engine& getEngine() const { return *engine; }
    [[nodiscard]] Engine& getEngine() { return *engine; }
    [[nodiscard]] double getStage() const noexcept { return currentStage; }

private:
    double currentStage;
    double numberOfStage;
    double mass;
    double propellantMass;
    Aerodynamics aero;
    std::unique_ptr<Engine> engine;
};

}  // namespace trajsim
