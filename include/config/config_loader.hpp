#pragma once
// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Vincent Ning

/// @file config_loader.hpp
/// @brief JSON config loading with validation and UI-ready diagnostics.
///
/// A single JSON file contains all configuration sections:
/// @code
///     {
///         "vehicle":      { "stageCfg": { ... }, "aerodynamics": { ... }, ... },
///         "simulation":   { ... },
///         "guidance":     [ { ... } ],
///         "mission":      { ... }
///     }
/// @endcode
///
/// Usage:
/// @code
///     auto result = loadConfig("config_mission_1.json");
///     for (const auto& w : result.warnings)
///         ui.showWarning(w);
///     const AppConfig& cfg = result.config;
/// @endcode
///
/// Diagnostics contract:
///   - Unrecognized JSON keys  → added to ConfigResult::warnings (non-fatal)
///   - Missing required fields → std::invalid_argument thrown (fatal)
///   - Invalid values          → std::invalid_argument thrown via validate() (fatal)
///   - File not found/bad JSON → std::runtime_error thrown (fatal)
#include "core/constants.hpp"
#include "models/vehicle/aerodynamics.hpp"
#include "models/guidance/guidance.hpp"
#include "models/vehicle/vehicle.hpp"
#include "models/reference_mission.hpp"
#include "core/sim.hpp"
#include <string>
#include <vector>

namespace trajsim {

// =============================================================================
// Diagnostic Container
// =============================================================================

/// @brief Holds a loaded config and any non-fatal warnings collected during loading.
template <typename T>
struct ConfigResult {
    T config;
    std::vector<std::string> warnings;

    [[nodiscard]] bool hasWarnings() const noexcept { return !warnings.empty(); }
};

// =============================================================================
// Combined Application Config
// =============================================================================

/// @brief All configuration needed for a single simulation run.
struct AppConfig {
    Vehicle::Config                  vehicle;
    SimConfig                        simulation;
    std::vector<Guidance::Config>    guidance;
    ReferenceMission                 mission;
};

// =============================================================================
// Loader
// =============================================================================

/// @brief Load all configuration from a single JSON file.
/// @param path Path to the combined JSON config file.
/// @return ConfigResult with loaded AppConfig and any warnings.
/// @throws std::runtime_error if file cannot be opened or JSON is malformed.
/// @throws std::invalid_argument if required fields/sections are missing or values are invalid.
[[nodiscard]] ConfigResult<AppConfig> loadConfig(const std::string& path);

} // namespace trajsim
