// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Vincent Ning

/// @file config_loader.cpp
/// @brief JSON config loading implementation.

#include "config/config_loader.hpp"
#include "models/vehicle/vehicle.hpp"
#include <fstream>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

namespace trajsim {

// =============================================================================
// Internal helpers
// =============================================================================

static json openJson(const std::string& path) {
    std::ifstream file(path);
    if (!file.is_open())
        throw std::runtime_error("Config: cannot open file '" + path + "'");
    try {
        return json::parse(file);
    } catch (const json::parse_error& e) {
        throw std::runtime_error("Config: JSON parse error in '" + path + "': " + e.what());
    }
}

template <typename T>
static T requireField(const json& j, const std::string& key, const std::string& context) {
    if (!j.contains(key))
        throw std::invalid_argument(context + ": required field '" + key + "' is missing");
    try {
        return j.at(key).get<T>();
    } catch (const json::type_error&) {
        throw std::invalid_argument(context + ": field '" + key + "' has wrong type (expected " +
                                    typeid(T).name() + ")");
    }
}

static void warnUnknownKeys(const json& j,
                             const std::vector<std::string>& known,
                             const std::string& context,
                             std::vector<std::string>& warnings) {
    for (auto& [key, _] : j.items()) {
        bool found = false;
        for (const auto& k : known)
            if (k == key) { found = true; break; }
        if (!found)
            warnings.push_back(context + ": unrecognized key '" + key +
                                "' will be ignored (possible typo?)");
    }
}

/// @brief Validates algorithm type string
static void validateAlgorithmType(const std::string& type, const std::string& context) {
    if (type != "IterativeGuidance" && type != "OpenLoopGuidance") {
        throw std::invalid_argument(context + ": invalid algorithm type '" + type +
                                    "' (must be 'IterativeGuidance' or 'OpenLoopGuidance')");
    }
}

static Vec3 requireVec3(const json& j, const std::string& key, const std::string& context) {
    if (!j.contains(key))
        throw std::invalid_argument(context + ": required field '" + key + "' is missing");
    const auto& arr = j.at(key);
    if (!arr.is_array() || arr.size() != 3)
        throw std::invalid_argument(context + ": field '" + key + "' must be an array of 3 doubles");
    return Vec3{arr[0].get<double>(), arr[1].get<double>(), arr[2].get<double>()};
}

static SteeringAngles requireSteeringAnglesDeg(const json& j, const std::string& key, const std::string& context) {
    if (!j.contains(key))
        throw std::invalid_argument(context + ": required field '" + key + "' is missing");
    const auto& arr = j.at(key);
    if (!arr.is_array() || arr.size() != 3)
        throw std::invalid_argument(context + ": field '" + key + "' must be an array of 3 doubles");
    return SteeringAngles{
        arr[0].get<double>() * degToRad,
        arr[1].get<double>() * degToRad,
        arr[2].get<double>() * degToRad
    };
}

static const json& requireSection(const json& root, const std::string& key, const std::string& context) {
    if (!root.contains(key))
        throw std::invalid_argument(context + ": required section '" + key + "' is missing");
    const auto& section = root.at(key);
    if (!section.is_object())
        throw std::invalid_argument(context + ": section '" + key + "' must be a JSON object");
    return section;
}

// =============================================================================
// Section parsers
// =============================================================================

static Vehicle::Config parseVehicle(const json& j, std::vector<std::string>& warnings) {
    const std::string ctx = "vehicle";

    static const std::vector<std::string> knownKeys = {
        "mass"
    };
    warnUnknownKeys(j, knownKeys, ctx, warnings);

    Vehicle::Config cfg;
    cfg.mass = requireField<double>(j, "mass", ctx);

    return cfg;
}

static std::vector<Vehicle::StageConfig> parseEngines(const json& arr, std::vector<std::string>& warnings) {
    static const std::vector<std::string> knownKeys = {
        "stage", "propellant", "nozzleType", "numberOfEngine",
        "thrustPerEngine", "ISP", "massFlowRate", "exitArea"
    };

    std::vector<Vehicle::StageConfig> stages;
    stages.reserve(arr.size());

    for (size_t i = 0; i < arr.size(); ++i) {
        const auto& e = arr[i];
        const std::string ctx = "engines[" + std::to_string(i) + "]";

        if (!e.is_object())
            throw std::invalid_argument(ctx + ": each engine entry must be a JSON object");

        warnUnknownKeys(e, knownKeys, ctx, warnings);

        Vehicle::StageConfig sc;
        sc.stage              = requireField<int>        (e, "stage",          ctx);
        sc.numberOfEngine     = requireField<int>        (e, "numberOfEngine", ctx);
        sc.engine.propellant  = requireField<std::string>(e, "propellant",     ctx);
        sc.engine.nozzleType  = requireField<std::string>(e, "nozzleType",     ctx);
        sc.engine.thrust      = requireField<double>     (e, "thrustPerEngine",ctx);
        sc.engine.isp         = requireField<double>     (e, "ISP",            ctx);
        sc.engine.mdot        = requireField<double>     (e, "massFlowRate",   ctx);
        sc.engine.nozzleExitArea = requireField<double>  (e, "exitArea",       ctx);

        stages.push_back(sc);
    }

    return stages;
}

static Aerodynamics::Config parseAerodynamics(const json& j, std::vector<std::string>& warnings) {
    const std::string ctx = "aerodynamics";

    static const std::vector<std::string> knownKeys = {
        "refArea", "refLength", "machMin", "machMax", "machTransition"
    };
    warnUnknownKeys(j, knownKeys, ctx, warnings);

    Aerodynamics::Config cfg;
    cfg.refArea        = requireField<double>(j, "refArea",        ctx);
    cfg.refLength      = requireField<double>(j, "refLength",      ctx);
    cfg.machMin        = requireField<double>(j, "machMin",        ctx);
    cfg.machMax        = requireField<double>(j, "machMax",        ctx);
    cfg.machTransition = requireField<double>(j, "machTransition", ctx);

    cfg.validate();
    return cfg;
}

static SimConfig parseSimulation(const json& j, std::vector<std::string>& warnings) {
    const std::string ctx = "simulation";

    static const std::vector<std::string> knownKeys = {
        "timeStepRK4", "tolerance"
    };
    warnUnknownKeys(j, knownKeys, ctx, warnings);

    SimConfig cfg;
    cfg.timeStepRK4 = requireField<double>(j, "timeStepRK4", ctx);
    cfg.tolerance   = requireField<double>(j, "tolerance",   ctx);

    cfg.validate();
    return cfg;
}

static std::vector<Guidance::Config> parseGuidance(const json& arr, std::vector<std::string>& warnings) {
    static const std::vector<std::string> knownStageKeys = {
        "stage", "tolerance", "maxSteeringRate", "algorithms"
    };
    static const std::vector<std::string> knownIgmKeys = {
        "type", "guidanceCycle", "steeringHoldTime",
        "maxConvergenceIterations", "timeToGoConvergenceTolerance", "cutoffCriteria"
    };
    static const std::vector<std::string> knownOpenLoopKeys = {
        "type", "cutoffCriteria"
    };
    static const std::vector<std::string> knownIgmCutoffKeys = {
        "inclinationTolerance", "eccentricityTolerance"
    };
    static const std::vector<std::string> knownOpenLoopCutoffKeys = {
        "range", "altitude"
    };

    std::vector<Guidance::Config> configs;
    configs.reserve(arr.size());

    for (size_t i = 0; i < arr.size(); ++i) {
        const auto& g = arr[i];
        const std::string ctx = "guidance[" + std::to_string(i) + "]";

        if (!g.is_object())
            throw std::invalid_argument(ctx + ": each guidance entry must be a JSON object");

        warnUnknownKeys(g, knownStageKeys, ctx, warnings);

        Guidance::Config cfg;
        cfg.stage = requireField<int>(g, "stage", ctx);
        cfg.tolerance = requireField<double>(g, "tolerance", ctx);
        cfg.maxSteeringRate = requireField<double>(g, "maxSteeringRate", ctx);

        if (!g.contains("algorithms") || !g.at("algorithms").is_array() || g.at("algorithms").empty())
            throw std::invalid_argument(ctx + ": 'algorithms' must be a non-empty JSON array");

        const auto& algoArr = g.at("algorithms");
        for (size_t j = 0; j < algoArr.size(); ++j) {
            const auto& a = algoArr[j];
            const std::string actx = ctx + ".algorithms[" + std::to_string(j) + "]";

            if (!a.is_object())
                throw std::invalid_argument(actx + ": each algorithm entry must be a JSON object");

            std::string type = requireField<std::string>(a, "type", actx);
            validateAlgorithmType(type, actx);

            Guidance::AlgorithmEntry entry;
            entry.type = type;

            if (type == "IterativeGuidance") {
                warnUnknownKeys(a, knownIgmKeys, actx, warnings);
                entry.igmConfig.guidanceCycle = requireField<double>(a, "guidanceCycle", actx);
                entry.igmConfig.steeringHoldTime = requireField<double>(a, "steeringHoldTime", actx);
                entry.igmConfig.maxConvergenceIterations = requireField<int>(a, "maxConvergenceIterations", actx);
                entry.igmConfig.timeToGoConvergenceTolerance = requireField<double>(a, "timeToGoConvergenceTolerance", actx);

                if (a.contains("cutoffCriteria")) {
                    const auto& cc = a.at("cutoffCriteria");
                    const std::string cctx = actx + ".cutoffCriteria";
                    warnUnknownKeys(cc, knownIgmCutoffKeys, cctx, warnings);
                    entry.igmCutoffCriteria.inclinationTolerance = requireField<double>(cc, "inclinationTolerance", cctx);
                    entry.igmCutoffCriteria.eccentricityTolerance = requireField<double>(cc, "eccentricityTolerance", cctx);
                }
            } else if (type == "OpenLoopGuidance") {
                warnUnknownKeys(a, knownOpenLoopKeys, actx, warnings);
                entry.openLoopConfig.tolerance = cfg.tolerance;

                if (a.contains("cutoffCriteria")) {
                    const auto& cc = a.at("cutoffCriteria");
                    const std::string cctx = actx + ".cutoffCriteria";
                    warnUnknownKeys(cc, knownOpenLoopCutoffKeys, cctx, warnings);
                    entry.openLoopCutoffCriteria.range = cc.contains("range") ? cc.at("range").get<double>() : 0.0;
                    entry.openLoopCutoffCriteria.altitude = cc.contains("altitude") ? cc.at("altitude").get<double>() : 0.0;
                }
            }

            cfg.algorithms.push_back(std::move(entry));
        }

        cfg.validate();
        configs.push_back(std::move(cfg));
    }

    return configs;
}

static ReferenceMission parseMission(const json& j, std::vector<std::string>& warnings) {
    const std::string ctx = "mission";

    static const std::vector<std::string> knownKeys = {
        "semiMajorAxis", "longitudeAscendingNode", "inclination", "eccentricity",
        "trueAnomaly", "argumentOfPeriapsis", "ldnLaunchsite", "lanLaunchsite",
        "aimingAzimuth", "latitude", "geocentricLatitude", "launchSiteLongitude",
        "heightLaunchSite", "velocityTerminal", "positionTerminal",
        "initialSteeringAngles", "initialTime", "cutoffTime"
    };
    warnUnknownKeys(j, knownKeys, ctx, warnings);

    ReferenceMission m;

    m.semiMajorAxis          = requireField<double>(j, "semiMajorAxis",          ctx);
    m.longitudeAscendingNode = requireField<double>(j, "longitudeAscendingNode", ctx) * degToRad;
    m.inclination            = requireField<double>(j, "inclination",            ctx) * degToRad;
    m.eccentricity           = requireField<double>(j, "eccentricity",           ctx);
    m.trueAnomaly            = requireField<double>(j, "trueAnomaly",            ctx) * degToRad;
    m.argumentOfPeriapsis    = requireField<double>(j, "argumentOfPeriapsis",    ctx) * degToRad;
    m.ldnLaunchsite          = requireField<double>(j, "ldnLaunchsite",          ctx) * degToRad;
    m.lanLaunchsite          = requireField<double>(j, "lanLaunchsite",          ctx) * degToRad;
    m.aimingAzimuth          = requireField<double>(j, "aimingAzimuth",          ctx) * degToRad;
    m.latitude               = requireField<double>(j, "latitude",              ctx) * degToRad;
    m.geocentricLatitude     = requireField<double>(j, "geocentricLatitude",     ctx) * degToRad;
    m.launchSiteLongitude    = requireField<double>(j, "launchSiteLongitude",    ctx) * degToRad;
    m.heightLaunchSite       = requireField<double>(j, "heightLaunchSite",       ctx);

    m.velocityTerminal      = requireVec3(j, "velocityTerminal",  ctx);
    m.positionTerminal      = requireVec3(j, "positionTerminal",  ctx);
    m.initialSteeringAngles = requireSteeringAnglesDeg(j, "initialSteeringAngles", ctx);

    m.initialTime  = requireField<double>(j, "initialTime",  ctx);
    m.cutoffTime   = requireField<double>(j, "cutoffTime",   ctx);

    return m;
}

// =============================================================================
// Public API
// =============================================================================

ConfigResult<AppConfig> loadConfig(const std::string& path) {
    const json root = openJson(path);

    static const std::vector<std::string> knownSections = {
        "vehicle", "engines", "aerodynamics", "simulation", "guidance", "mission"
    };

    ConfigResult<AppConfig> result;
    warnUnknownKeys(root, knownSections, "Config", result.warnings);

    result.config.vehicle      = parseVehicle     (requireSection(root, "vehicle",      "Config"), result.warnings);

    // Engines array
    if (!root.contains("engines"))
        throw std::invalid_argument("Config: required section 'engines' is missing");
    if (!root.at("engines").is_array() || root.at("engines").empty())
        throw std::invalid_argument("Config: 'engines' must be a non-empty JSON array");
    result.config.vehicle.stages = parseEngines(root.at("engines"), result.warnings);

    // Derive numberOfStage from engines array
    result.config.vehicle.numberOfStage = static_cast<double>(result.config.vehicle.stages.size());

    result.config.aerodynamics = parseAerodynamics(requireSection(root, "aerodynamics", "Config"), result.warnings);
    result.config.simulation   = parseSimulation  (requireSection(root, "simulation",   "Config"), result.warnings);

    // Guidance array (builds vectors from per-stage entries)
    if (!root.contains("guidance"))
        throw std::invalid_argument("Config: required section 'guidance' is missing");
    if (!root.at("guidance").is_array() || root.at("guidance").empty())
        throw std::invalid_argument("Config: 'guidance' must be a non-empty JSON array");
    result.config.guidance = parseGuidance(root.at("guidance"), result.warnings);

    result.config.mission      = parseMission     (requireSection(root, "mission",      "Config"), result.warnings);

    return result;
}

} // namespace trajsim
