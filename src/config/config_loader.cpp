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

/// @brief Recursively parse an exit expression tree node from JSON.
///
/// JSON shapes:
///   Leaf: { "semiMajorAxis": { "exceedsBy": 0 } }
///   And:  { "and": [ <node>, <node>, ... ] }
///   Or:   { "or":  [ <node>, <node>, ... ] }
static ExitNode parseExitNode(const json& j, const std::string& ctx) {
    if (!j.is_object())
        throw std::invalid_argument(ctx + ": exit node must be a JSON object");

    // Combinator nodes
    if (j.contains("and")) {
        const auto& arr = j.at("and");
        if (!arr.is_array() || arr.empty())
            throw std::invalid_argument(ctx + ".and: must be a non-empty JSON array");
        std::vector<ExitNode> children;
        children.reserve(arr.size());
        for (size_t k = 0; k < arr.size(); ++k)
            children.push_back(parseExitNode(arr[k], ctx + ".and[" + std::to_string(k) + "]"));
        return ExitNode::andOf(std::move(children));
    }
    if (j.contains("or")) {
        const auto& arr = j.at("or");
        if (!arr.is_array() || arr.empty())
            throw std::invalid_argument(ctx + ".or: must be a non-empty JSON array");
        std::vector<ExitNode> children;
        children.reserve(arr.size());
        for (size_t k = 0; k < arr.size(); ++k)
            children.push_back(parseExitNode(arr[k], ctx + ".or[" + std::to_string(k) + "]"));
        return ExitNode::orOf(std::move(children));
    }

    // Leaf node — find the parameter key
    static const std::vector<std::pair<std::string, ExitNode::Parameter>> paramKeys = {
        {"semiMajorAxis", ExitNode::Parameter::SemiMajorAxis},
        {"eccentricity",  ExitNode::Parameter::Eccentricity},
        {"inclination",   ExitNode::Parameter::Inclination},
        {"range",         ExitNode::Parameter::Range},
        {"altitude",      ExitNode::Parameter::Altitude}
    };

    for (const auto& [key, param] : paramKeys) {
        if (!j.contains(key)) continue;

        const auto& comp = j.at(key);
        if (!comp.is_object())
            throw std::invalid_argument(ctx + "." + key + ": must be a JSON object with 'withinTolerance' or 'exceedsBy'");

        bool hasWithin  = comp.contains("withinTolerance");
        bool hasExceeds = comp.contains("exceedsBy");
        if (hasWithin == hasExceeds)
            throw std::invalid_argument(ctx + "." + key + ": must specify exactly one of 'withinTolerance' or 'exceedsBy'");

        if (hasWithin)
            return ExitNode::leaf(param, ExitNode::Comparison::WithinTolerance,
                                    comp.at("withinTolerance").get<double>());
        else
            return ExitNode::leaf(param, ExitNode::Comparison::ExceedsBy,
                                    comp.at("exceedsBy").get<double>());
    }

    throw std::invalid_argument(ctx + ": exit node must contain 'and', 'or', or a parameter name "
                                "(semiMajorAxis, eccentricity, inclination, range, altitude)");
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

static Vehicle::StageCfg parseStageCfg(const json& stageCfgObj, std::vector<std::string>& warnings) {
    static const std::vector<std::string> stageKeys = {"stageMass", "numberOfEngine", "engine"};
    static const std::vector<std::string> engineKeys = {"propellant", "nozzleType", "thrust", "ISP", "massFlowRate", "exitArea"};

    const std::string ctx = "stageCfg";
    if (!stageCfgObj.is_object()) throw std::invalid_argument(ctx + " must be a JSON object");
    warnUnknownKeys(stageCfgObj, stageKeys, ctx, warnings);

    Vehicle::StageCfg stage;
    stage.numberOfEngine = requireField<std::vector<int>>(stageCfgObj, "numberOfEngine", ctx);
    stage.mass           = requireField<std::vector<double>>(stageCfgObj, "stageMass", ctx);

    if (!stageCfgObj.contains("engine") || !stageCfgObj.at("engine").is_array() || stageCfgObj.at("engine").empty())
        throw std::invalid_argument(ctx + ": 'engine' must be a non-empty JSON array");

    const auto& engineArr = stageCfgObj.at("engine");
    stage.engineCfg.reserve(engineArr.size());

    for (size_t i = 0; i < engineArr.size(); ++i) {
        const auto& engineObj = engineArr[i];
        const std::string engCtx = ctx + ".engine[" + std::to_string(i) + "]";

        if (!engineObj.is_object()) throw std::invalid_argument(engCtx + " must be a JSON object");
        warnUnknownKeys(engineObj, engineKeys, engCtx, warnings);

        Engine::Config cfg;
        cfg.propellant     = requireField<std::string>(engineObj, "propellant",   engCtx);
        cfg.nozzleType     = requireField<std::string>(engineObj, "nozzleType",   engCtx);
        cfg.thrust         = requireField<double>     (engineObj, "thrust",       engCtx);
        cfg.isp            = requireField<double>     (engineObj, "ISP",          engCtx);
        cfg.mdot           = requireField<double>     (engineObj, "massFlowRate", engCtx);
        cfg.nozzleExitArea = requireField<double>     (engineObj, "exitArea",     engCtx);

        stage.engineCfg.push_back(std::move(cfg));
    }

    return stage;
}

static Vehicle::Config parseVehicle(const json& j, std::vector<std::string>& warnings) {
    const std::string ctx = "vehicle";

    static const std::vector<std::string> knownKeys = {
        "mass", "stages", "aerodynamics", "stageCfg"
    };
    warnUnknownKeys(j, knownKeys, ctx, warnings);

    Vehicle::Config cfg;
    cfg.mass          = requireField<double>(j, "mass", ctx);
    cfg.numberOfStage = requireField<int>(j, "stages", ctx);
    cfg.aeroCfg       = parseAerodynamics(requireSection(j, "aerodynamics", ctx), warnings);
    cfg.stage         = parseStageCfg(requireSection(j, "stageCfg", ctx), warnings);

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
        "stage", "maxSteeringRate", "algorithms"
    };
    static const std::vector<std::string> knownIgmKeys = {
        "type", "guidanceCycle", "steeringHoldTime",
        "maxConvergenceIterations", "timeToGoConvergenceTolerance", "tolerance", "exitCriteria", "pitchCorrectionStopTime", "yawCorrectionStopTime"
    };
    static const std::vector<std::string> knownOpenLoopKeys = {
        "type", "tolerance"
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
                entry.igmConfig.tolerance = requireField<double>(a, "tolerance", actx);
                entry.igmConfig.pitchCorrectionStopTime = requireField<int>(a, "pitchCorrectionStopTime", actx);
                entry.igmConfig.yawCorrectionStopTime   = requireField<int>(a, "yawCorrectionStopTime",   actx);


                if (a.contains("exitCriteria")) {
                    const std::string cctx = actx + ".exitCriteria";
                    entry.exitCriteria.root = parseExitNode(a.at("exitCriteria"), cctx);
                }
            } else if (type == "OpenLoopGuidance") {
                warnUnknownKeys(a, knownOpenLoopKeys, actx, warnings);
                entry.openLoopConfig.tolerance = requireField<double>(a, "tolerance", actx);
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
        "initialPosition", "initialVelocity", "initialMass",
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
    m.initialPosition       = requireVec3(j, "initialPosition",   ctx);
    m.initialVelocity       = requireVec3(j, "initialVelocity",   ctx);
    m.initialMass           = requireField<double>(j, "initialMass", ctx);
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
        "vehicle", "simulation", "guidance", "mission"
    };

    ConfigResult<AppConfig> result;
    warnUnknownKeys(root, knownSections, "Config", result.warnings);

    result.config.vehicle      = parseVehicle     (requireSection(root, "vehicle",      "Config"), result.warnings);
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
