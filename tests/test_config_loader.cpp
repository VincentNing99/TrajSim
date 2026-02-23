// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Vincent Ning

/// @file test_config_loader.cpp
/// @brief Unit tests for the JSON configuration loader.
///
/// Tests cover:
///   - Successful loading of valid config files
///   - Missing file and malformed JSON error handling
///   - Missing required sections and fields
///   - Unknown key warnings (non-fatal diagnostics)
///   - Invalid values (negative mass, bad Mach ranges, etc.)
///   - Degree-to-radian conversion of angular fields

#include <gtest/gtest.h>
#include <fstream>
#include <filesystem>
#include <cmath>
#include <numbers>
#include "config/config_loader.hpp"

namespace trajsim::test {

// =============================================================================
// Helpers
// =============================================================================

/// @brief Writes a string to a temporary JSON file, returning its path.
static std::string writeTempJson(const std::string& content, const std::string& name) {
    std::string path = std::filesystem::temp_directory_path() / ("trajsim_test_" + name + ".json");
    std::ofstream f(path);
    f << content;
    f.close();
    return path;
}

/// @brief Removes a temporary file if it exists.
static void removeTempJson(const std::string& path) {
    std::filesystem::remove(path);
}

/// @brief Minimal valid JSON config matching the loader's required fields.
static const char* MINIMAL_VALID_JSON = R"({
    "vehicle": { "mass": 1000.0 },
    "engines": [
        {
            "stage": 1,
            "propellant": "liquid",
            "nozzleType": "vacuum",
            "numberOfEngine": 1,
            "thrustPerEngine": 5000.0,
            "ISP": 300.0,
            "massFlowRate": 1.7,
            "exitArea": 0.1
        }
    ],
    "aerodynamics": {
        "refArea": 1.0,
        "refLength": 1.0,
        "machMin": 0.1,
        "machMax": 5.0,
        "machTransition": 0.8
    },
    "simulation": {
        "timeStepRK4": 0.001,
        "tolerance": 1e-10,
        "igmStopTime": 2.0
    },
    "guidance": [
        {
            "stage": 1,
            "mode": "IGM",
            "tolerance": 1e-10,
            "maxSteeringRate": 5.0,
            "guidanceCycle": 0.01,
            "inclinationTolerance": 0.07,
            "eccentricityTolerance": 0.005,
            "steeringHoldTime": 15.0,
            "maxConvergenceIterations": 100,
            "timeToGoConvergenceTolerance": 1e-5
        }
    ],
    "mission": {
        "semiMajorAxis": 6903085.0,
        "longitudeAscendingNode": 260.0,
        "inclination": 97.5,
        "eccentricity": 1e-5,
        "trueAnomaly": 154.0,
        "argumentOfPeriapsis": 185.0,
        "ldnLaunchsite": 264.0,
        "lanLaunchsite": 84.0,
        "aimingAzimuth": 191.0,
        "latitude": 40.8,
        "geocentricLatitude": 40.6,
        "launchSiteLongitude": 100.0,
        "heightLaunchSite": 1000.0,
        "velocityTerminal": [-7124.0, 2631.0, 249.0],
        "positionTerminal": [-2405466.0, -12845047.0, 205013.0],
        "initialSteeringAngles": [172.0, 0.0, 0.0],
        "initialTime": 3140.0,
        "cutoffTime": 3344.0
    }
})";

// =============================================================================
// Test Fixture
// =============================================================================

class ConfigLoaderTest : public ::testing::Test {
protected:
    void TearDown() override {
        for (const auto& p : tempFiles)
            removeTempJson(p);
    }

    std::string writeTemp(const std::string& content, const std::string& name) {
        auto path = writeTempJson(content, name);
        tempFiles.push_back(path);
        return path;
    }

    std::vector<std::string> tempFiles;
};

// =============================================================================
// Successful Loading
// =============================================================================

TEST_F(ConfigLoaderTest, LoadsMinimalValidConfig) {
    auto path = writeTemp(MINIMAL_VALID_JSON, "valid_minimal");
    EXPECT_NO_THROW(loadConfig(path));
}

TEST_F(ConfigLoaderTest, LoadedValuesMatchInput) {
    auto path = writeTemp(MINIMAL_VALID_JSON, "valid_values");
    auto result = loadConfig(path);
    const auto& cfg = result.config;

    EXPECT_DOUBLE_EQ(cfg.vehicle.mass, 1000.0);
    EXPECT_EQ(cfg.vehicle.stages.size(), 1u);
    EXPECT_DOUBLE_EQ(cfg.vehicle.stages[0].engine.thrust, 5000.0);
    EXPECT_DOUBLE_EQ(cfg.vehicle.stages[0].engine.isp, 300.0);
    EXPECT_EQ(cfg.vehicle.stages[0].engine.propellant, "liquid");
    EXPECT_EQ(cfg.vehicle.stages[0].engine.nozzleType, "vacuum");
    EXPECT_DOUBLE_EQ(cfg.aerodynamics.refArea, 1.0);
    EXPECT_DOUBLE_EQ(cfg.simulation.timeStepRK4, 0.001);
    EXPECT_DOUBLE_EQ(cfg.mission.semiMajorAxis, 6903085.0);
}

TEST_F(ConfigLoaderTest, NumberOfStagesDerivedFromEngines) {
    auto path = writeTemp(MINIMAL_VALID_JSON, "stages");
    auto result = loadConfig(path);

    // Single engine entry → numberOfStage = 1
    EXPECT_DOUBLE_EQ(result.config.vehicle.numberOfStage, 1.0);
}

TEST_F(ConfigLoaderTest, AngularFieldsConvertedToRadians) {
    auto path = writeTemp(MINIMAL_VALID_JSON, "radians");
    auto result = loadConfig(path);
    const auto& m = result.config.mission;

    // inclination: 97.5° → ~1.7017 rad
    EXPECT_NEAR(m.inclination, 97.5 * std::numbers::pi / 180.0, 1e-10);
    // latitude: 40.8° → rad
    EXPECT_NEAR(m.latitude, 40.8 * std::numbers::pi / 180.0, 1e-10);
    // initialSteeringAngles: 172° → rad
    EXPECT_NEAR(m.initialSteeringAngles.phi, 172.0 * std::numbers::pi / 180.0, 1e-10);
}

TEST_F(ConfigLoaderTest, NoWarningsForCleanConfig) {
    auto path = writeTemp(MINIMAL_VALID_JSON, "no_warnings");
    auto result = loadConfig(path);

    EXPECT_FALSE(result.hasWarnings());
}

// =============================================================================
// Loading Real Config Files
// =============================================================================

TEST_F(ConfigLoaderTest, LoadsMission1Config) {
    auto result = loadConfig("../config/config_mission_1.json");
    const auto& cfg = result.config;

    EXPECT_DOUBLE_EQ(cfg.vehicle.mass, 50000.0);
    EXPECT_EQ(cfg.vehicle.stages.size(), 1u);
    EXPECT_DOUBLE_EQ(cfg.vehicle.stages[0].engine.thrust, 3000.0);
    EXPECT_DOUBLE_EQ(cfg.vehicle.stages[0].engine.isp, 315.0);
    EXPECT_DOUBLE_EQ(cfg.mission.semiMajorAxis, 6903085.0);
    EXPECT_DOUBLE_EQ(cfg.mission.heightLaunchSite, 1000.0);
}

TEST_F(ConfigLoaderTest, LoadsMission2Config) {
    auto result = loadConfig("../config/config_mission_2.json");
    const auto& cfg = result.config;

    EXPECT_DOUBLE_EQ(cfg.vehicle.mass, 108936.6);
    EXPECT_DOUBLE_EQ(cfg.mission.semiMajorAxis, 6740599.4);
}

// =============================================================================
// File Errors
// =============================================================================

TEST_F(ConfigLoaderTest, ThrowsOnMissingFile) {
    EXPECT_THROW(loadConfig("nonexistent_file.json"), std::runtime_error);
}

TEST_F(ConfigLoaderTest, ThrowsOnMalformedJson) {
    auto path = writeTemp("{ not valid json }", "malformed");
    EXPECT_THROW(loadConfig(path), std::runtime_error);
}

TEST_F(ConfigLoaderTest, ThrowsOnEmptyFile) {
    auto path = writeTemp("", "empty");
    EXPECT_THROW(loadConfig(path), std::runtime_error);
}

// =============================================================================
// Missing Required Sections
// =============================================================================

TEST_F(ConfigLoaderTest, ThrowsOnMissingVehicleSection) {
    auto path = writeTemp(R"({
        "engines": [{"stage":1,"propellant":"liquid","nozzleType":"vacuum",
                      "numberOfEngine":1,"thrustPerEngine":5000,"ISP":300,
                      "massFlowRate":1.7,"exitArea":0.1}],
        "aerodynamics": {"refArea":1,"refLength":1,"machMin":0.1,"machMax":5,"machTransition":0.8},
        "simulation": {"timeStepRK4":0.001,"tolerance":1e-10,"igmStopTime":2},
        "guidance": [{"stage":1,"mode":"IGM","tolerance":1e-10,"maxSteeringRate":5,
                      "guidanceCycle":0.01,"inclinationTolerance":0.07,
                      "eccentricityTolerance":0.005,"steeringHoldTime":15,
                      "maxConvergenceIterations":100,"timeToGoConvergenceTolerance":1e-5}],
        "mission": {"semiMajorAxis":6903085,"longitudeAscendingNode":260,"inclination":97.5,
                     "eccentricity":1e-5,"trueAnomaly":154,"argumentOfPeriapsis":185,
                     "ldnLaunchsite":264,"lanLaunchsite":84,"aimingAzimuth":191,
                     "latitude":40.8,"geocentricLatitude":40.6,"launchSiteLongitude":100,
                     "heightLaunchSite":1000,"velocityTerminal":[-7124,2631,249],
                     "positionTerminal":[-2405466,-12845047,205013],
                     "initialSteeringAngles":[172,0,0],"initialTime":3140,"cutoffTime":3344}
    })", "no_vehicle");

    EXPECT_THROW(loadConfig(path), std::invalid_argument);
}

TEST_F(ConfigLoaderTest, ThrowsOnMissingEnginesSection) {
    auto path = writeTemp(R"({
        "vehicle": { "mass": 1000.0 },
        "aerodynamics": {"refArea":1,"refLength":1,"machMin":0.1,"machMax":5,"machTransition":0.8},
        "simulation": {"timeStepRK4":0.001,"tolerance":1e-10,"igmStopTime":2},
        "guidance": [{"stage":1,"mode":"IGM","tolerance":1e-10,"maxSteeringRate":5,
                      "guidanceCycle":0.01,"inclinationTolerance":0.07,
                      "eccentricityTolerance":0.005,"steeringHoldTime":15,
                      "maxConvergenceIterations":100,"timeToGoConvergenceTolerance":1e-5}],
        "mission": {"semiMajorAxis":6903085,"longitudeAscendingNode":260,"inclination":97.5,
                     "eccentricity":1e-5,"trueAnomaly":154,"argumentOfPeriapsis":185,
                     "ldnLaunchsite":264,"lanLaunchsite":84,"aimingAzimuth":191,
                     "latitude":40.8,"geocentricLatitude":40.6,"launchSiteLongitude":100,
                     "heightLaunchSite":1000,"velocityTerminal":[-7124,2631,249],
                     "positionTerminal":[-2405466,-12845047,205013],
                     "initialSteeringAngles":[172,0,0],"initialTime":3140,"cutoffTime":3344}
    })", "no_engines");

    EXPECT_THROW(loadConfig(path), std::invalid_argument);
}

TEST_F(ConfigLoaderTest, ThrowsOnMissingMissionSection) {
    auto path = writeTemp(R"({
        "vehicle": { "mass": 1000.0 },
        "engines": [{"stage":1,"propellant":"liquid","nozzleType":"vacuum",
                      "numberOfEngine":1,"thrustPerEngine":5000,"ISP":300,
                      "massFlowRate":1.7,"exitArea":0.1}],
        "aerodynamics": {"refArea":1,"refLength":1,"machMin":0.1,"machMax":5,"machTransition":0.8},
        "simulation": {"timeStepRK4":0.001,"tolerance":1e-10,"igmStopTime":2},
        "guidance": [{"stage":1,"mode":"IGM","tolerance":1e-10,"maxSteeringRate":5,
                      "guidanceCycle":0.01,"inclinationTolerance":0.07,
                      "eccentricityTolerance":0.005,"steeringHoldTime":15,
                      "maxConvergenceIterations":100,"timeToGoConvergenceTolerance":1e-5}]
    })", "no_mission");

    EXPECT_THROW(loadConfig(path), std::invalid_argument);
}

// =============================================================================
// Missing Required Fields
// =============================================================================

TEST_F(ConfigLoaderTest, ThrowsOnMissingVehicleMass) {
    auto path = writeTemp(R"({
        "vehicle": {},
        "engines": [{"stage":1,"propellant":"liquid","nozzleType":"vacuum",
                      "numberOfEngine":1,"thrustPerEngine":5000,"ISP":300,
                      "massFlowRate":1.7,"exitArea":0.1}],
        "aerodynamics": {"refArea":1,"refLength":1,"machMin":0.1,"machMax":5,"machTransition":0.8},
        "simulation": {"timeStepRK4":0.001,"tolerance":1e-10,"igmStopTime":2},
        "guidance": [{"stage":1,"mode":"IGM","tolerance":1e-10,"maxSteeringRate":5,
                      "guidanceCycle":0.01,"inclinationTolerance":0.07,
                      "eccentricityTolerance":0.005,"steeringHoldTime":15,
                      "maxConvergenceIterations":100,"timeToGoConvergenceTolerance":1e-5}],
        "mission": {"semiMajorAxis":6903085,"longitudeAscendingNode":260,"inclination":97.5,
                     "eccentricity":1e-5,"trueAnomaly":154,"argumentOfPeriapsis":185,
                     "ldnLaunchsite":264,"lanLaunchsite":84,"aimingAzimuth":191,
                     "latitude":40.8,"geocentricLatitude":40.6,"launchSiteLongitude":100,
                     "heightLaunchSite":1000,"velocityTerminal":[-7124,2631,249],
                     "positionTerminal":[-2405466,-12845047,205013],
                     "initialSteeringAngles":[172,0,0],"initialTime":3140,"cutoffTime":3344}
    })", "no_mass");

    EXPECT_THROW(loadConfig(path), std::invalid_argument);
}

TEST_F(ConfigLoaderTest, ThrowsOnMissingEngineISP) {
    auto path = writeTemp(R"({
        "vehicle": { "mass": 1000 },
        "engines": [{"stage":1,"propellant":"liquid","nozzleType":"vacuum",
                      "numberOfEngine":1,"thrustPerEngine":5000,
                      "massFlowRate":1.7,"exitArea":0.1}],
        "aerodynamics": {"refArea":1,"refLength":1,"machMin":0.1,"machMax":5,"machTransition":0.8},
        "simulation": {"timeStepRK4":0.001,"tolerance":1e-10,"igmStopTime":2},
        "guidance": [{"stage":1,"mode":"IGM","tolerance":1e-10,"maxSteeringRate":5,
                      "guidanceCycle":0.01,"inclinationTolerance":0.07,
                      "eccentricityTolerance":0.005,"steeringHoldTime":15,
                      "maxConvergenceIterations":100,"timeToGoConvergenceTolerance":1e-5}],
        "mission": {"semiMajorAxis":6903085,"longitudeAscendingNode":260,"inclination":97.5,
                     "eccentricity":1e-5,"trueAnomaly":154,"argumentOfPeriapsis":185,
                     "ldnLaunchsite":264,"lanLaunchsite":84,"aimingAzimuth":191,
                     "latitude":40.8,"geocentricLatitude":40.6,"launchSiteLongitude":100,
                     "heightLaunchSite":1000,"velocityTerminal":[-7124,2631,249],
                     "positionTerminal":[-2405466,-12845047,205013],
                     "initialSteeringAngles":[172,0,0],"initialTime":3140,"cutoffTime":3344}
    })", "no_isp");

    EXPECT_THROW(loadConfig(path), std::invalid_argument);
}

// =============================================================================
// Invalid Values
// =============================================================================

TEST_F(ConfigLoaderTest, ThrowsOnNegativeTimeStep) {
    auto path = writeTemp(R"({
        "vehicle": { "mass": 1000 },
        "engines": [{"stage":1,"propellant":"liquid","nozzleType":"vacuum",
                      "numberOfEngine":1,"thrustPerEngine":5000,"ISP":300,
                      "massFlowRate":1.7,"exitArea":0.1}],
        "aerodynamics": {"refArea":1,"refLength":1,"machMin":0.1,"machMax":5,"machTransition":0.8},
        "simulation": {"timeStepRK4":-0.001,"tolerance":1e-10,"igmStopTime":2},
        "guidance": [{"stage":1,"mode":"IGM","tolerance":1e-10,"maxSteeringRate":5,
                      "guidanceCycle":0.01,"inclinationTolerance":0.07,
                      "eccentricityTolerance":0.005,"steeringHoldTime":15,
                      "maxConvergenceIterations":100,"timeToGoConvergenceTolerance":1e-5}],
        "mission": {"semiMajorAxis":6903085,"longitudeAscendingNode":260,"inclination":97.5,
                     "eccentricity":1e-5,"trueAnomaly":154,"argumentOfPeriapsis":185,
                     "ldnLaunchsite":264,"lanLaunchsite":84,"aimingAzimuth":191,
                     "latitude":40.8,"geocentricLatitude":40.6,"launchSiteLongitude":100,
                     "heightLaunchSite":1000,"velocityTerminal":[-7124,2631,249],
                     "positionTerminal":[-2405466,-12845047,205013],
                     "initialSteeringAngles":[172,0,0],"initialTime":3140,"cutoffTime":3344}
    })", "neg_timestep");

    EXPECT_THROW(loadConfig(path), std::invalid_argument);
}

TEST_F(ConfigLoaderTest, ThrowsOnInvalidGuidanceMode) {
    auto path = writeTemp(R"({
        "vehicle": { "mass": 1000 },
        "engines": [{"stage":1,"propellant":"liquid","nozzleType":"vacuum",
                      "numberOfEngine":1,"thrustPerEngine":5000,"ISP":300,
                      "massFlowRate":1.7,"exitArea":0.1}],
        "aerodynamics": {"refArea":1,"refLength":1,"machMin":0.1,"machMax":5,"machTransition":0.8},
        "simulation": {"timeStepRK4":0.001,"tolerance":1e-10,"igmStopTime":2},
        "guidance": [{"stage":1,"mode":"INVALID_MODE","tolerance":1e-10,"maxSteeringRate":5,
                      "guidanceCycle":0.01,"inclinationTolerance":0.07,
                      "eccentricityTolerance":0.005,"steeringHoldTime":15,
                      "maxConvergenceIterations":100,"timeToGoConvergenceTolerance":1e-5}],
        "mission": {"semiMajorAxis":6903085,"longitudeAscendingNode":260,"inclination":97.5,
                     "eccentricity":1e-5,"trueAnomaly":154,"argumentOfPeriapsis":185,
                     "ldnLaunchsite":264,"lanLaunchsite":84,"aimingAzimuth":191,
                     "latitude":40.8,"geocentricLatitude":40.6,"launchSiteLongitude":100,
                     "heightLaunchSite":1000,"velocityTerminal":[-7124,2631,249],
                     "positionTerminal":[-2405466,-12845047,205013],
                     "initialSteeringAngles":[172,0,0],"initialTime":3140,"cutoffTime":3344}
    })", "bad_mode");

    EXPECT_THROW(loadConfig(path), std::invalid_argument);
}

TEST_F(ConfigLoaderTest, ThrowsOnEmptyEnginesArray) {
    auto path = writeTemp(R"({
        "vehicle": { "mass": 1000 },
        "engines": [],
        "aerodynamics": {"refArea":1,"refLength":1,"machMin":0.1,"machMax":5,"machTransition":0.8},
        "simulation": {"timeStepRK4":0.001,"tolerance":1e-10,"igmStopTime":2},
        "guidance": [{"stage":1,"mode":"IGM","tolerance":1e-10,"maxSteeringRate":5,
                      "guidanceCycle":0.01,"inclinationTolerance":0.07,
                      "eccentricityTolerance":0.005,"steeringHoldTime":15,
                      "maxConvergenceIterations":100,"timeToGoConvergenceTolerance":1e-5}],
        "mission": {"semiMajorAxis":6903085,"longitudeAscendingNode":260,"inclination":97.5,
                     "eccentricity":1e-5,"trueAnomaly":154,"argumentOfPeriapsis":185,
                     "ldnLaunchsite":264,"lanLaunchsite":84,"aimingAzimuth":191,
                     "latitude":40.8,"geocentricLatitude":40.6,"launchSiteLongitude":100,
                     "heightLaunchSite":1000,"velocityTerminal":[-7124,2631,249],
                     "positionTerminal":[-2405466,-12845047,205013],
                     "initialSteeringAngles":[172,0,0],"initialTime":3140,"cutoffTime":3344}
    })", "empty_engines");

    EXPECT_THROW(loadConfig(path), std::invalid_argument);
}

TEST_F(ConfigLoaderTest, ThrowsOnNegativeRefArea) {
    auto path = writeTemp(R"({
        "vehicle": { "mass": 1000 },
        "engines": [{"stage":1,"propellant":"liquid","nozzleType":"vacuum",
                      "numberOfEngine":1,"thrustPerEngine":5000,"ISP":300,
                      "massFlowRate":1.7,"exitArea":0.1}],
        "aerodynamics": {"refArea":-1.0,"refLength":1,"machMin":0.1,"machMax":5,"machTransition":0.8},
        "simulation": {"timeStepRK4":0.001,"tolerance":1e-10,"igmStopTime":2},
        "guidance": [{"stage":1,"mode":"IGM","tolerance":1e-10,"maxSteeringRate":5,
                      "guidanceCycle":0.01,"inclinationTolerance":0.07,
                      "eccentricityTolerance":0.005,"steeringHoldTime":15,
                      "maxConvergenceIterations":100,"timeToGoConvergenceTolerance":1e-5}],
        "mission": {"semiMajorAxis":6903085,"longitudeAscendingNode":260,"inclination":97.5,
                     "eccentricity":1e-5,"trueAnomaly":154,"argumentOfPeriapsis":185,
                     "ldnLaunchsite":264,"lanLaunchsite":84,"aimingAzimuth":191,
                     "latitude":40.8,"geocentricLatitude":40.6,"launchSiteLongitude":100,
                     "heightLaunchSite":1000,"velocityTerminal":[-7124,2631,249],
                     "positionTerminal":[-2405466,-12845047,205013],
                     "initialSteeringAngles":[172,0,0],"initialTime":3140,"cutoffTime":3344}
    })", "neg_refarea");

    EXPECT_THROW(loadConfig(path), std::invalid_argument);
}

// =============================================================================
// Unknown Key Warnings
// =============================================================================

TEST_F(ConfigLoaderTest, UnknownTopLevelKeyProducesWarning) {
    auto path = writeTemp(R"({
        "vehicle": { "mass": 1000 },
        "engines": [{"stage":1,"propellant":"liquid","nozzleType":"vacuum",
                      "numberOfEngine":1,"thrustPerEngine":5000,"ISP":300,
                      "massFlowRate":1.7,"exitArea":0.1}],
        "aerodynamics": {"refArea":1,"refLength":1,"machMin":0.1,"machMax":5,"machTransition":0.8},
        "simulation": {"timeStepRK4":0.001,"tolerance":1e-10,"igmStopTime":2},
        "guidance": [{"stage":1,"mode":"IGM","tolerance":1e-10,"maxSteeringRate":5,
                      "guidanceCycle":0.01,"inclinationTolerance":0.07,
                      "eccentricityTolerance":0.005,"steeringHoldTime":15,
                      "maxConvergenceIterations":100,"timeToGoConvergenceTolerance":1e-5}],
        "mission": {"semiMajorAxis":6903085,"longitudeAscendingNode":260,"inclination":97.5,
                     "eccentricity":1e-5,"trueAnomaly":154,"argumentOfPeriapsis":185,
                     "ldnLaunchsite":264,"lanLaunchsite":84,"aimingAzimuth":191,
                     "latitude":40.8,"geocentricLatitude":40.6,"launchSiteLongitude":100,
                     "heightLaunchSite":1000,"velocityTerminal":[-7124,2631,249],
                     "positionTerminal":[-2405466,-12845047,205013],
                     "initialSteeringAngles":[172,0,0],"initialTime":3140,"cutoffTime":3344},
        "unknownSection": { "foo": "bar" }
    })", "unknown_key");

    auto result = loadConfig(path);
    EXPECT_TRUE(result.hasWarnings());

    // Should contain a warning about "unknownSection"
    bool found = false;
    for (const auto& w : result.warnings) {
        if (w.find("unknownSection") != std::string::npos)
            found = true;
    }
    EXPECT_TRUE(found) << "Expected warning about 'unknownSection'";
}

TEST_F(ConfigLoaderTest, UnknownVehicleFieldProducesWarning) {
    auto path = writeTemp(R"({
        "vehicle": { "mass": 1000, "color": "red" },
        "engines": [{"stage":1,"propellant":"liquid","nozzleType":"vacuum",
                      "numberOfEngine":1,"thrustPerEngine":5000,"ISP":300,
                      "massFlowRate":1.7,"exitArea":0.1}],
        "aerodynamics": {"refArea":1,"refLength":1,"machMin":0.1,"machMax":5,"machTransition":0.8},
        "simulation": {"timeStepRK4":0.001,"tolerance":1e-10,"igmStopTime":2},
        "guidance": [{"stage":1,"mode":"IGM","tolerance":1e-10,"maxSteeringRate":5,
                      "guidanceCycle":0.01,"inclinationTolerance":0.07,
                      "eccentricityTolerance":0.005,"steeringHoldTime":15,
                      "maxConvergenceIterations":100,"timeToGoConvergenceTolerance":1e-5}],
        "mission": {"semiMajorAxis":6903085,"longitudeAscendingNode":260,"inclination":97.5,
                     "eccentricity":1e-5,"trueAnomaly":154,"argumentOfPeriapsis":185,
                     "ldnLaunchsite":264,"lanLaunchsite":84,"aimingAzimuth":191,
                     "latitude":40.8,"geocentricLatitude":40.6,"launchSiteLongitude":100,
                     "heightLaunchSite":1000,"velocityTerminal":[-7124,2631,249],
                     "positionTerminal":[-2405466,-12845047,205013],
                     "initialSteeringAngles":[172,0,0],"initialTime":3140,"cutoffTime":3344}
    })", "unknown_vehicle_field");

    auto result = loadConfig(path);
    EXPECT_TRUE(result.hasWarnings());
}

// =============================================================================
// Vec3 and SteeringAngles Field Parsing
// =============================================================================

TEST_F(ConfigLoaderTest, ThrowsOnVelocityTerminalWrongSize) {
    auto path = writeTemp(R"({
        "vehicle": { "mass": 1000 },
        "engines": [{"stage":1,"propellant":"liquid","nozzleType":"vacuum",
                      "numberOfEngine":1,"thrustPerEngine":5000,"ISP":300,
                      "massFlowRate":1.7,"exitArea":0.1}],
        "aerodynamics": {"refArea":1,"refLength":1,"machMin":0.1,"machMax":5,"machTransition":0.8},
        "simulation": {"timeStepRK4":0.001,"tolerance":1e-10,"igmStopTime":2},
        "guidance": [{"stage":1,"mode":"IGM","tolerance":1e-10,"maxSteeringRate":5,
                      "guidanceCycle":0.01,"inclinationTolerance":0.07,
                      "eccentricityTolerance":0.005,"steeringHoldTime":15,
                      "maxConvergenceIterations":100,"timeToGoConvergenceTolerance":1e-5}],
        "mission": {"semiMajorAxis":6903085,"longitudeAscendingNode":260,"inclination":97.5,
                     "eccentricity":1e-5,"trueAnomaly":154,"argumentOfPeriapsis":185,
                     "ldnLaunchsite":264,"lanLaunchsite":84,"aimingAzimuth":191,
                     "latitude":40.8,"geocentricLatitude":40.6,"launchSiteLongitude":100,
                     "heightLaunchSite":1000,"velocityTerminal":[-7124,2631],
                     "positionTerminal":[-2405466,-12845047,205013],
                     "initialSteeringAngles":[172,0,0],"initialTime":3140,"cutoffTime":3344}
    })", "bad_vec3_size");

    EXPECT_THROW(loadConfig(path), std::invalid_argument);
}

// =============================================================================
// Type Mismatch
// =============================================================================

TEST_F(ConfigLoaderTest, ThrowsOnWrongFieldType) {
    auto path = writeTemp(R"({
        "vehicle": { "mass": "not_a_number" },
        "engines": [{"stage":1,"propellant":"liquid","nozzleType":"vacuum",
                      "numberOfEngine":1,"thrustPerEngine":5000,"ISP":300,
                      "massFlowRate":1.7,"exitArea":0.1}],
        "aerodynamics": {"refArea":1,"refLength":1,"machMin":0.1,"machMax":5,"machTransition":0.8},
        "simulation": {"timeStepRK4":0.001,"tolerance":1e-10,"igmStopTime":2},
        "guidance": [{"stage":1,"mode":"IGM","tolerance":1e-10,"maxSteeringRate":5,
                      "guidanceCycle":0.01,"inclinationTolerance":0.07,
                      "eccentricityTolerance":0.005,"steeringHoldTime":15,
                      "maxConvergenceIterations":100,"timeToGoConvergenceTolerance":1e-5}],
        "mission": {"semiMajorAxis":6903085,"longitudeAscendingNode":260,"inclination":97.5,
                     "eccentricity":1e-5,"trueAnomaly":154,"argumentOfPeriapsis":185,
                     "ldnLaunchsite":264,"lanLaunchsite":84,"aimingAzimuth":191,
                     "latitude":40.8,"geocentricLatitude":40.6,"launchSiteLongitude":100,
                     "heightLaunchSite":1000,"velocityTerminal":[-7124,2631,249],
                     "positionTerminal":[-2405466,-12845047,205013],
                     "initialSteeringAngles":[172,0,0],"initialTime":3140,"cutoffTime":3344}
    })", "wrong_type");

    EXPECT_THROW(loadConfig(path), std::invalid_argument);
}

// =============================================================================
// ConfigResult Helper
// =============================================================================

TEST(ConfigResultTest, HasWarningsReturnsFalseWhenEmpty) {
    ConfigResult<int> r;
    r.config = 42;
    EXPECT_FALSE(r.hasWarnings());
}

TEST(ConfigResultTest, HasWarningsReturnsTrueWhenNonEmpty) {
    ConfigResult<int> r;
    r.config = 42;
    r.warnings.push_back("test warning");
    EXPECT_TRUE(r.hasWarnings());
}

}  // namespace trajsim::test
