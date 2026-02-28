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

/// @brief Compact vehicle section for reuse in test JSON strings.
/// Contains: mass, stages, stageCfg (with engine array), aerodynamics.
static const char* VEHICLE_SECTION =
    R"("vehicle":{"mass":1000,"stages":1,)"
    R"("stageCfg":{"stageMass":[1000],"numberOfEngine":[1],)"
    R"("engine":[{"propellant":"liquid","nozzleType":"vacuum","thrust":5000,"ISP":300,"massFlowRate":1.7,"exitArea":0.1}]},)"
    R"("aerodynamics":{"refArea":1,"refLength":1,"machMin":0.1,"machMax":5,"machTransition":0.8}})";

static const char* SIMULATION_SECTION =
    R"("simulation":{"timeStepRK4":0.001,"tolerance":1e-10})";

static const char* GUIDANCE_SECTION =
    R"("guidance":[{"stage":1,"maxSteeringRate":5,"algorithms":[{"type":"IterativeGuidance","guidanceCycle":0.01,"steeringHoldTime":15,"maxConvergenceIterations":100,"timeToGoConvergenceTolerance":1e-5,"tolerance":1e-10,"pitchCorrectionStopTime":35,"yawCorrectionStopTime":15,"exitCriteria":{"and":[{"semiMajorAxis":{"exceedsBy":0}},{"eccentricity":{"withinTolerance":0.005}},{"inclination":{"withinTolerance":0.07}}]}}]}])";

static const char* MISSION_SECTION =
    R"("mission":{"semiMajorAxis":6903085,"longitudeAscendingNode":260,"inclination":97.5,)"
    R"("eccentricity":1e-5,"trueAnomaly":154,"argumentOfPeriapsis":185,)"
    R"("ldnLaunchsite":264,"lanLaunchsite":84,"aimingAzimuth":191,)"
    R"("latitude":40.8,"geocentricLatitude":40.6,"launchSiteLongitude":100,)"
    R"("heightLaunchSite":1000,"velocityTerminal":[-7124,2631,249],)"
    R"("positionTerminal":[-2405466,-12845047,205013],)"
    R"("initialPosition":[0,0,0],"initialVelocity":[0,0,0],"initialMass":1000,)"
    R"("initialSteeringAngles":[172,0,0],"initialTime":3140,"cutoffTime":3344})";

/// @brief Build a full valid JSON config by joining sections.
static std::string makeFullJson() {
    return std::string("{") + VEHICLE_SECTION + "," + SIMULATION_SECTION + ","
         + GUIDANCE_SECTION + "," + MISSION_SECTION + "}";
}

/// @brief Minimal valid JSON config matching the loader's required fields.
static const std::string MINIMAL_VALID_JSON = makeFullJson();

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
    EXPECT_EQ(cfg.vehicle.stage.engineCfg.size(), 1u);
    EXPECT_DOUBLE_EQ(cfg.vehicle.stage.engineCfg[0].thrust, 5000.0);
    EXPECT_DOUBLE_EQ(cfg.vehicle.stage.engineCfg[0].isp, 300.0);
    EXPECT_EQ(cfg.vehicle.stage.engineCfg[0].propellant, "liquid");
    EXPECT_EQ(cfg.vehicle.stage.engineCfg[0].nozzleType, "vacuum");
    EXPECT_DOUBLE_EQ(cfg.vehicle.aeroCfg.refArea, 1.0);
    EXPECT_DOUBLE_EQ(cfg.simulation.timeStepRK4, 0.001);
    EXPECT_DOUBLE_EQ(cfg.mission.semiMajorAxis, 6903085.0);
}

TEST_F(ConfigLoaderTest, NumberOfStagesDerivedFromConfig) {
    auto path = writeTemp(MINIMAL_VALID_JSON, "stages");
    auto result = loadConfig(path);

    EXPECT_EQ(result.config.vehicle.numberOfStage, 1);
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
    EXPECT_EQ(cfg.vehicle.stage.engineCfg.size(), 1u);
    EXPECT_DOUBLE_EQ(cfg.vehicle.stage.engineCfg[0].thrust, 3000.0);
    EXPECT_DOUBLE_EQ(cfg.vehicle.stage.engineCfg[0].isp, 315.0);
    EXPECT_DOUBLE_EQ(cfg.mission.semiMajorAxis, 6903085.0);
    EXPECT_DOUBLE_EQ(cfg.mission.heightLaunchSite, 1000.0);
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
    auto path = writeTemp(std::string("{") + SIMULATION_SECTION + ","
        + GUIDANCE_SECTION + "," + MISSION_SECTION + "}", "no_vehicle");
    EXPECT_THROW(loadConfig(path), std::invalid_argument);
}

TEST_F(ConfigLoaderTest, ThrowsOnMissingSimulationSection) {
    auto path = writeTemp(std::string("{") + VEHICLE_SECTION + ","
        + GUIDANCE_SECTION + "," + MISSION_SECTION + "}", "no_simulation");
    EXPECT_THROW(loadConfig(path), std::invalid_argument);
}

TEST_F(ConfigLoaderTest, ThrowsOnMissingMissionSection) {
    auto path = writeTemp(std::string("{") + VEHICLE_SECTION + ","
        + SIMULATION_SECTION + "," + GUIDANCE_SECTION + "}", "no_mission");
    EXPECT_THROW(loadConfig(path), std::invalid_argument);
}

// =============================================================================
// Missing Required Fields
// =============================================================================

TEST_F(ConfigLoaderTest, ThrowsOnMissingVehicleMass) {
    auto path = writeTemp(std::string(R"({"vehicle":{"stages":1,)")
        + R"("stageCfg":{"stageMass":[1000],"numberOfEngine":[1],)"
        + R"("engine":[{"propellant":"liquid","nozzleType":"vacuum","thrust":5000,"ISP":300,"massFlowRate":1.7,"exitArea":0.1}]},)"
        + R"("aerodynamics":{"refArea":1,"refLength":1,"machMin":0.1,"machMax":5,"machTransition":0.8}},)"
        + SIMULATION_SECTION + "," + GUIDANCE_SECTION + "," + MISSION_SECTION + "}", "no_mass");
    EXPECT_THROW(loadConfig(path), std::invalid_argument);
}

TEST_F(ConfigLoaderTest, ThrowsOnMissingEngineISP) {
    auto path = writeTemp(std::string(R"({"vehicle":{"mass":1000,"stages":1,)")
        + R"("stageCfg":{"stageMass":[1000],"numberOfEngine":[1],)"
        + R"("engine":[{"propellant":"liquid","nozzleType":"vacuum","thrust":5000,"massFlowRate":1.7,"exitArea":0.1}]},)"
        + R"("aerodynamics":{"refArea":1,"refLength":1,"machMin":0.1,"machMax":5,"machTransition":0.8}},)"
        + SIMULATION_SECTION + "," + GUIDANCE_SECTION + "," + MISSION_SECTION + "}", "no_isp");
    EXPECT_THROW(loadConfig(path), std::invalid_argument);
}

// =============================================================================
// Invalid Values
// =============================================================================

TEST_F(ConfigLoaderTest, ThrowsOnNegativeTimeStep) {
    auto path = writeTemp(std::string("{") + VEHICLE_SECTION + ","
        + R"("simulation":{"timeStepRK4":-0.001,"tolerance":1e-10},)"
        + GUIDANCE_SECTION + "," + MISSION_SECTION + "}", "neg_timestep");
    EXPECT_THROW(loadConfig(path), std::invalid_argument);
}

TEST_F(ConfigLoaderTest, ThrowsOnInvalidAlgorithmType) {
    auto path = writeTemp(std::string("{") + VEHICLE_SECTION + ","
        + SIMULATION_SECTION + ","
        + R"("guidance":[{"stage":1,"maxSteeringRate":5,"algorithms":[{"type":"INVALID_TYPE","guidanceCycle":0.01,"steeringHoldTime":15,"maxConvergenceIterations":100,"timeToGoConvergenceTolerance":1e-5,"tolerance":1e-10}]}],)"
        + MISSION_SECTION + "}", "bad_mode");
    EXPECT_THROW(loadConfig(path), std::invalid_argument);
}

TEST_F(ConfigLoaderTest, ThrowsOnMissingStageCfg) {
    auto path = writeTemp(std::string(R"({"vehicle":{"mass":1000,"stages":1,)")
        + R"("aerodynamics":{"refArea":1,"refLength":1,"machMin":0.1,"machMax":5,"machTransition":0.8}},)"
        + SIMULATION_SECTION + "," + GUIDANCE_SECTION + "," + MISSION_SECTION + "}", "no_stagecfg");
    EXPECT_THROW(loadConfig(path), std::invalid_argument);
}

TEST_F(ConfigLoaderTest, ThrowsOnEmptyEngineArray) {
    auto path = writeTemp(std::string(R"({"vehicle":{"mass":1000,"stages":1,)")
        + R"("stageCfg":{"stageMass":[1000],"numberOfEngine":[1],"engine":[]},)"
        + R"("aerodynamics":{"refArea":1,"refLength":1,"machMin":0.1,"machMax":5,"machTransition":0.8}},)"
        + SIMULATION_SECTION + "," + GUIDANCE_SECTION + "," + MISSION_SECTION + "}", "empty_engines");
    EXPECT_THROW(loadConfig(path), std::invalid_argument);
}

TEST_F(ConfigLoaderTest, ThrowsOnNegativeRefArea) {
    auto path = writeTemp(std::string(R"({"vehicle":{"mass":1000,"stages":1,)")
        + R"("stageCfg":{"stageMass":[1000],"numberOfEngine":[1],)"
        + R"("engine":[{"propellant":"liquid","nozzleType":"vacuum","thrust":5000,"ISP":300,"massFlowRate":1.7,"exitArea":0.1}]},)"
        + R"("aerodynamics":{"refArea":-1.0,"refLength":1,"machMin":0.1,"machMax":5,"machTransition":0.8}},)"
        + SIMULATION_SECTION + "," + GUIDANCE_SECTION + "," + MISSION_SECTION + "}", "neg_refarea");
    EXPECT_THROW(loadConfig(path), std::invalid_argument);
}

// =============================================================================
// Unknown Key Warnings
// =============================================================================

TEST_F(ConfigLoaderTest, UnknownTopLevelKeyProducesWarning) {
    auto path = writeTemp(std::string("{") + VEHICLE_SECTION + ","
        + SIMULATION_SECTION + "," + GUIDANCE_SECTION + "," + MISSION_SECTION
        + R"(,"unknownSection":{"foo":"bar"}})", "unknown_key");

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
    auto path = writeTemp(std::string(R"({"vehicle":{"mass":1000,"stages":1,"color":"red",)")
        + R"("stageCfg":{"stageMass":[1000],"numberOfEngine":[1],)"
        + R"("engine":[{"propellant":"liquid","nozzleType":"vacuum","thrust":5000,"ISP":300,"massFlowRate":1.7,"exitArea":0.1}]},)"
        + R"("aerodynamics":{"refArea":1,"refLength":1,"machMin":0.1,"machMax":5,"machTransition":0.8}},)"
        + SIMULATION_SECTION + "," + GUIDANCE_SECTION + "," + MISSION_SECTION + "}", "unknown_vehicle_field");

    auto result = loadConfig(path);
    EXPECT_TRUE(result.hasWarnings());
}

// =============================================================================
// Vec3 and SteeringAngles Field Parsing
// =============================================================================

TEST_F(ConfigLoaderTest, ThrowsOnVelocityTerminalWrongSize) {
    auto path = writeTemp(std::string("{") + VEHICLE_SECTION + ","
        + SIMULATION_SECTION + "," + GUIDANCE_SECTION + ","
        + R"("mission":{"semiMajorAxis":6903085,"longitudeAscendingNode":260,"inclination":97.5,)"
        + R"("eccentricity":1e-5,"trueAnomaly":154,"argumentOfPeriapsis":185,)"
        + R"("ldnLaunchsite":264,"lanLaunchsite":84,"aimingAzimuth":191,)"
        + R"("latitude":40.8,"geocentricLatitude":40.6,"launchSiteLongitude":100,)"
        + R"("heightLaunchSite":1000,"velocityTerminal":[-7124,2631],)"
        + R"("positionTerminal":[-2405466,-12845047,205013],)"
        + R"("initialPosition":[0,0,0],"initialVelocity":[0,0,0],"initialMass":1000,)"
        + R"("initialSteeringAngles":[172,0,0],"initialTime":3140,"cutoffTime":3344}})", "bad_vec3_size");
    EXPECT_THROW(loadConfig(path), std::invalid_argument);
}

// =============================================================================
// Type Mismatch
// =============================================================================

TEST_F(ConfigLoaderTest, ThrowsOnWrongFieldType) {
    auto path = writeTemp(std::string(R"({"vehicle":{"mass":"not_a_number","stages":1,)")
        + R"("stageCfg":{"stageMass":[1000],"numberOfEngine":[1],)"
        + R"("engine":[{"propellant":"liquid","nozzleType":"vacuum","thrust":5000,"ISP":300,"massFlowRate":1.7,"exitArea":0.1}]},)"
        + R"("aerodynamics":{"refArea":1,"refLength":1,"machMin":0.1,"machMax":5,"machTransition":0.8}},)"
        + SIMULATION_SECTION + "," + GUIDANCE_SECTION + "," + MISSION_SECTION + "}", "wrong_type");
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
