//
//  sim_runner.hpp
//  FlightSim
//
//  Wrapper class to run simulation from GUI
//

#ifndef sim_runner_hpp
#define sim_runner_hpp

#include <string>
#include <vector>
#include <fstream>
#include <thread>
#include <mutex>
#include <atomic>

// Simulation parameters passed from GUI
struct SimulationParams {
    // Time parameters
    double t_start;
    double t_end;

    // Initial conditions
    std::vector<double> Pi_init;  // Initial position [x, y, z]
    std::vector<double> Vi_init;  // Initial velocity [x, y, z]
    double initial_mass;
    double steering_angle_deg;    // in degrees

    // Launch site initialization
    double launch_azimuth_A0_deg;     // Launch azimuth A0 in degrees
    double launch_azimuth_B0_deg;     // Launch azimuth B0 in degrees
    double launch_longitude_deg;      // Launch site longitude in degrees
    double launch_latitude_deg;       // Launch site latitude in degrees
    double launch_height_m;           // Launch site height in meters
    double initial_altitude_m;        // Initial altitude above launch site in meters

    // CSV file paths
    std::string euler_angles_file;
    std::string v_inertial_file;
    std::string p_inertial_file;
    std::string highspeed_aero_file;
    std::string lowspeed_aero_file;

    // Output file
    std::string output_file;
};

// Results structure (for future plotting)
struct SimulationResults {
    bool success;
    std::string error_message;
    int steps_completed;
};

// Real-time telemetry data
struct TelemetryData {
    // Time
    double current_time;
    double time_to_go;

    // Position (meters)
    double pos_x, pos_y, pos_z;

    // Velocity (m/s)
    double vel_x, vel_y, vel_z;
    double velocity_magnitude;

    // Attitude (degrees)
    double phi;    // Pitch
    double psi;    // Yaw
    double gamma;  // Roll

    // Mass & Fuel
    double total_mass;              // Current total mass (Given)
    double vehicle_mass;            // Dry mass / structural mass
    double fuel_consumed;           // Total fuel burned
    double fuel_remaining;          // Fuel still available
    double fuel_consumption_rate;   // Mass flow rate (kg/s)

    // Orbital Elements (6 Classical Keplerian Elements)
    double altitude;
    double semi_major_axis;        // a - size of orbit
    double eccentricity;            // e - shape (0=circular, <1=elliptical)
    double inclination;             // i - tilt of orbital plane (degrees)
    double right_ascension;         // Ω (RAAN) - orientation of ascending node (degrees)
    double argument_of_perigee;     // ω - orientation in orbital plane (degrees)
    double true_anomaly;            // ν - position along orbit (degrees)

    // Derived orbital parameters
    double apogee;
    double perigee;

    // Deltas from target orbital elements
    double delta_semi_major_axis;
    double delta_eccentricity;
    double delta_inclination;
    double delta_right_ascension;
    double delta_argument_of_perigee;
    double delta_true_anomaly;

    // Performance
    double mach_number;
    double dynamic_pressure;

    // Engine
    double thrust;              // Total thrust magnitude (N)
    double exit_velocity;       // Exhaust velocity (m/s)
    double isp;                 // Specific impulse (s)

    // Status
    int step_number;
    bool is_running;

    // Target Deltas (differences from target orbit)
    double delta_pos_x, delta_pos_y, delta_pos_z;  // Position error from SECO target
    double delta_vel_x, delta_vel_y, delta_vel_z;  // Velocity error from terminal
    double range_angle;  // Range angle (beta)
};

class SimulationRunner {
public:
    SimulationRunner() : is_running_(false) {
        // Initialize telemetry
        telemetry_.is_running = false;
        telemetry_.current_time = 0.0;
        telemetry_.step_number = 0;
    }

    ~SimulationRunner() {
        // Wait for thread to finish if still running
        if (sim_thread_.joinable()) {
            sim_thread_.join();
        }
    }

    // Start the simulation in a background thread
    void start_async(const SimulationParams& params);

    // Check if simulation is currently running
    bool is_running() const { return is_running_.load(); }

    // Get current telemetry data (thread-safe)
    TelemetryData get_telemetry() const {
        std::lock_guard<std::mutex> lock(telemetry_mutex_);
        return telemetry_;
    }

    // Get latest results (call after simulation completes)
    SimulationResults get_results() const {
        std::lock_guard<std::mutex> lock(results_mutex_);
        return last_results_;
    }

    // Get trajectory history (thread-safe)
    std::vector<std::vector<double>> get_trajectory_history() const {
        std::lock_guard<std::mutex> lock(trajectory_mutex_);
        return trajectory_history_;
    }

    // Clear trajectory history
    void clear_trajectory_history() {
        std::lock_guard<std::mutex> lock(trajectory_mutex_);
        trajectory_history_.clear();
    }

private:
    // The actual simulation code (runs in background thread)
    void run_simulation(const SimulationParams params);

    std::atomic<bool> is_running_;
    std::thread sim_thread_;

    mutable std::mutex telemetry_mutex_;  // Protects telemetry_
    TelemetryData telemetry_;

    mutable std::mutex results_mutex_;    // Protects last_results_
    SimulationResults last_results_;

    mutable std::mutex trajectory_mutex_;  // Protects trajectory_history_
    std::vector<std::vector<double>> trajectory_history_;  // Each entry: [x, y, z]
};

#endif /* sim_runner_hpp */
