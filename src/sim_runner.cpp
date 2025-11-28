//
//  sim_runner.cpp
//  FlightSim
//
//  Implementation of simulation runner
//

#include "sim_runner.hpp"
#include "sim.hpp"
#include "math.hpp"
#include "rocket.hpp"
#include "gravity.hpp"
#include "aerodynamics.hpp"
#include "Atmosphere_properties.hpp"
#include "GNC.hpp"
#include "engine_model.hpp"
#include "utilities.hpp"
#include "trajectory.hpp"
#include "constants.h"

#include <iostream>
#include <iomanip>

using namespace std;

void SimulationRunner::start_async(const SimulationParams& params) {
    // Wait for previous simulation to finish
    if (sim_thread_.joinable()) {
        sim_thread_.join();
    }

    // Start new simulation in background thread
    is_running_ = true;
    sim_thread_ = std::thread(&SimulationRunner::run_simulation, this, params);
}

void SimulationRunner::run_simulation(const SimulationParams params) {
    SimulationResults results;
    results.success = false;
    results.steps_completed = 0;

    is_running_ = true;

    try {
        // Initialize time
        double t = params.t_start;
        double t_end = params.t_end;

        // Initial conditions from parameters
        vector<double> Pi_init = params.Pi_init;
        vector<double> Vi_init = params.Vi_init;

        // Convert steering angle from degrees to radians
        vector<double> steering_angle_initial = {params.steering_angle_deg * D2R, 0.0, 0.0};

        // Convert launch parameters from degrees to radians
        double A0_rad = params.launch_azimuth_A0_deg * D2R;
        double B0_rad = params.launch_azimuth_B0_deg * D2R;
        double longitude_rad = params.launch_longitude_deg * D2R;
        double latitude_rad = params.launch_latitude_deg * D2R;

        // Launch site coordinates
        vector<double> launch_coords = {longitude_rad, latitude_rad, params.launch_height_m};

        // Initialize gravity model with GUI parameters
        gravity_model gravity = gravity_model(launch_coords, Pi_init, A0_rad, B0_rad);

        // Calculate earth rotation velocity with GUI parameters
        vector<double> omega_e0_vec = {cos(A0_rad)*cos(B0_rad), sin(B0_rad), -cos(B0_rad)*sin(A0_rad)};
        vector<double> omega_e_vec = scalar_vector(omega_e, omega_e0_vec);
        vector<double> Ve_init = cross_product33(omega_e_vec, gravity.get_R_vec());
        vector<double> aero_angle = {0.0, 0.0};

        // Body-relative velocity
        vector<double> Vb_init = subtract_vectors(Vi_init, Ve_init);

        // Initialize atmosphere model with GUI parameter
        atmosphere_model atm = atmosphere_model(gravity.get_altitude(Pi_init));

        // Select initial flight state based on stage
        flight_states initial_flight_state;
        switch (params.stage) {
            case SimulationStage::LaunchSite:
                initial_flight_state = flight_states::FIRST_STAGE_FLIGHT;
                break;
            case SimulationStage::SecondStage:
                initial_flight_state = flight_states::SECOND_STAGE_FLIGHT;
                break;
            case SimulationStage::ThirdStage:
                initial_flight_state = flight_states::THIRD_STAGE_FLIGHT;
                break;
            default:
                initial_flight_state = flight_states::THIRD_STAGE_FLIGHT;
                break;
        }

        // Initialize rocket with stage-specific flight state
        rkt::rocket rocket = rkt::rocket(t, Pi_init, Vi_init, Vb_init,
                                          aero_angle, params.initial_mass, initial_flight_state,
                                          steering_angle_initial, gravity.get_R_vec());

        // Initialize engine
        engine_model engine = engine_model();

        // Terminal conditions from GUI parameters
        vector<double> Pi_terminal = params.Pi_terminal;
        vector<double> Vi_terminal = params.Vi_terminal;

        // Initialize guidance system - load trajectory tables
        Guidance guidance = Guidance(t,
                                     rocket,
                                     read_table(params.euler_angles_file),
                                     read_table(params.v_inertial_file),
                                     read_table(params.p_inertial_file));

        // Set the simulation stage for proper time-to-go calculation
        guidance.set_simulation_stage(initial_flight_state);

        // Set launch site parameters (A0, B0, longitude) from GUI to override hardcoded trajectory.hpp values
        guidance.set_launch_site_params(A0_rad, B0_rad, longitude_rad);

        // Pass GUI orbital parameters to override trajectory.hpp defaults
        // This also recalculates the G rotation matrix with Earth rotation correction
        guidance.set_target_orbit(params.a_terminal, params.eccentricity, params.inclination,
                                  params.right_ascension, params.argument_of_perigee, params.true_anomaly,
                                  params.t_end);
        guidance.set_terminal_conditions(params.Pi_terminal, params.Vi_terminal);

        // Initialize IGM with stage-specific terminal conditions
        guidance.IGM_initialize(Vi_init,
                                Pi_init,
                                steering_angle_initial,
                                t_end);

        // Initialize aerodynamics
        aerodynamics aero = aerodynamics(read_table(params.highspeed_aero_file),
                                         read_table(params.lowspeed_aero_file),
                                         EffectiveArea, L);

        // Buffer for storing simulation data in memory (performance optimization)
        // Preallocate to avoid dynamic reallocations during simulation
        std::vector<std::vector<double>> data_buffer;
        data_buffer.reserve(250000);  // Preallocate ~250k rows (slightly more than typical 200k steps)

        // Clear trajectory history at start
        {
            std::lock_guard<std::mutex> lock(trajectory_mutex_);
            trajectory_history_.clear();
        }

        // Main simulation loop
        int step_counter = 0;
        int trajectory_sample_counter = 0;
        const int trajectory_sample_interval = 10;  // Store every 10 steps to reduce memory usage
        const int telemetry_update_interval = 100;   // Update telemetry every 100 steps (0.1s @ 1ms step)
                                                     // GUI runs at 60 Hz (~16ms), so this is more than sufficient

        // Record initial state before loop (buffered in memory)
        data_buffer.push_back({
            t,
            rocket.get_phi() * R2D,
            rocket.get_psi() * R2D,
            rocket.get_gamma() * R2D,
            rocket.get_mass(),
            rocket.get_PI()[0],
            rocket.get_PI()[1],
            rocket.get_PI()[2],
            rocket.get_VI()[0],
            rocket.get_VI()[1],
            rocket.get_VI()[2],
            atm.get_mach_number(),
            gravity.get_altitude(rocket.get_PI()),
            guidance.get_semi_major_axis(),
            guidance.get_time_to_go()
        });
        
        // Store initial trajectory point
        {
            std::lock_guard<std::mutex> lock(trajectory_mutex_);
            trajectory_history_.push_back({rocket.get_PI()[0], rocket.get_PI()[1], rocket.get_PI()[2]});
        }
        
        while (!guidance.Cutoff(rocket.get_PI(), rocket.get_VI(), guidance.get_time_to_go(), t)) {
            // Record current state (buffered in memory - performance optimization)
            data_buffer.push_back({
                t,
                rocket.get_phi() * R2D,
                rocket.get_psi() * R2D,
                rocket.get_gamma() * R2D,
                rocket.get_mass(),
                rocket.get_PI()[0],
                rocket.get_PI()[1],
                rocket.get_PI()[2],
                rocket.get_VI()[0],
                rocket.get_VI()[1],
                rocket.get_VI()[2],
                atm.get_mach_number(),
                gravity.get_altitude(rocket.get_PI()),
                guidance.get_semi_major_axis(),
                guidance.get_time_to_go()
            });

            // Store trajectory point periodically (thread-safe)
            if (trajectory_sample_counter % trajectory_sample_interval == 0) {
                std::lock_guard<std::mutex> lock(trajectory_mutex_);
                trajectory_history_.push_back({rocket.get_PI()[0], rocket.get_PI()[1], rocket.get_PI()[2]});
            }
            trajectory_sample_counter++;

            // Update telemetry periodically (for real-time display) - THREAD-SAFE
            // Only update every N steps to reduce mutex contention (performance optimization)
            if (step_counter % telemetry_update_interval == 0) {
                std::lock_guard<std::mutex> lock(telemetry_mutex_);
                telemetry_.current_time = t;
                telemetry_.time_to_go = guidance.get_time_to_go();
                telemetry_.pos_x = rocket.get_PI()[0];
                telemetry_.pos_y = rocket.get_PI()[1];
                telemetry_.pos_z = rocket.get_PI()[2];
                telemetry_.vel_x = rocket.get_VI()[0];
                telemetry_.vel_y = rocket.get_VI()[1];
                telemetry_.vel_z = rocket.get_VI()[2];
                telemetry_.velocity_magnitude = vector_mag(rocket.get_VI());
                telemetry_.phi = rocket.get_phi() * R2D;
                telemetry_.psi = rocket.get_psi() * R2D;
                telemetry_.gamma = rocket.get_gamma() * R2D;

                // Mass & Fuel
                const double vehicle_dry_mass = 5796.1;  // Dry mass = 6731.1 - 935.0 (353.9 + 581.1)
                telemetry_.total_mass = rocket.get_mass();
                telemetry_.vehicle_mass = vehicle_dry_mass;
                telemetry_.fuel_consumed = params.initial_mass - rocket.get_mass();
                telemetry_.fuel_remaining = rocket.get_mass() - vehicle_dry_mass;
                telemetry_.fuel_consumption_rate = -rocket.get_del_m();  // Negative because mass_dot is negative

                telemetry_.altitude = gravity.get_altitude(rocket.get_PI());

                // Orbital elements - 6 Classical Keplerian Elements updated in real-time
                telemetry_.semi_major_axis = guidance.get_semi_major_axis();
                telemetry_.eccentricity = guidance.get_eccentricity();
                telemetry_.inclination = guidance.get_inclination() * R2D;  // Convert to degrees
                telemetry_.right_ascension = guidance.get_right_ascension() * R2D;  // Convert to degrees
                telemetry_.argument_of_perigee = guidance.get_argument_of_perigee() * R2D;  // Convert to degrees
                telemetry_.true_anomaly = guidance.get_true_anomaly() * R2D;  // Convert to degrees

                // Calculate apogee and perigee altitudes from orbital elements
                // apogee = a(1+e) - R_earth, perigee = a(1-e) - R_earth
                if (telemetry_.semi_major_axis > 0) {
                    telemetry_.apogee = telemetry_.semi_major_axis * (1.0 + telemetry_.eccentricity) - a_e;
                    telemetry_.perigee = telemetry_.semi_major_axis * (1.0 - telemetry_.eccentricity) - a_e;
                } else {
                    telemetry_.apogee = 0.0;
                    telemetry_.perigee = 0.0;
                }

                // Calculate deltas from target orbital elements (use params instead of hardcoded trajectory.hpp values)
                telemetry_.delta_semi_major_axis = params.a_terminal - telemetry_.semi_major_axis;
                telemetry_.delta_eccentricity = params.eccentricity - telemetry_.eccentricity;
                telemetry_.delta_inclination = (params.inclination * R2D) - telemetry_.inclination;
                telemetry_.delta_right_ascension = (params.right_ascension * R2D) - telemetry_.right_ascension;
                telemetry_.delta_argument_of_perigee = (params.argument_of_perigee * R2D) - telemetry_.argument_of_perigee;
                telemetry_.delta_true_anomaly = (params.true_anomaly * R2D) - telemetry_.true_anomaly;

                telemetry_.mach_number = atm.get_mach_number();

                // Engine parameters (based on flight state)
                std::vector<double> thrust_vec = engine.get_total_thrust();
                telemetry_.thrust = vector_mag(thrust_vec);

                // ISP and exit velocity depend on flight state
                if (rocket.get_fstate() == flight_states::THIRD_STAGE_FLIGHT) {
                    telemetry_.isp = Isp3;
                    telemetry_.exit_velocity = Vex3;
                } else if (rocket.get_fstate() == flight_states::SECOND_STAGE_FLIGHT) {
                    telemetry_.isp = Isp2;
                    telemetry_.exit_velocity = Vex2;
                } else if (rocket.get_fstate() == flight_states::FIRST_STAGE_FLIGHT) {
                    telemetry_.isp = Isp_SL;
                    telemetry_.exit_velocity = Isp_SL * g0;
                } else {
                    telemetry_.isp = 0.0;
                    telemetry_.exit_velocity = 0.0;
                }

                telemetry_.step_number = step_counter;
                telemetry_.is_running = true;

                // Target deltas (differences from terminal targets - use params instead of hardcoded)
                telemetry_.delta_pos_x = rocket.get_PI()[0] - params.Pi_terminal[0];
                telemetry_.delta_pos_y = rocket.get_PI()[1] - params.Pi_terminal[1];
                telemetry_.delta_pos_z = rocket.get_PI()[2] - params.Pi_terminal[2];
                telemetry_.delta_vel_x = rocket.get_VI()[0] - params.Vi_terminal[0];
                telemetry_.delta_vel_y = rocket.get_VI()[1] - params.Vi_terminal[1];
                telemetry_.delta_vel_z = rocket.get_VI()[2] - params.Vi_terminal[2];
                telemetry_.range_angle = guidance.getRangeAngle() * R2D;
            }  // End if (step_counter % telemetry_update_interval == 0)

            // Run one RK4 integration step
            rk4(t, step_counter, rocket, gravity, atm, engine, guidance, aero, rocket.get_fstate());

            // Update time
            t += step;
            step_counter++;
        }

        // Write all buffered data to file in one bulk operation (performance optimization)
        ofstream record(params.output_file);
        if (!record.is_open()) {
            results.error_message = "Error opening output file: " + params.output_file;
            results.success = false;
            // Store error results
            {
                std::lock_guard<std::mutex> lock(results_mutex_);
                last_results_ = results;
            }
            is_running_ = false;
            return;
        }

        // Write CSV header
        record << "time,phi,psi,gamma,M,Px,Py,Pz,Vx,Vy,Vz,Ma,H,semi_major_axis,time_to_go\n";

        // Write all buffered data rows
        // Use \n instead of endl to avoid unnecessary buffer flushes (performance optimization)
        for (const auto& row : data_buffer) {
            record << fixed << setprecision(6);
            for (size_t i = 0; i < row.size(); ++i) {
                record << row[i];
                if (i < row.size() - 1) {
                    record << ",";
                }
            }
            record << "\n";  // Use \n instead of endl (no flush until file close)
        }

        record.close();

        // Final telemetry update to capture end state
        {
            std::lock_guard<std::mutex> lock(telemetry_mutex_);
            telemetry_.current_time = t;
            telemetry_.time_to_go = guidance.get_time_to_go();
            telemetry_.pos_x = rocket.get_PI()[0];
            telemetry_.pos_y = rocket.get_PI()[1];
            telemetry_.pos_z = rocket.get_PI()[2];
            telemetry_.vel_x = rocket.get_VI()[0];
            telemetry_.vel_y = rocket.get_VI()[1];
            telemetry_.vel_z = rocket.get_VI()[2];
            telemetry_.velocity_magnitude = vector_mag(rocket.get_VI());
            telemetry_.altitude = gravity.get_altitude(rocket.get_PI());
            telemetry_.semi_major_axis = guidance.get_semi_major_axis();
            telemetry_.eccentricity = guidance.get_eccentricity();
            telemetry_.step_number = step_counter;
            telemetry_.is_running = false;  // Mark as stopped
        }

        // Simulation completed successfully
        results.success = true;
        results.steps_completed = step_counter;
        results.error_message = "Simulation completed successfully!";

    } catch (const std::exception& e) {
        results.success = false;
        results.error_message = string("Exception: ") + e.what();
    } catch (...) {
        results.success = false;
        results.error_message = "Unknown exception occurred during simulation";
    }

    // Store results
    {
        std::lock_guard<std::mutex> lock(results_mutex_);
        last_results_ = results;
    }

    is_running_ = false;
}
