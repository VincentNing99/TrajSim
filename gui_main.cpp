//
//  gui_main.cpp
//  FlightSim GUI
//
//  Created for Flight Simulation GUI Control
//

#include "imgui/imgui.h"
#include "imgui/backends/imgui_impl_glfw.h"
#include "imgui/backends/imgui_impl_opengl3.h"
#include <GLFW/glfw3.h>
#include <stdio.h>
#include <string>

// Cross-platform file dialogs
#include "portable-file-dialogs.h"

// Your simulation includes
#include "sim.hpp"
#include "sim_runner.hpp"  // NEW: Simulation runner wrapper
#include "rocket.hpp"
#include "math.hpp"
#include "gravity.hpp"
#include "aerodynamics.hpp"
#include "Atmosphere_properties.hpp"
#include "GNC.hpp"
#include "engine_model.hpp"
#include "utilities.hpp"
#include "trajectory.hpp"

// Stage selection enum
enum class SimulationStage {
    LaunchSite,
    SecondStage,
    ThirdStage
};

static void glfw_error_callback(int error, const char* description) {
    fprintf(stderr, "GLFW Error %d: %s\n", error, description);
}

// Cross-platform file dialog helper function
std::string open_file_dialog(const char* default_path = nullptr) {
    // Create file filters for CSV files
    std::vector<std::string> filters = { "CSV Files", "*.csv" };

    // Determine the initial directory
    std::string initial_dir = "";
    if (default_path && default_path[0] != '\0') {
        std::string path_str(default_path);
        size_t last_slash = path_str.find_last_of("/\\");
        if (last_slash != std::string::npos) {
            initial_dir = path_str.substr(0, last_slash);
        }
    }

    // Show the open file dialog
    auto selection = pfd::open_file("Select CSV File", initial_dir, filters).result();

    // Return the selected file path (or empty string if cancelled)
    if (!selection.empty()) {
        return selection[0];
    }
    return "";
}

// Function removed - initial conditions now entered directly in GUI

int main(int, char**) {
    // Setup window
    glfwSetErrorCallback(glfw_error_callback);
    if (!glfwInit())
        return 1;

    // GL 3.3 + GLSL 150
    const char* glsl_version = "#version 150";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);

    // Create window
    GLFWwindow* window = glfwCreateWindow(1280, 720, "Flight Simulation Control", nullptr, nullptr);
    if (window == nullptr)
        return 1;
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // Enable vsync

    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;

    // Load font with Unicode support (Greek letters, delta, etc.)
    // Try to load a system font that supports Unicode
    bool font_loaded = false;

#ifdef __APPLE__
    // macOS font path
    const char* font_paths[] = {
        "/System/Library/Fonts/Supplemental/Arial Unicode.ttf",
        "/System/Library/Fonts/Helvetica.ttc"
    };
#elif _WIN32
    // Windows font paths
    const char* font_paths[] = {
        "C:\\Windows\\Fonts\\arial.ttf",
        "C:\\Windows\\Fonts\\segoeui.ttf"
    };
#else
    // Linux font paths
    const char* font_paths[] = {
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf",
        "/usr/share/fonts/TTF/DejaVuSans.ttf"
    };
#endif

    for (const char* font_path : font_paths) {
        if (io.Fonts->AddFontFromFileTTF(font_path, 16.0f, NULL, io.Fonts->GetGlyphRangesDefault())) {
            font_loaded = true;
            break;
        }
    }

    if (!font_loaded) {
        // Fallback to default font if no system font found
        io.Fonts->AddFontDefault();
    }

    // Setup Dear ImGui style
    ImGui::StyleColorsClassic();

    // Setup Platform/Renderer backends
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);

    // GUI State Variables
    static char euler_angles_path[256] = "files/euler angles.csv";
    static char v_inertial_path[256] = "files/V_inertial.csv";
    static char p_inertial_path[256] = "files/P_inertial.csv";
    static char highspeed_path[256] = "files/highspeed.csv";
    static char lowspeed_path[256] = "files/lowspeed.csv";

    // Time parameters
    static float initial_time = 3140.3405f;
    static float end_time = 3344.5194f;

    // Initial state vectors (inertial frame)
    
    static float initial_position[3] = {-912205.4f, -13212262.5f, 149163.6f};
    static float initial_velocity[3] = {-7440.6404f, 953.8138f, 294.5883f};
    static float initial_mass = 6731.1f;
    static float steering_angle = 172.7593f;  // degrees
    static float Pitch = 172.7593f;
    static float Yaw = 0.0f;
    static float Roll = 0.0f;

    // Launch site initialization parameters
    static float launch_azimuth_A0 = 191.47506f;  // degrees
    static float launch_azimuth_B0 = 40.80768f;   // degrees
    static float launch_longitude = 100.13805f;    // degrees
    static float launch_latitude = 40.80768f;      // degrees
    static float launch_height = 1000.0f;          // meters

    // Stage selection
    static SimulationStage selected_stage = SimulationStage::LaunchSite;
    static const char* stage_names[] = { "Launch Site", "Second Stage", "Third Stage" };

    // NEW: Simulation state variables
    SimulationRunner sim_runner;
    bool simulation_complete = false;
    bool was_running = false;  // Track if simulation just finished
    std::string last_result_message = "";
    static char output_filename[256] = "files/rk4_IGM_sim_MSC.csv";
    
    // Attitude data history for plotting
    struct AttitudePoint {
        float time;
        float pitch;
        float roll;
        float yaw;
    };
    static std::vector<AttitudePoint> attitude_history;
    static const int max_history_points = 10000;  // Limit history size

    // Window visibility flags
    static bool show_window_manager = true;
    static bool show_simulation_control = true;
    static bool show_telemetry = true;
    static bool show_errors = true;
    static bool show_orbital_elements = true;

    // Main loop
    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();

        // Check if simulation just finished
        bool is_running_now = sim_runner.is_running();
        if (was_running && !is_running_now) {
            // Simulation just completed - get results
            SimulationResults results = sim_runner.get_results();
            simulation_complete = true;
            if (results.success) {
                last_result_message = "SUCCESS: " + std::to_string(results.steps_completed) +
                                      " steps completed. Output saved.";
            } else {
                last_result_message = "FAILED: " + results.error_message;
            }
        }
        was_running = is_running_now;

        // Start ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // ===== WINDOW MANAGER CONTROL PANEL =====
        if (show_window_manager) {
            ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_FirstUseEver);
            ImGui::SetNextWindowSize(ImVec2(250, 180), ImGuiCond_FirstUseEver);
            ImGui::Begin("Window Manager", &show_window_manager);

            ImGui::Text("Show/Hide Windows:");
            ImGui::Separator();
            ImGui::Spacing();

            ImGui::Checkbox("Simulation Control Panel", &show_simulation_control);
            ImGui::Checkbox("Real-Time Telemetry", &show_telemetry);
            ImGui::Checkbox("Navigation Errors", &show_errors);
            ImGui::Checkbox("Orbital Elements", &show_orbital_elements);

            ImGui::Spacing();
            ImGui::Separator();

            ImGui::End();
        }

        // ===== SIMULATION CONTROL PANEL =====
        if (show_simulation_control) {
            ImGui::SetNextWindowPos(ImVec2(270, 10), ImGuiCond_FirstUseEver);
            ImGui::SetNextWindowSize(ImVec2(500, 650), ImGuiCond_FirstUseEver);
            ImGui::Begin("Simulation Control Panel", &show_simulation_control);

        ImGui::Text("CSV File Inputs");
        ImGui::Separator();
        ImGui::InputText("Euler Angles", euler_angles_path, IM_ARRAYSIZE(euler_angles_path));
        ImGui::SameLine();
        if (ImGui::Button("Browse##1")) {
            std::string path = open_file_dialog(euler_angles_path);
            if (!path.empty()) {
                strncpy(euler_angles_path, path.c_str(), IM_ARRAYSIZE(euler_angles_path) - 1);
                euler_angles_path[IM_ARRAYSIZE(euler_angles_path) - 1] = '\0';
            }
        }

        ImGui::InputText("V Inertial", v_inertial_path, IM_ARRAYSIZE(v_inertial_path));
        ImGui::SameLine();
        if (ImGui::Button("Browse##2")) {
            std::string path = open_file_dialog(v_inertial_path);
            if (!path.empty()) {
                strncpy(v_inertial_path, path.c_str(), IM_ARRAYSIZE(v_inertial_path) - 1);
                v_inertial_path[IM_ARRAYSIZE(v_inertial_path) - 1] = '\0';
            }
        }

        ImGui::InputText("P Inertial", p_inertial_path, IM_ARRAYSIZE(p_inertial_path));
        ImGui::SameLine();
        if (ImGui::Button("Browse##3")) {
            std::string path = open_file_dialog(p_inertial_path);
            if (!path.empty()) {
                strncpy(p_inertial_path, path.c_str(), IM_ARRAYSIZE(p_inertial_path) - 1);
                p_inertial_path[IM_ARRAYSIZE(p_inertial_path) - 1] = '\0';
            }
        }

        ImGui::InputText("High Speed Aero", highspeed_path, IM_ARRAYSIZE(highspeed_path));
        ImGui::SameLine();
        if (ImGui::Button("Browse##4")) {
            std::string path = open_file_dialog(highspeed_path);
            if (!path.empty()) {
                strncpy(highspeed_path, path.c_str(), IM_ARRAYSIZE(highspeed_path) - 1);
                highspeed_path[IM_ARRAYSIZE(highspeed_path) - 1] = '\0';
            }
        }

        ImGui::InputText("Low Speed Aero", lowspeed_path, IM_ARRAYSIZE(lowspeed_path));
        ImGui::SameLine();
        if (ImGui::Button("Browse##5")) {
            std::string path = open_file_dialog(lowspeed_path);
            if (!path.empty()) {
                strncpy(lowspeed_path, path.c_str(), IM_ARRAYSIZE(lowspeed_path) - 1);
                lowspeed_path[IM_ARRAYSIZE(lowspeed_path) - 1] = '\0';
            }
        }

        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Text("Initialization");
        ImGui::Separator();

        // Launch Site Parameters
        ImGui::Text("Launch Site Parameters:");
        ImGui::InputFloat("Azimuth A0 (deg)", &launch_azimuth_A0);
        ImGui::InputFloat("Azimuth B0 (deg)", &launch_azimuth_B0);
        ImGui::InputFloat("Longitude (deg)", &launch_longitude);
        ImGui::InputFloat("Latitude (deg)", &launch_latitude);
        ImGui::InputFloat("Site Height (m)", &launch_height);

        ImGui::Spacing();
        ImGui::Separator();

        // Stage selection
        ImGui::Text("Start From Stage:");
        int current_stage = (int)selected_stage;
        if (ImGui::Combo("##Stage", &current_stage, stage_names, IM_ARRAYSIZE(stage_names))) {
            selected_stage = (SimulationStage)current_stage;
        }

        ImGui::Spacing();
        ImGui::Separator();

        // Time and Mass Parameters
        ImGui::Text("Time & Mass Parameters:");
        ImGui::InputFloat("Initial Time (s)", &initial_time);
        ImGui::InputFloat("Initial Mass (kg)", &initial_mass);
        ImGui::InputFloat("Steering Angle (deg)", &steering_angle);

        ImGui::Spacing();
        ImGui::Separator();

        // Initial State Vectors (Inertial Frame)
        ImGui::Text("Initial State (Inertial Frame):");
        ImGui::Text("Initial Position (m):");
        ImGui::InputFloat("Px", &initial_position[0]);
        ImGui::InputFloat("Py", &initial_position[1]);
        ImGui::InputFloat("Pz", &initial_position[2]);

        ImGui::Text("Initial Velocity (m/s):");
        ImGui::InputFloat("Vx", &initial_velocity[0]);
        ImGui::InputFloat("Vy", &initial_velocity[1]);
        ImGui::InputFloat("Vz", &initial_velocity[2]);

        ImGui::Spacing();
        ImGui::Separator();

        // NEW: Output filename input
        ImGui::InputText("Output CSV", output_filename, IM_ARRAYSIZE(output_filename));

        ImGui::Spacing();

        // NEW: Run simulation button with actual logic
        bool is_sim_running = sim_runner.is_running();

        // Disable button while simulation is running
        if (is_sim_running) {
            ImGui::PushStyleVar(ImGuiStyleVar_Alpha, 0.5f);
        }

        bool button_clicked = ImGui::Button("Run Simulation", ImVec2(200, 40));

        if (is_sim_running) {
            ImGui::PopStyleVar();
        }

        if (button_clicked && !is_sim_running) {
            // Build parameter structure from GUI inputs
            SimulationParams params;
            params.t_start = initial_time;
            params.t_end = end_time;
            params.Pi_init = {initial_position[0], initial_position[1], initial_position[2]};
            params.Vi_init = {initial_velocity[0], initial_velocity[1], initial_velocity[2]};
            params.initial_mass = initial_mass;
            params.steering_angle_deg = steering_angle;

            // Launch site initialization parameters
            params.launch_azimuth_A0_deg = launch_azimuth_A0;
            params.launch_azimuth_B0_deg = launch_azimuth_B0;
            params.launch_longitude_deg = launch_longitude;
            params.launch_latitude_deg = launch_latitude;
            params.launch_height_m = launch_height;

            // CSV file paths
            params.euler_angles_file = std::string(euler_angles_path);
            params.v_inertial_file = std::string(v_inertial_path);
            params.p_inertial_file = std::string(p_inertial_path);
            params.highspeed_aero_file = std::string(highspeed_path);
            params.lowspeed_aero_file = std::string(lowspeed_path);
            params.output_file = std::string(output_filename);

            // Start the simulation in background thread
            sim_runner.start_async(params);
            simulation_complete = false;  // Reset completion flag
            last_result_message = "Simulation running...";
            
            // Clear attitude history for new simulation
            attitude_history.clear();
        }

        ImGui::SameLine();
    

        // NEW: Display simulation results
        if (simulation_complete) {
            ImGui::Spacing();
            ImGui::Separator();
            ImGui::TextWrapped("Last Result: %s", last_result_message.c_str());
        }

            ImGui::End();
        }

        // ===== TELEMETRY WINDOW =====
        TelemetryData telemetry = sim_runner.get_telemetry();
        
        // Update attitude history
        if (telemetry.is_running) {
            AttitudePoint point;
            point.time = (float)telemetry.current_time;
            point.pitch = (float)telemetry.phi;
            point.roll = (float)telemetry.gamma;
            point.yaw = (float)telemetry.psi;
            attitude_history.push_back(point);
            
            // Limit history size
            if (attitude_history.size() > max_history_points) {
                attitude_history.erase(attitude_history.begin());
            }
        }

        if (show_telemetry) {
            ImGui::SetNextWindowPos(ImVec2(780, 10), ImGuiCond_FirstUseEver);
            ImGui::SetNextWindowSize(ImVec2(450, 680), ImGuiCond_FirstUseEver);
            ImGui::Begin("Real-Time Telemetry", &show_telemetry);

        // Status indicator
        if (telemetry.is_running) {
            ImGui::TextColored(ImVec4(0.0f, 1.0f, 0.0f, 1.0f), "● SIMULATION RUNNING");
        } else {
            ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f), "○ Idle");
        }

        // Time information (always visible)
        ImGui::Separator();
        ImGui::Text("Time: %.3f s  |  Time to Go: %.3f s  |  Step: %d",
                    telemetry.current_time, telemetry.time_to_go, telemetry.step_number);
        ImGui::Separator();

        // Create tabs
        if (ImGui::BeginTabBar("TelemetryTabs")) {
            // ===== ATTITUDE TAB =====
            if (ImGui::BeginTabItem("Attitude")) {
                ImGui::Spacing();

                // Position and Velocity side by side
                float column_width = ImGui::GetContentRegionAvail().x * 0.5f - 10.0f;
                ImGui::Columns(2, "PosVelColumns", false);
                ImGui::SetColumnWidth(0, column_width);
                
                // Position (Left column)
                ImGui::Text("Position (Inertial Frame)");
                ImGui::Text("  X: %12.2f m", telemetry.pos_x);
                ImGui::Text("  Y: %12.2f m", telemetry.pos_y);
                ImGui::Text("  Z: %12.2f m", telemetry.pos_z);
                ImGui::Text("  Altitude: %10.2f m", telemetry.altitude);
                
                ImGui::NextColumn();
                
                // Velocity (Right column)
                ImGui::Text("Velocity (Inertial Frame)");
                ImGui::Text("  Vx: %10.2f m/s", telemetry.vel_x);
                ImGui::Text("  Vy: %10.2f m/s", telemetry.vel_y);
                ImGui::Text("  Vz: %10.2f m/s", telemetry.vel_z);
                ImGui::Text("  |V|: %9.2f m/s", telemetry.velocity_magnitude);
                
                ImGui::Columns(1);
                
                ImGui::Spacing();
                ImGui::Separator();

                // Attitude (Euler Angles)
                ImGui::Text("Attitude (Euler Angles)");
                ImGui::Text("  Pitch (φ): %8.3f°", telemetry.phi);
                ImGui::Text("  Yaw   (ψ): %8.3f°", telemetry.psi);
                ImGui::Text("  Roll  (γ): %8.3f°", telemetry.gamma);

                ImGui::Spacing();
                ImGui::Separator();
                
                // Attitude Plots
                if (attitude_history.size() > 0) {
                    // Reserve space for plots first to avoid overlapping with text
                    float plot_padding = 50.0f;
                    float plot_height = 100.0f;
                    float spacing = 15.0f;
                    float title_space = 20.0f;
                    float total_plots_height = 3 * (plot_height + title_space) + 2 * spacing + 30.0f;  // 3 plots + spacing + time label
                    
                    // Reserve space
                    ImGui::Dummy(ImVec2(0, total_plots_height));
                    
                    ImDrawList* draw_list = ImGui::GetWindowDrawList();
                    
                    // Find time range (flight time = current time - initial time)
                    float max_time = attitude_history.back().time - attitude_history[0].time;
                    if (max_time < 0.1f) max_time = 0.1f;  // Avoid division by zero
                    
                    // Get position for plots (before the dummy we just added)
                    ImVec2 plot_start_pos = ImVec2(
                        ImGui::GetCursorScreenPos().x,
                        ImGui::GetCursorScreenPos().y - total_plots_height
                    );
                    float plot_width = ImGui::GetContentRegionAvail().x - 2 * plot_padding;
                    float plot_x = plot_start_pos.x + plot_padding;
                    
                    // Helper function to draw a single plot
                    auto draw_plot = [&](const char* title, float* values, ImU32 color, 
                                         float min_val, float max_val, float y_offset, float plot_h) {
                        float plot_y = plot_start_pos.y + y_offset;
                        float title_height = 20.0f;
                        
                        // Draw background
                        draw_list->AddRectFilled(ImVec2(plot_x, plot_y), 
                                               ImVec2(plot_x + plot_width, plot_y + plot_h),
                                               IM_COL32(20, 20, 30, 255));
                        
                        // Draw grid lines (horizontal)
                        int num_grid_lines = 4;
                        for (int i = 0; i <= num_grid_lines; i++) {
                            float y = plot_y + (plot_h * i / num_grid_lines);
                            draw_list->AddLine(ImVec2(plot_x, y), ImVec2(plot_x + plot_width, y),
                                             IM_COL32(50, 50, 50, 100), 1.0f);
                        }
                        
                        // Draw zero line if it's in range
                        if (min_val <= 0.0f && max_val >= 0.0f) {
                            float zero_y = plot_y + plot_h * (1.0f - (0.0f - min_val) / (max_val - min_val));
                            draw_list->AddLine(ImVec2(plot_x, zero_y), ImVec2(plot_x + plot_width, zero_y),
                                             IM_COL32(100, 100, 100, 150), 1.0f);
                        }
                        
                        // Draw axes
                        draw_list->AddLine(ImVec2(plot_x, plot_y), ImVec2(plot_x, plot_y + plot_h),
                                         IM_COL32(200, 200, 200, 255), 2.0f);
                        draw_list->AddLine(ImVec2(plot_x, plot_y + plot_h), 
                                         ImVec2(plot_x + plot_width, plot_y + plot_h),
                                         IM_COL32(200, 200, 200, 255), 2.0f);
                        
                        // Draw Y-axis labels
                        char label_buf[32];
                        for (int i = 0; i <= num_grid_lines; i++) {
                            float val = min_val + (max_val - min_val) * (1.0f - (float)i / num_grid_lines);
                            float y = plot_y + (plot_h * i / num_grid_lines);
                            snprintf(label_buf, sizeof(label_buf), "%.1f°", val);
                            ImVec2 text_size = ImGui::CalcTextSize(label_buf);
                            draw_list->AddText(ImVec2(plot_x - text_size.x - 5, y - text_size.y * 0.5f),
                                             IM_COL32(200, 200, 200, 255), label_buf);
                        }
                        
                        // Draw the curve
                        if (attitude_history.size() > 1) {
                            for (size_t i = 1; i < attitude_history.size(); i++) {
                                float flight_time1 = attitude_history[i-1].time - attitude_history[0].time;
                                float flight_time2 = attitude_history[i].time - attitude_history[0].time;
                                float x1 = plot_x + (flight_time1 / max_time) * plot_width;
                                float x2 = plot_x + (flight_time2 / max_time) * plot_width;
                                
                                float val1 = values[i-1];
                                float val2 = values[i];
                                float y1 = plot_y + plot_h * (1.0f - (val1 - min_val) / (max_val - min_val));
                                float y2 = plot_y + plot_h * (1.0f - (val2 - min_val) / (max_val - min_val));
                                
                                draw_list->AddLine(ImVec2(x1, y1), ImVec2(x2, y2), color, 2.0f);
                            }
                        }
                        
                        // Draw title
                        ImVec2 title_size = ImGui::CalcTextSize(title);
                        draw_list->AddText(ImVec2(plot_x + plot_width * 0.5f - title_size.x * 0.5f, plot_y - title_height),
                                         IM_COL32(255, 255, 255, 255), title);
                        
                        // Draw current value
                        if (attitude_history.size() > 0) {
                            char val_buf[64];
                            snprintf(val_buf, sizeof(val_buf), "Current: %.2f°", values[attitude_history.size()-1]);
                            ImVec2 val_size = ImGui::CalcTextSize(val_buf);
                            draw_list->AddText(ImVec2(plot_x + plot_width - val_size.x, plot_y - title_height),
                                             color, val_buf);
                        }
                    };
                    
                    // Calculate min/max for each angle
                    float pitch_min = attitude_history[0].pitch;
                    float pitch_max = attitude_history[0].pitch;
                    float roll_min = attitude_history[0].roll;
                    float roll_max = attitude_history[0].roll;
                    float yaw_min = attitude_history[0].yaw;
                    float yaw_max = attitude_history[0].yaw;
                    
                    for (const auto& point : attitude_history) {
                        if (point.pitch < pitch_min) pitch_min = point.pitch;
                        if (point.pitch > pitch_max) pitch_max = point.pitch;
                        if (point.roll < roll_min) roll_min = point.roll;
                        if (point.roll > roll_max) roll_max = point.roll;
                        if (point.yaw < yaw_min) yaw_min = point.yaw;
                        if (point.yaw > yaw_max) yaw_max = point.yaw;
                    }
                    
                    // Add padding to ranges
                    float pitch_range = pitch_max - pitch_min;
                    float roll_range = roll_max - roll_min;
                    float yaw_range = yaw_max - yaw_min;
                    if (pitch_range > 0.01f) {
                        pitch_min -= pitch_range * 0.1f;
                        pitch_max += pitch_range * 0.1f;
                    }
                    if (roll_range > 0.01f) {
                        roll_min -= roll_range * 0.1f;
                        roll_max += roll_range * 0.1f;
                    }
                    if (yaw_range > 0.01f) {
                        yaw_min -= yaw_range * 0.1f;
                        yaw_max += yaw_range * 0.1f;
                    }
                    
                    // Ensure minimum range
                    if (pitch_max - pitch_min < 10.0f) {
                        float center = (pitch_max + pitch_min) * 0.5f;
                        pitch_min = center - 5.0f;
                        pitch_max = center + 5.0f;
                    }
                    if (roll_max - roll_min < 10.0f) {
                        float center = (roll_max + roll_min) * 0.5f;
                        roll_min = center - 5.0f;
                        roll_max = center + 5.0f;
                    }
                    if (yaw_max - yaw_min < 10.0f) {
                        float center = (yaw_max + yaw_min) * 0.5f;
                        yaw_min = center - 5.0f;
                        yaw_max = center + 5.0f;
                    }
                    
                    // Prepare data arrays
                    std::vector<float> pitch_values, roll_values, yaw_values;
                    for (const auto& point : attitude_history) {
                        pitch_values.push_back(point.pitch);
                        roll_values.push_back(point.roll);
                        yaw_values.push_back(point.yaw);
                    }
                    
                    // Calculate Y offsets for each plot (with title space)
                    float y_offset_pitch = title_space;
                    float y_offset_roll = y_offset_pitch + plot_height + spacing + title_space;
                    float y_offset_yaw = y_offset_roll + plot_height + spacing + title_space;
                    
                    // Draw three plots
                    draw_plot("Pitch (φ)", pitch_values.data(), IM_COL32(255, 0, 0, 255), 
                             pitch_min, pitch_max, y_offset_pitch, plot_height);
                    
                    draw_plot("Roll (γ)", roll_values.data(), IM_COL32(0, 255, 0, 255), 
                             roll_min, roll_max, y_offset_roll, plot_height);
                    
                    draw_plot("Yaw (ψ)", yaw_values.data(), IM_COL32(0, 100, 255, 255), 
                             yaw_min, yaw_max, y_offset_yaw, plot_height);
                    
                    // Draw time axis label on the last plot
                    float time_label_y = plot_start_pos.y + y_offset_yaw + plot_height + 5.0f;
                    ImVec2 time_label_pos = ImVec2(plot_x + plot_width * 0.5f - 50.0f, time_label_y);
                    draw_list->AddText(time_label_pos, IM_COL32(200, 200, 200, 255), "Flight Time (s)");
                } else {
                    ImGui::Spacing();
                    ImGui::TextDisabled("No attitude plot data available");
                    ImGui::TextDisabled("Run a simulation to see attitude plots");
                }

                ImGui::EndTabItem();
            }

            // ===== ENGINE PARAMETERS TAB =====
            if (ImGui::BeginTabItem("Engine")) {
                ImGui::Spacing();

                // Mass & Fuel
                ImGui::Text("Mass & Propellant");
                ImGui::Text("  Total Mass (Given):   %8.2f kg", telemetry.total_mass);
                ImGui::Text("  Vehicle Mass:         %8.2f kg", telemetry.vehicle_mass);
                ImGui::Text("  Fuel Consumed:        %8.2f kg", telemetry.fuel_consumed);
                ImGui::Text("  Fuel Remaining:       %8.2f kg", telemetry.fuel_remaining);
                ImGui::Text("  Consumption Rate:     %8.3f kg/s", telemetry.fuel_consumption_rate);

                ImGui::Spacing();
                ImGui::Separator();

                // Engine Parameters
                ImGui::Text("Engine Parameters");
                ImGui::Text("  Thrust:        %10.2f N", telemetry.thrust);
                ImGui::Text("  Exit Velocity: %10.2f m/s", telemetry.exit_velocity);
                ImGui::Text("  Specific Impulse (Isp): %6.2f s", telemetry.isp);

                ImGui::EndTabItem();
            }

            ImGui::EndTabBar();
        }

            ImGui::End();
        }

        // ===== NAVIGATION ERRORS WINDOW =====
        if (show_errors) {
            ImGui::SetNextWindowPos(ImVec2(1240, 10), ImGuiCond_FirstUseEver);
            ImGui::SetNextWindowSize(ImVec2(350, 680), ImGuiCond_FirstUseEver);
            ImGui::Begin("Navigation Errors", &show_errors);

        if (telemetry.is_running || simulation_complete) {
            // Position Errors
            ImGui::Text("Position Errors (m)");
            ImGui::Separator();
            ImGui::Text("δx: %12.2f", telemetry.delta_pos_x);
            ImGui::Text("δy: %12.2f", telemetry.delta_pos_y);
            ImGui::Text("δz: %12.2f", telemetry.delta_pos_z);

            ImGui::Spacing();
            ImGui::Separator();

            // Velocity Errors
            ImGui::Text("Velocity Errors (m/s)");
            ImGui::Separator();
            ImGui::Text("δVx: %10.2f", telemetry.delta_vel_x);
            ImGui::Text("δVy: %10.2f", telemetry.delta_vel_y);
            ImGui::Text("δVz: %10.2f", telemetry.delta_vel_z);

            ImGui::Spacing();
            ImGui::Separator();

            // Orbital Element Errors
            ImGui::Text("Orbital Element Errors");
            ImGui::Separator();
            ImGui::Text("δa (Semi-major Axis): %10.2f m", telemetry.delta_semi_major_axis);
            ImGui::Text("δe (Eccentricity):    %.6f", telemetry.delta_eccentricity);
            ImGui::Text("δi (Inclination):     %10.4f°", telemetry.delta_inclination);
            ImGui::Text("δΩ (RAAN):            %10.4f°", telemetry.delta_right_ascension);
            ImGui::Text("δω (Arg of Perigee):  %10.4f°", telemetry.delta_argument_of_perigee);
            ImGui::Text("δν (True Anomaly):    %10.4f°", telemetry.delta_true_anomaly);

        } else {
            ImGui::TextDisabled("No error data available");
            ImGui::TextDisabled("Run a simulation to see errors");
        }

            ImGui::End();
        }

        // ===== ORBITAL ELEMENTS WINDOW =====
        if (show_orbital_elements) {
            ImGui::SetNextWindowPos(ImVec2(1600, 10), ImGuiCond_FirstUseEver);
            ImGui::SetNextWindowSize(ImVec2(280, 500), ImGuiCond_FirstUseEver);
            ImGui::Begin("Orbital Elements", &show_orbital_elements);

        if (telemetry.is_running || simulation_complete) {
            ImGui::Separator();

            // 1. Semi-major Axis (a)
            ImGui::Text("1. Semi-major Axis (a):");
            ImGui::Text("   %12.2f km", telemetry.semi_major_axis / 1000.0);

            ImGui::Spacing();
            ImGui::Separator();

            // 2. Eccentricity (e)
            ImGui::Text("2. Eccentricity (e):");
            ImGui::Text("   %.6f", telemetry.eccentricity);

            ImGui::Spacing();
            ImGui::Separator();

            // 3. Inclination (i)
            ImGui::Text("3. Inclination (i):");
            ImGui::Text("   %.4f°", telemetry.inclination);

            ImGui::Spacing();
            ImGui::Separator();

            // 4. Right Ascension (Ω)
            ImGui::Text("4. RAAN (Ω):");
            ImGui::Text("   %.4f°", telemetry.right_ascension);

            ImGui::Spacing();
            ImGui::Separator();

            // 5. Argument of Perigee (ω)
            ImGui::Text("5. Arg of Perigee (ω):");
            ImGui::Text("   %.4f°", telemetry.argument_of_perigee);

            ImGui::Spacing();
            ImGui::Separator();

            // 6. True Anomaly (ν)
            ImGui::Text("6. True Anomaly (ν):");
            ImGui::Text("   %.4f°", telemetry.true_anomaly);

        } else {
            ImGui::TextDisabled("No orbital data available");
            ImGui::TextDisabled("Run a simulation to see orbital elements");
        }

            ImGui::End();
        }


        // Rendering
        ImGui::Render();
        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(0.45f, 0.55f, 0.60f, 1.00f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
    }

    // Cleanup
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;
}
