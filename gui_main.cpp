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

// Your simulation includes
#include "sim.hpp"
#include "rocket.hpp"
#include "math.hpp"
#include "gravity.hpp"
#include "aerodynamics.hpp"
#include "Atmosphere_properties.hpp"
#include "GNC.hpp"
#include "engine_model.hpp"
#include "utilities.hpp"
#include "trajectory.hpp"

static void glfw_error_callback(int error, const char* description) {
    fprintf(stderr, "GLFW Error %d: %s\n", error, description);
}

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

    // Setup Dear ImGui style
    ImGui::StyleColorsDark();

    // Setup Platform/Renderer backends
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);

    // GUI State Variables
    static char euler_angles_path[256] = "euler angles.csv";
    static char v_inertial_path[256] = "V_inertial.csv";
    static char p_inertial_path[256] = "P_inertial.csv";
    static char highspeed_path[256] = "highspeed.csv";
    static char lowspeed_path[256] = "lowspeed.csv";

    static float initial_time = 3140.3405f;
    static float end_time = 3344.5194f;
    static float initial_position[3] = {-912205.4f, -13212262.5f, 149163.6f};
    static float initial_velocity[3] = {-7440.6404f, 953.8138f, 294.5883f};
    static float initial_mass = 6731.1f;
    static float steering_angle = 172.7593f;

    bool run_simulation = false;
    bool show_demo_window = true;

    // Main loop
    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();

        // Start ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // Demo window (for learning ImGui)
        if (show_demo_window)
            ImGui::ShowDemoWindow(&show_demo_window);

        // Main GUI Window
        ImGui::Begin("Flight Simulation Control Panel");

        ImGui::Text("CSV File Inputs");
        ImGui::Separator();
        ImGui::InputText("Euler Angles", euler_angles_path, IM_ARRAYSIZE(euler_angles_path));
        ImGui::SameLine();
        if (ImGui::Button("Browse##1")) {
            // TODO: Add file browser
        }

        ImGui::InputText("V Inertial", v_inertial_path, IM_ARRAYSIZE(v_inertial_path));
        ImGui::SameLine();
        if (ImGui::Button("Browse##2")) {
            // TODO: Add file browser
        }

        ImGui::InputText("P Inertial", p_inertial_path, IM_ARRAYSIZE(p_inertial_path));
        ImGui::SameLine();
        if (ImGui::Button("Browse##3")) {
            // TODO: Add file browser
        }

        ImGui::InputText("High Speed Aero", highspeed_path, IM_ARRAYSIZE(highspeed_path));
        ImGui::SameLine();
        if (ImGui::Button("Browse##4")) {
            // TODO: Add file browser
        }

        ImGui::InputText("Low Speed Aero", lowspeed_path, IM_ARRAYSIZE(lowspeed_path));
        ImGui::SameLine();
        if (ImGui::Button("Browse##5")) {
            // TODO: Add file browser
        }

        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Text("Simulation Parameters");
        ImGui::Separator();

        ImGui::InputFloat("Initial Time (s)", &initial_time);
        ImGui::InputFloat("End Time (s)", &end_time);
        ImGui::InputFloat("Initial Mass (kg)", &initial_mass);
        ImGui::InputFloat("Steering Angle (deg)", &steering_angle);

        ImGui::Text("Initial Position (m)");
        ImGui::InputFloat("Px", &initial_position[0]);
        ImGui::InputFloat("Py", &initial_position[1]);
        ImGui::InputFloat("Pz", &initial_position[2]);

        ImGui::Text("Initial Velocity (m/s)");
        ImGui::InputFloat("Vx", &initial_velocity[0]);
        ImGui::InputFloat("Vy", &initial_velocity[1]);
        ImGui::InputFloat("Vz", &initial_velocity[2]);

        ImGui::Spacing();
        ImGui::Separator();

        if (ImGui::Button("Run Simulation", ImVec2(200, 40))) {
            run_simulation = true;
            printf("Starting simulation...\n");
            // TODO: Call your simulation code here
        }

        ImGui::SameLine();
        ImGui::Checkbox("Show ImGui Demo", &show_demo_window);

        ImGui::End();

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
