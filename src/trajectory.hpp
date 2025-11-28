#pragma once
#include "constants.h"
#include <vector>
//orbit parameters at TECO

const double a_terminal = 6903085.00;
const double right_ascension = 260.861370  * D2R;
const double inclination = 97.496919 * D2R; // 90-180 逆行轨道（相对于地球反方向旋转）
const double eccentricity = 8.851433e-06;
//const double time_to_SECO = t_end - t_2;
const double longitude_to_descending_node_SECO = (174.697  + 90.0) * D2R; //从赤道坐标系x轴为参照升交点赤经为180.009，旋转x轴到升交点赤经，再转90度将y轴转到降交点
const double longitude_to_ascending_node_SECO = (174.697  - 90.0) * D2R;
const double true_anomaly = 154.051241 * D2R;
const double arugument_of_perigee = 185.130932 * D2R;
// 逆行轨道, 从z轴看下去顺时针为负角度
const double range_angle_initial_AN = arugument_of_perigee + true_anomaly;
//发惯系x轴与北的夹角 in radians
const double A0 = 191.47506 * D2R;
//地理纬度 in radians
const double B0 = 40.80768 * D2R;
//地心纬度
const double Phi = 40.61741 * D2R;
//launch site coordinates
//大地经度
const double longitude = 100.13805 * D2R;
//大地纬度
const double latitude = 40.80768 * D2R;
const double height_launch_site = 1000;
const double initial_COM_first_stage = 62.7;

const double vx_SECO = -7124.1826; 
const double vy_SECO = 2631.5395; 
const double vz_SECO = 249.9496; 
const double x_SECO = -2405466.9; 
const double y_SECO = -12845047.6 ; 
const double z_SECO = 205013.6; 

const double vx_TECO = -7124.1826;
const double vy_TECO = 2631.5395; 
const double vz_TECO = 249.9496; 
const double x_TECO = -2405466.9; 
const double y_TECO = -12845047.6 ; 
const double z_TECO = 205013.6; 
//calculate ECSF to guidance_terminal rotation matrix without considering injection angle.
const std::vector<std::vector<double>> C_ea = {{-cos(A0)*sin(B0), cos(B0), sin(A0)*sin(B0)},
                                                {sin(A0), 0, cos(A0)},
                                                {cos(A0)*cos(B0), sin(B0), -sin(A0)*cos(B0)}};

const std::vector<std::vector<double>> C = {{cos(longitude_to_ascending_node_SECO), sin(longitude_to_ascending_node_SECO), 0}, {-sin(longitude_to_ascending_node_SECO), cos(longitude_to_ascending_node_SECO), 0},
    {0, 0, 1}}; //checked✅
//如果y轴对准降交点，那么轨道倾角相当于y对准升交点的负数，
const std::vector<std::vector<double>> D = {{cos(inclination), 0 , -sin(inclination)}, {0, 1, 0}, {sin(inclination), 0, cos(inclination)}}; //checked✅

const std::vector<std::vector<double>> G = multiply_matrices(D, multiply_matrices(C, C_ea));
const double e_b = sqrt(a * a - b * b) / b;
const double phi = atan(1 / (1 + pow(e_b, 2)) * tan(latitude));
const double R0 = a * b / sqrt(pow(a,2) * pow(sin(phi), 2) + pow(b,2) * pow(cos(phi), 2)) + height_launch_site;
const std::vector<double> R0_vec  = {-R0 * sin(latitude - phi) * cos(A0), R0 * cos(latitude - phi), R0 * sin(A0) * sin(latitude - phi)};
const std::vector<double> omega_e0 = {cos(A0)*cos(B0), sin(B0), -cos(B0)*sin(A0)};
const std::vector<std::vector<double>> omegae0_cross = {{0, -omega_e0[2] , omega_e0[1]}, 
                                            {omega_e0[2], 0 , -omega_e0[0]}, 
                                            {-omega_e0[1], omega_e0[0], 0}};

const std::vector<double> launch_coordinates = {longitude, latitude, height_launch_site};

const std::vector<double> Pi_terminal = {x_SECO, y_SECO, z_SECO};
const std::vector<double> Vi_terminal = {vx_SECO, vy_SECO, vz_SECO};

//===========Stage-Specific Parameters for GUI=============

//===========Third Stage=============
// Time parameters
const float initial_time3 = 3140.3405f;
const float end_time3 = 3344.5194f;

// Initial state vectors (inertial frame)
const float initial_position3[3] = {-912205.4f, -13212262.5f, 149163.6f};
const float initial_velocity3[3] = {-7440.6404f, 953.8138f, 294.5883f};
const float initial_mass3 = 6731.1f;

const float initial_pitch3 = 172.7593f;
const float initial_yaw3 = 0.0f;
const float initial_roll3 = 0.0f;

const double terminal_position3[3] = {-2405466.9, -12845047.6 , 205013.6};
const double terminal_velocity3[3] = {-7124.1826, 2631.5395, 249.9496};
const double a_terminal3 = 6903085.00;
const double right_ascension3 = 260.861370  * D2R;
const double inclination3 = 97.496919 * D2R;
const double eccentricity3 = 8.851433e-06;
const double longitude_to_descending_node3 = (174.7704  + 90.0) * D2R;
const double longitude_to_ascending_node3 = (174.7704  - 90.0) * D2R;
const double true_anomaly3 = 154.051241 * D2R;
const double arugument_of_perigee3 = 185.130932 * D2R;
const double range_angle_initial_AN3 = arugument_of_perigee3 + true_anomaly3;

//===========Second Stage=============
// Time parameters
const float initial_time2 = 149.8000f;
const float end_time2 = 470.4166f;

// Initial state vectors (inertial frame)
const float initial_position2[3] = {91748.3f, 74465.8f, -51380.7f};
const float initial_velocity2[3] = {2211.1545f, 1037.2763f, -338.5107f};
const float initial_mass2 = 121175.5f;

const float initial_pitch2 = 27.0329f;
const float initial_yaw2 = 0.0f;
const float initial_roll2 = 0.0f;

const double terminal_position2[3] = {1427834.9f, 45222.2f, -153813.3f};
const double terminal_velocity2[3] = {7677.0525f, -1739.1305f,  -290.0255f};

const double a_terminal2 = 6740599.40;
const double right_ascension2 = 272.847611  * D2R;
const double inclination2 = 97.461979 * D2R;
const double eccentricity2 = 2.402729e-02;
const double longitude_to_descending_node2 = (174.6916  + 90.0) * D2R;
const double longitude_to_ascending_node2 = (174.6916  - 90.0) * D2R;
const double true_anomaly2 = 0.451858 * D2R;
const double arugument_of_perigee2 = 151.230742 * D2R;
const double range_angle_initial_AN2 = arugument_of_perigee2 + true_anomaly2;

//===========First Stage=============
// Time parameters
const float initial_time1 = 0.0f;
const float end_time1 = 149.8000f;

// Initial state vectors (inertial frame)
const float initial_position1[3] = {0.0f, 62.7f, 0.0f};
const float initial_velocity1[3] = {-70.1471f, 0.1000f, -345.5538f};
const float initial_mass1 = 581499.9f;

const float initial_pitch1 = 90.0f;
const float initial_yaw1 = 0.0f;
const float initial_roll1 = 0.0f;

const double terminal_position1[3] = {91748.3, 74465.8, -51380.7};
const double terminal_velocity1[3] = {2211.1545, 1037.2763, -338.5107};