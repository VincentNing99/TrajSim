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
// 卫星为逆行轨道, 从z轴看下去顺时针为负角度
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

const double vx_SECO = -7124.1826; // x velocity in inertial reference frame at SECO
const double vy_SECO = 2631.5395; // y velocity in inertial reference frame at SECO
const double vz_SECO = 249.9496; // z velocity in inertial reference frame at SECO
const double x_SECO = -2405466.9; // x position in inertial reference frame at SECO
const double y_SECO = -12845047.6 ; // y position in inertial reference frame at SECO
const double z_SECO = 205013.6; // z position in inertial reference frame at SECO

const double vx_TECO = -7124.1826; // x velocity in inertial reference frame at SECO
const double vy_TECO = 2631.5395; // y velocity in inertial reference frame at SECO
const double vz_TECO = 249.9496; // z velocity in inertial reference frame at SECO
const double x_TECO = -2405466.9; // x position in inertial reference frame at SECO
const double y_TECO = -12845047.6 ; // y position in inertial reference frame at SECO
const double z_TECO = 205013.6; // z position in inertial reference frame at SECO
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

const double height_initial = 77054.0;
