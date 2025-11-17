//
//  math.hpp
//  FlightSim
//
//  Created by Vincent Ning on 8/7/24.
//

#ifndef math_hpp
#define math_hpp

#include "sim.hpp"
#include "constants.h"
// 前向声明
namespace rkt { class rocket; }
class gravity_model;
class atmosphere_model;
class engine_model;
class Guidance;
class aerodynamics;

extern std::vector<double> add_vectors(const std::vector<std::vector<double>>& vectors);

extern std::vector<double> coord2carte(const std::vector<double>& coord);

extern std::vector<double> scalar_vector(double a, const std::vector<double>& V);

extern std::vector<double> cross_product33(const std::vector<double>& a, const std::vector<double>& b);

extern double dot_product(const std::vector<double>& a, const std::vector<double>& b);

extern std::vector<std::vector<double>> scalar_matrix(double a, const std::vector<std::vector<double>>& b);

extern std::vector<double> vector_add(const std::vector<double>& a, const std::vector<double>& b);

extern std::vector<double> subtract_vectors(const std::vector<double>& v1, const std::vector<double>& v2);

extern std::vector<double> Matrix_multiply_vector(const std::vector<std::vector<double>>& a, const std::vector<double>& b);

extern std::vector<std::vector<double>> rotation_matrix_x(double alpha);

extern std::vector<std::vector<double>> rotation_matrix_y(double alpha);

extern std::vector<std::vector<double>> rotation_matrix_z(double alpha);

extern std::vector<std::vector<double>> multiply_matrices(const std::vector<std::vector<double>>& A, const std::vector<std::vector<double>>& B);

extern std::vector<std::vector<double>> rotation_matrix_inertial_to_rocket(const std::vector<double>& cmd);

extern std::vector<std::vector<double>> rotation_matrix_rocket_to_inertial(const std::vector<double>& cmd);

extern std::vector<std::vector<double>> rotation_matrix_velocity_to_body(double alpha, double beta);

extern std::vector<double> launch_inertial_to_ECSF(const std::vector<double>& PI);

//extern std::vector<std::vector<double>> rotation_matrix_ECSF_to_terminal_guidance(double aiming_azimuth, double launch_lattitude, double inclination, double longitude_to_descending_node, double injection_angle);
extern std::vector<std::vector<double>> rotation_matrix_inertial_to_terminal_guidance(double beta_c);

extern std::vector<std::vector<double>> rotation_matrix_inertial_to_launch(double t);

extern std::vector<std::vector<double>> rotation_matrix_inertial_to_launch_simplified(double t);

extern std::vector<std::vector<double>> transpose_matrix(const std::vector<std::vector<double>>& matrix);

extern double vector_mag(const std::vector<double>& a);

extern std::vector<size_t> ascending_binary_search(const std::vector<std::vector<double>>& table, int column, double num);

extern std::vector<size_t> ascending_binary_search_bounded(const std::vector<std::vector<double>>& table, int column, double num, size_t left, size_t right);

extern std::vector<size_t> descending_binary_search(const std::vector<std::vector<double>>& table, int column, double num);

extern void rk4(double t, int step_counter, rkt::rocket& rocket, gravity_model& gravity, atmosphere_model& atmosphere, engine_model& engine, Guidance& guidance, aerodynamics& aero, flight_states states);

#endif /* math_hpp */
