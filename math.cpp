//
//  math.cpp
//  FlightSim
//
//  Created by Vincent Ning on 8/7/24.
//
#include "math.hpp"
#include "sim.hpp"
#include "rocket.hpp"
#include "gravity.hpp"
#include "aerodynamics.hpp"
#include "Atmosphere_properties.hpp"
#include "GNC.hpp"
#include "engine_model.hpp"
#include "trajectory.hpp"
using namespace std;



vector<vector<double>> scalar_matrix(double a, const vector<vector<double>> b) {
    vector<vector<double>> result(b.size(), vector<double>(b[0].size()));
    for (size_t i = 0; i < b.size(); ++i) {
        for (size_t j = 0; j < b[0].size(); ++j) {
            result[i][j] = a * b[i][j];
        }
    }
    return result;
}
vector<double> add_vectors(const vector<vector<double>>& vectors) {
    if (vectors.empty()) {
        // Handle empty input
        return {};
    }

    size_t num_vectors = vectors.size();
    size_t size = vectors[0].size();

    vector<double> result(size);

    for (size_t i = 0; i < size; ++i) {
        for (size_t j = 0; j < num_vectors; ++j) {
            result[i] += vectors[j][i];
        }
    }

    return result;
}

vector<double> subtract_vectors(const vector<double> v1, const vector<double> v2) {
    if (v1.empty() || v2.empty()) {
        // Handle empty input
        throw "empty vectors!";
    }
    size_t size = v1.size();

    vector<double> result(size);

    for (size_t i = 0; i < size; ++i) {
        result[i] = v1[i] - v2[i];
    }

    return result;
}

vector<double> scalar_vector(double a, vector<double> V) {
    vector<double> result(V.size());
    for (size_t i = 0; i < V.size(); ++i) {
        result[i] = a * V[i];
    }
    return result;
}

double dot_product(vector<double> a, vector<double> b)
{
    double result = 0;;
    if(a.size() != b.size())
    {
        throw invalid_argument("Vectors must be the same in size for dot product");
    }
    
    for (int i = 0; i < a.size(); i++)
    {
        result += a[i] * b[i];
    }
    return result;
}

vector<double> cross_product33(vector<double> a, vector<double> b) {
    if (a.size() != 3 || b.size() != 3) {
        throw invalid_argument("Vectors must be of size 3 for cross product.");
    }

    vector<double> result(3);
    result[0] = a[1] * b[2] - a[2] * b[1];
    result[1] = a[2] * b[0] - a[0] * b[2];
    result[2] = a[0] * b[1] - a[1] * b[0];

    return result;
}
double vector_mag(vector<double> a) {
    double sum_of_squares = 0.0;
    for ( auto element : a) {
        sum_of_squares += element * element;
    }
    return sqrt(sum_of_squares);
}

vector<double> Matrix_multiply_vector(vector<vector<double>> a, vector<double> b) {
    if (a.empty() || a[0].size() != b.size()) {
        throw std::invalid_argument("Invalid matrix or vector dimensions.");
    }

    vector<double> result(a.size(), 0);
    for (size_t i = 0; i < a.size(); ++i) {
        for (size_t j = 0; j < b.size(); ++j) {
            result[i] += a[i][j] * b[j];
        }
    }

    return result;
}

vector<double> vector_add(vector<double> a, vector<double> b) {
    // Check if vectors have the same size
    if (a.size() != b.size()) {
        throw std::invalid_argument("Vectors must have the same size.");
    }

    vector<double> result(a.size());
    for (size_t i = 0; i < a.size(); ++i) {
        result[i] = a[i] + b[i];
    }

    return result;
}
//matrix transfromation, cw rotation is positive look down at the axis of rotation
vector<vector<double>> rotation_matrix_x(double alpha) {
    vector<vector<double>> Mx(3, vector<double>(3,0));
    Mx[0][0] = 1;
    Mx[1][1] = cos(alpha);
    Mx[1][2] = sin(alpha);
    Mx[2][1] = -sin(alpha);
    Mx[2][2] = cos(alpha);
return Mx;
}


vector<vector<double>> rotation_matrix_y(double alpha) {
    vector<vector<double>> My(3, vector<double>(3, 0));
    My[0][0] = cos(alpha);
    My[0][2] = -sin(alpha);
    My[1][0] = 0;
    My[1][1] = 1;
    My[1][2] = 0;
    My[2][0] = sin(alpha);
    My[2][1] = 0;
    My[2][2] = cos(alpha);
    return My;
}

vector<vector<double>> rotation_matrix_z(double alpha) {
    vector<vector<double>> Mz(3, vector<double>(3, 0));
    Mz[0][0] = cos(alpha);
    Mz[0][1] = sin(alpha);
    Mz[1][0] = -sin(alpha);
    Mz[1][1] = cos(alpha);
    Mz[2][2] = 1;
    return Mz;
}

vector<vector<double>> multiply_matrices( vector<vector<double>> A,  vector<vector<double>> B) {
    unsigned long rowsA = A.size();
    unsigned long colsA = A[0].size();
    unsigned long colsB = B[0].size();

    vector<vector<double>> result(rowsA, vector<double>(colsB, 0));

    for (int i = 0; i < rowsA; ++i) {
        for (int j = 0; j < colsB; ++j) {
            for (int k = 0; k < colsA; ++k) {
                result[i][j] += A[i][k] * B[k][j];
            }
        }
    }

    return result;
}
//发惯至地心惯性坐标系
vector<double> launch_inertial_to_ECSF(vector<double> Pi) {
    vector<double> result;
    result = vector_add(Pi, R0_vec);
    return result;
}
//地心惯性坐标系至终端引导坐标系

vector<vector<double>> rotation_matrix_inertial_to_terminal_guidance(double range_angle)
{
    vector<vector<double>> M_range_angle(3, vector<double>(3, 0));
    M_range_angle = {{cos(range_angle), sin(range_angle), 0}, {-sin(range_angle), cos(range_angle), 0},{0, 0, 1}};
    return M_range_angle;
}

vector<vector<double>> rotation_matrix_inertial_to_launch(double t)
{
    double g11 = pow(cos(A0),2)* pow(cos(B0),2)*(1 - cos(omega_e * t)) + cos(omega_e * t);
    double g12 = cos(A0)*sin(B0)*cos(B0)*(1 - cos(omega_e * t)) - sin(A0) * cos(B0) * sin(omega_e * t);
    double g13 = -sin(A0)*cos(A0)*pow(cos(B0), 2) * (1 - cos(omega_e * t)) - sin(A0) * sin(omega_e * t);
    double g21 = cos(A0)*sin(B0)*cos(B0)*(1 - cos(omega_e * t)) + sin(A0)*cos(B0)*sin(omega_e * t);
    double g22 = pow(sin(B0), 2) * (1 - cos(omega_e * t)) + cos(omega_e * t);
    double g23 = -sin(A0)*sin(B0)*cos(B0)*(1 - cos(omega_e * t)) + cos(A0)*cos(B0)*sin(omega_e * t);
    double g31 = -sin(A0)*cos(A0)*pow(cos(B0), 2)*(1 - cos(omega_e * t)) + sin(B0) * sin(omega_e * t);
    double g32 = -sin(A0)*sin(B0)*cos(B0)*(1 - cos(omega_e * t)) - cos(A0)*cos(B0)*sin(omega_e * t);
    double g33 = pow(sin(A0), 2)*pow(cos(B0), 2)*(1 - cos(omega_e * t)) + cos(omega_e * t);
    vector<vector<double>> M(3, vector<double>(3, 0));
    M[0][0] = g11;
    M[0][1] = g12;
    M[0][2] = g13;
    M[1][0] = g21;
    M[1][1] = g22;
    M[1][2] = g23;
    M[2][0] = g31;
    M[2][1] = g32;
    M[2][2] = g33;
    return M;
}


vector<vector<double>> rotation_matrix_rocket_to_inertial(vector<double> cmd) {
    double phi = -cmd[0];
    double psi = -cmd[1];
    double gamma = -cmd[2];
    vector<vector<double>> Mz = rotation_matrix_z(phi);
    vector<vector<double>> My = rotation_matrix_y(psi);
    vector<vector<double>> Mx = rotation_matrix_x(gamma);

    vector<vector<double>> temp = multiply_matrices(My, Mx);
    vector<vector<double>> result = multiply_matrices(Mz, temp);
    return result;
}

vector<vector<double>> rotation_matrix_inertial_to_rocket(vector<double> cmd)
{
    double phi = cmd[0];
    double psi = cmd[1];
    double gamma = cmd[2];
    vector<vector<double>> Mz = rotation_matrix_z(phi);
    vector<vector<double>> My = rotation_matrix_y(psi);
    vector<vector<double>> Mx = rotation_matrix_x(gamma);

    vector<vector<double>> temp = multiply_matrices(My, Mz);
    vector<vector<double>> result = multiply_matrices(Mx, temp);
    return result;
}

//速度至箭体
vector<vector<double>> rotation_matrix_velocity_to_body(double alpha, double beta) {
    vector<vector<double>> result(3, vector<double>(3, 0));
    result = {{cos(alpha)*cos(beta), -sin(alpha), cos(alpha)*sin(beta)},
              {sin(alpha)*cos(beta), cos(alpha), sin(alpha)*sin(beta)},
              {-sin(beta), 0 , cos(beta)}};
    return result;
}

vector<vector<double>> transpose_matrix(vector<vector<double>> matrix) {
    unsigned long rows = matrix.size();
    unsigned long cols = matrix[0].size();
    vector<vector<double>> transposed(cols, vector<double>(rows));

    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            transposed[j][i] = matrix[i][j];
        }
    }

    return transposed;
}

vector<size_t> ascending_binary_search(vector<vector<double>> table, int coloum, double num)
{
    vector<size_t> result;
    size_t row_n = table.size();

    size_t left, right;
    if(coloum > row_n){
        throw std::invalid_argument("coloum out of range");
    }
    
    right = row_n - 1;
    left = 0;
    if(num > table[right][coloum - 1] || num < table[left][coloum - 1]) {
        throw std::invalid_argument("value out of range");
    }
    while(left <= right)
    {
        size_t mid = left + (right - left) / 2;
        if(table[left][coloum - 1] == num) {
            result = {left, left};
            return result;
        }
        else if(table[right][coloum - 1] == num) {
            result = {right, right};
            return result;
        }
        if (table[mid][coloum - 1] > num)
        {
            right = mid;
        }
        else if (table[mid][coloum - 1] < num)
        {
            left = mid;
        }
        if(table[mid][coloum - 1] > num && table[mid - 1][coloum-1] < num) {
            left = mid - 1;
            result = {left, right};
            return result;
        }
        else if (table[mid][coloum - 1] < num && table[mid + 1][coloum-1] > num)
        {
            right = mid + 1;
            result = {left, right};
            return result;
        }
        else if(table[mid][coloum - 1] == num)
        {
            result = {mid, mid};
            return result;
        }
    }
    return result;
}

vector<size_t> ascending_binary_search_bounded(vector<vector<double>> table, int coloum, double num, size_t left, size_t right)
{
    vector<size_t> result;
    size_t row_n = table.size();
    
    if(coloum > row_n){
        throw std::invalid_argument("coloum out of range");
    }
    
    if(num > table[right][coloum - 1] || num < table[left][coloum - 1]) {
        throw std::invalid_argument("value out of range");
    }
    while(left <= right)
    {
        size_t mid = left + (right - left) / 2;
        if(table[left][coloum - 1] == num) {
            result = {left, left};
            return result;
        }
        else if(table[right][coloum - 1] == num) {
            result = {right, right};
            return result;
        }
        if (table[mid][coloum - 1] > num)
        {
            right = mid;
        }
        else if (table[mid][coloum - 1] < num)
        {
            left = mid;
        }
        if(table[mid][coloum - 1] > num && table[mid - 1][coloum-1] < num) {
            left = mid - 1;
            result = {left, right};
            return result;
        }
        else if (table[mid][coloum - 1] < num && table[mid + 1][coloum-1] > num)
        {
            right = mid + 1;
            result = {left, right};
            return result;
        }
        else if(table[mid][coloum - 1] == num)
        {
            result = {mid, mid};
            return result;
        }
    }
    return result;
}

vector<size_t> descending_binary_search(vector<vector<double>> table, int coloum, double num)
{
    vector<size_t> result;
    size_t row_n = table.size();
    size_t left, right;
    if(coloum > row_n){
        throw std::invalid_argument("coloum out of range");
    }
    right = row_n - 1;
    left = 0;
    while(left <= right)
    {
        if(num < table[right][coloum - 1] || num > table[left][coloum - 1]) {
            throw std::invalid_argument("value out of range");
        }
        size_t mid = left + (right - left) / 2;
        if(table[left][coloum - 1] == num) {
            result = {left, left};
            return result;
        } else if(table[right][coloum - 1] == num) {
            result = {right, right};
            return result;
        }
        if (table[mid][coloum - 1] > num)
        {
            left = mid;
        }
        else if (table[mid][coloum - 1] < num)
        {
            right = mid;
        }
        if(table[mid][coloum - 1] < num && table[mid - 1][coloum-1] > num) {
            left = mid - 1;
            result = {left, right};
            return result;
        }
        else if (table[mid][coloum - 1] > num && table[mid + 1][coloum-1] < num)
        {
            right = mid + 1;
            result = {left, right};
            return result;
        }
        else if(table[mid][coloum - 1] == num)
        {
            result = {mid, mid};
            return result;
        }
    }
    return result;
}

vector<double> coord2carte(const vector<double> coord)
{
    //Geocentric latitude beta
    vector<double> result;
    double beta, N, x,y,z;
    double e = sqrt(1 - (b*b/a*a));
    beta = atan((b*b/a*a) * tan(coord[0]));
    N = a / sqrt(1 - e * e * sin(beta) * sin(beta));
    x = (N + coord[2]) * cos(beta) * cos(coord[1]);
    y = (N + coord[2]) * cos(beta) * sin(coord[1]);
    z =  ((1 - pow(e, 2)) * N + coord[2]) * sin(beta);
    result = {x,y,z};
    return result;
}

// Function to calculate derivatives
vector<double> calculate_derivatives(double t, rkt::rocket& rocket, 
                                    gravity_model& gravity, atmosphere_model& atmosphere, 
                                    engine_model& engine, Guidance& guidance, 
                                    aerodynamics& aero, flight_states states) {
    vector<double> derivatives(7); // 3 for position, 3 for velocity, 1 for mass
    vector<vector<double>> MbI, MIb,result;//inertial to body rotation matrix
    vector<double>g, g_IGM, aero_force, thrust_force, total_force_body, total_force_inertial;
    // 获取当前状态
    const auto& Pi = rocket.get_PI();
    const auto& Vi = rocket.get_VI();
    switch(states)
    {
        case flight_states::YJFX:
            // 计算重力加速度
            g = gravity.calc_g(Pi);
            // 计算攻角和侧滑角
            rocket.angle_calc(rocket.get_attitude());
            // 更新大气参数
            atmosphere.atmosphere_para(rocket.get_Vb(), gravity.get_altitude(rocket.get_PI()));
            // 计算旋转矩阵
            MIb = rotation_matrix_inertial_to_rocket(rocket.get_attitude());
            MbI = rotation_matrix_rocket_to_inertial(rocket.get_attitude());
            //aerodynamics
            result = aero.calc_aerodynamics(atmosphere.get_mach_number(), rocket.get_sideslip_angle(), rocket.get_attack_angle(), atmosphere.get_q());
            aero_force = result[0];
            // 计算推力
            result = engine.thrust(guidance.get_servo_cmd(), t, atmosphere.get_air_pressure(), rocket.get_fstate(), rocket.get_COM());
            thrust_force = result[0];
            // 计算总力
            total_force_body = vector_add(thrust_force, aero_force);
            total_force_inertial = Matrix_multiply_vector(MbI, total_force_body);
            break;

        case flight_states::EJFX:
        case flight_states::SJFX:
            g = gravity.calc_g(Pi);
            rocket.angle_calc(rocket.get_attitude());
            MIb = rotation_matrix_inertial_to_rocket(rocket.get_attitude());
            MbI = transpose_matrix(MIb);
            result = engine.thrust(guidance.get_servo_cmd(), t, atmosphere.get_air_pressure(), rocket.get_fstate(), rocket.get_COM());
            thrust_force = result[0];
            total_force_inertial = Matrix_multiply_vector(MbI, thrust_force);
            break;

        default:
        break;
    }
    // 计算加速度
    double inv_mass = 1.0 / rocket.get_mass();
    for (int i = 0; i < 3; i++) {
        derivatives[i] = Vi[i];
        derivatives[i+3] = total_force_inertial[i] * inv_mass + g[i];
    }
    
    // 计算质量变化率
    derivatives[6] = rocket.get_del_m();
    return derivatives;
}

// RK4 integration step
void rk4(double t, int step_counter, rkt::rocket& rocket, gravity_model& gravity, atmosphere_model& atmosphere, engine_model& engine, Guidance& guidance, aerodynamics& aero, flight_states states)
{
    vector<double> g_IGM;
    double dt_half = step / 2.0;

    if(step_counter % 10 == 0 && guidance.get_time_to_go() > IGM_stop_time)
    {
        if(rocket.get_fstate() != flight_states::YJFX)
        {
            g_IGM = gravity.get_g_terminal_guidance(rocket.get_PI(),
                                                    {x_SECO,y_SECO,z_SECO},
                                                    guidance.get_terminal_guidance_rotation_matrix());
        }
        if(step_counter != 0)
        {
            if(rocket.get_fstate() == flight_states::EJFX)
            {
                guidance.set_time_to_go();
            }

            vector<double> attitude = guidance.get_attitude(t, g_IGM , rocket.get_PI(), rocket.get_VI(), rocket.get_mass());
            rocket.set_attitude(attitude);
        }
    }
    else if (step_counter % 10 == 0 && guidance.get_time_to_go() <= IGM_stop_time)
    {
        if(guidance.get_time_to_go() > guidance_cycle)
        {
            guidance.set_time_to_go();
        }
        else if (guidance.get_time_to_go() > step)
        {
            guidance.set_time_to_go(guidance.get_time_to_go() - step);
        }
        else
        {
            guidance.set_time_to_go(step);
        }
    }
    vector<double> k1 = calculate_derivatives(t, rocket, gravity, atmosphere, engine, guidance, aero, states);
    rocket.update_states(k1, dt_half);

    vector<double> k2 = calculate_derivatives(t + dt_half, rocket, gravity, atmosphere, engine, guidance, aero, states);
    rocket.update_states(k2, dt_half);

    vector<double> k3 = calculate_derivatives(t + dt_half, rocket, gravity, atmosphere, engine, guidance, aero, states);
    rocket.update_states(k3, step);

    vector<double> k4 = calculate_derivatives(t + step, rocket, gravity, atmosphere, engine, guidance, aero, states);
    rocket.update_fstates(k1,k2,k3,k4,t,step);
    
}

vector<vector<double>> rotation_matrix_inertial_to_launch_simplified(double t)
{
    vector<vector<double>> M_result = {{1, omega_e * omega_e0[2] * t, -omega_e0[1] * omega_e * t}
                                        ,{-omega_e0[2] * omega_e * t, 1 , omega_e0[0] * omega_e * t}
                                        ,{omega_e * omega_e0[1] * t, -omega_e * omega_e0[0] * t, 1}};
    return M_result;
}
