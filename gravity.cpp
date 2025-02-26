//
//  gravity.cpp
//  FlightSim
//
//  Created by Vincent Ning on 8/7/24.
//

#include "gravity.hpp"
#include "math.hpp"
#include "trajectory.hpp"
using namespace std;
void gravity_model::initialize(vector<double> P, double A0, double B0, vector<double> coordinates)
{
    //发惯系x轴与北的夹角 in radians
    this->A0 = A0;
    //发惯系y轴与赤道平面夹角 in radians
    this->B0 = B0;
    //launch-site coordinates
    longitude = coordinates[0];
    latitude = coordinates[1];
    height = coordinates[2];
    //地球旋转角速度发惯系下
    omega_e0 = {cos(A0)*cos(B0), sin(B0), -cos(B0)*sin(A0)};
    
    e_b = sqrt(a * a - b * b) / b;
    e_a = sqrt(a * a - b * b) / a;
    phi = atan(1 / (1 + pow(e_b, 2)) * tan(latitude));
    R0 = a * b / sqrt(pow(a,2) * pow(sin(phi), 2) + pow(b,2) * pow(cos(phi), 2)) + height;
    R0_vec[0] = -R0 * sin(latitude - phi) * cos(A0);
    R0_vec[1] = R0 * cos(latitude - phi);
    R0_vec[2] = R0 * sin(A0) * sin(latitude - phi);


    R_vec = vector_add(R0_vec, P);
    R_mag = vector_mag(R_vec);

    R_norm.clear();
    R_norm.push_back(R_vec[0] / R_mag);
    R_norm.push_back(R_vec[1] / R_mag);
    R_norm.push_back(R_vec[2] / R_mag);
    sine_phi = R_norm[0] * omega_e0[0] + R_norm[1] * omega_e0[1] + R_norm[2] * omega_e0[2];
    

}
vector<double> gravity_model::calc_g(vector<double> P)
{
    vector<double> R_vec = vector_add(R0_vec, P);
    double R_mag = vector_mag(R_vec);
    vector<double> R_norm = {R_vec[0] / R_mag,R_vec[1] / R_mag, R_vec[2] / R_mag};
    double sine_phi = R_norm[0] * omega_e0[0] + R_norm[1] * omega_e0[1] + R_norm[2] * omega_e0[2];
    
    double g_r = -GM / (R_mag * R_mag)*(1 + J * a_e * a_e *(1 - 5.0 * sine_phi * sine_phi) / (R_mag * R_mag));
    
    double g_omega = -2.0 * J * GM * a_e * a_e * sine_phi / pow(R_mag,4);
    
    vector<double> vector_gr = scalar_vector(g_r, R_norm);
    
    vector<double> vertor_gomega = scalar_vector(g_omega, omega_e0);
    
    vector<double> gI = vector_add(vector_gr, vertor_gomega);
    
    double phi = asin(sine_phi);
    
    double latitude = atan(tan(phi)/(b * b / a / a));
    
    double height = R_mag - a_e / sqrt(1 + e_22 * sine_phi * sine_phi);
    
    return gI;
}

vector<double> gravity_model::get_g_terminal_guidance(vector<double> Pi, vector<double> Pi_terminal, vector<vector<double>> k)
{
    vector<double> g_curr = Matrix_multiply_vector(k, calc_g(Pi));
    vector<double> g_terminal = Matrix_multiply_vector(k, calc_g(Pi_terminal));
    vector<double> result = scalar_vector(0.5 , vector_add(g_curr,g_terminal));
    return result;
}



double gravity_model::get_R_mag(vector<double> position)
{
    vector<double> R_vec = vector_add(R0_vec, position);
    double R_mag = vector_mag(R_vec);
    return R_mag;
}

double gravity_model::get_altitude(vector<double> position)
{
    
    vector<double> R_vec = vector_add(R0_vec, position);
    double R_mag = vector_mag(R_vec);
    vector<double> R_norm = {R_vec[0] / R_mag,R_vec[1] / R_mag, R_vec[2] / R_mag};
    double sine_phi = R_norm[0] * omega_e0[0] + R_norm[1] * omega_e0[1] + R_norm[2] * omega_e0[2];
    
    double height = R_mag - a_e / sqrt(1 + e_22 * sine_phi * sine_phi);
    return height;
}
