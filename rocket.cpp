//
//  rocket.cpp
//  FlightSim
//
//  Created by Vincent Ning on 8/7/24.
//

#include "rocket.hpp"
#include "math.hpp"
#include "trajectory.hpp"
using namespace std;


void rkt::rocket::initialize()
{
    TK1 = 99999;
    TK2 = 99999;
    TK3 = 99999;
    // Initialize g-load array (used in engine inertia calculations)
    n[0] = 0.0;
    n[1] = 0.0;
    n[2] = 0.0;
}
void rkt::rocket::angle_calc(vector<double> cmd)
{
    if(cmd.size() < 3) {
        cerr << "Error: Invalid command vector size" << endl;
        return;
    }
    //attack/sideslip angle calc
    phi = cmd[0];
    psi = cmd[1];
    gamma = cmd[2];
    vector<vector<double>> MIb  = rotation_matrix_inertial_to_rocket(cmd);
    vector<double> RI = vector_add(R0, get_PI());
    vector<double> VE = Matrix_multiply_vector(scalar_matrix(omega_e, omegae0_cross), RI); // 转速乘以半径 = 速度
    
//    VW_zero = wind(hb / 1000.0, windtype); // 插值获得, assuming no wind for now
//    VW = {VW_zero * RVW * cos(AW + RAW*PI), 0.0, VW_zero * RVW * sin(AW + RAW*PI)};
    vector<double> VIE = subtract_vectors(get_VI(), VE);
    Vb = Matrix_multiply_vector(MIb , subtract_vectors(get_VI(), VE));
    double Vb_mag = vector_mag(Vb);
    double VE_mag = vector_mag(VE);
    double VIE_mag = vector_mag(VIE);
    if (Vb_mag < Epsilon0)
    {
        alpha  = 0.0;
        beta = 0.0;
    }
    else
    {
        alpha = atan2(Vb[1], Vb[0]);
        beta = asin(Vb[2] / Vb_mag);
    }
     if (VE_mag < Epsilon0)
     {
         gamma_f = 90.0 * D2R; // y/x, y指向重力方向，x指向射向
         sigma = 0.0; // 弹道偏角
     }
     else
     {
         gamma_f = atan2(VIE[1], VIE[0]);
         sigma = -asin(VIE[2] / VIE_mag);
     }
    // Matrix_MUL_vector(Vg, CIg, VIE);
    // norm_Vg = sqrt(vector_norm2(Vg));
    // Matrix_MUL_vector(tmpV, CIg, Ri); //地球固连坐标系位置
    // //相对于地表的位置 , not used?
    // xyz[0] = tmpV[0] - R_0[0];
    // xyz[1] = tmpV[1] - R_0[1];
    // xyz[2] = tmpV[2] - R_0[2];
}

double rkt::rocket::mass_calc()
{
    switch (f_states)
    {
        case flight_states::YJFX:
            mass_dot = -2920.3;
            break;
        case flight_states::EJFX:
            mass_dot = -309.079;
            break;
        case flight_states::SJFX:
            mass_dot = -0.973;
            break;
        default:
            break;
    }
    //
    
    return mass_dot;
//    rocket_inertia_com(rkt_mass);
}

void rkt::rocket::mass_properties() // 后续需要更新数据，以及按数据格式重新修改
{
    size_t left, right;
    vector<size_t> result;
    vector<vector<double>> table;
    switch(f_states)
    {
        case flight_states::YJFX:
        case flight_states::YJFL:
        case flight_states::YJGJ:
            table = Table1Jxyz;
            break;
        case flight_states::EJFX:
            table = Table2Jxyz;
            break;
        case flight_states::FS:
        case flight_states::EJGJ:
            table = Table2Jxyz_fs;
            break;
        case flight_states::SJFX:
        case flight_states::SJGJ:
            table = Table3Jxyz;
            break;
    }
    result = descending_binary_search(table, 1, rkt_mass);
    left = result[0];
    right = result[1];
    if (left == right) {
        COM = table[left][1];
        Jx = table[left][2];
        Jy  = table[left][3];
        Jz = table[left][4];
    }
    else{
        COM = table[left][1] + (table[left][1] - table[right][1]) / (table[left][0] - table[right][0]) * (rkt_mass - table[left][0]);
        Jx = table[left][2] + (table[left][2] - table[right][2]) / (table[left][0] - table[right][0]) * (rkt_mass - table[left][0]);
        Jy = table[left][3] + (table[left][3] - table[right][3]) / (table[left][0] - table[right][0]) * (rkt_mass - table[left][0]);
        Jz = table[left][4] + (table[left][4] - table[right][4]) / (table[left][0] - table[right][0]) * (rkt_mass - table[left][0]);
    }
}

vector<double> rkt::rocket::get_PI() 
{
    return {x,y,z};
}

vector<double> rkt::rocket::get_VI()
{
    vector<double> result = {x_dot,y_dot,z_dot};
    return result;
}

vector<double> rkt::rocket::get_accel()
{
    vector<double>result;
    result = {x_ddot,y_ddot,z_ddot};
    return result;
}

void rkt::rocket::update_states(vector<double> k, double step)
{

    x = x_n + k[0] * step;
    y = y_n + k[1] * step;
    z = z_n + k[2] * step;

    x_dot = x_dot_n + k[3] * step;
    y_dot = y_dot_n + k[4] * step;
    z_dot = z_dot_n + k[5] * step;

    rkt_mass = rkt_mass_n + k[6] * step;

    // Update position and velocity vectors
}

void rkt::rocket::update_fstates(vector<double> k1, vector<double> k2, vector<double> k3, vector<double> k4, double t, double dt)
{
    this->t = t + dt;
    
    // Calculate all new states from initial values (x_n, y_n, etc.)
    x = x_n + (k1[0] + 2*k2[0] + 2*k3[0] + k4[0]) * dt / 6;
    y = y_n + (k1[1] + 2*k2[1] + 2*k3[1] + k4[1]) * dt / 6;
    z = z_n + (k1[2] + 2*k2[2] + 2*k3[2] + k4[2]) * dt / 6;
    
    x_dot = x_dot_n + (k1[3] + 2*k2[3] + 2*k3[3] + k4[3]) * dt / 6;
    y_dot = y_dot_n + (k1[4] + 2*k2[4] + 2*k3[4] + k4[4]) * dt / 6;
    z_dot = z_dot_n + (k1[5] + 2*k2[5] + 2*k3[5] + k4[5]) * dt / 6;
    
    rkt_mass = rkt_mass_n + (k1[6] + 2*k2[6] + 2*k3[6] + k4[6]) * dt / 6;
    
    // Update _n values for next step
    x_n = x;
    y_n = y;
    z_n = z;
    
    x_dot_n = x_dot;
    y_dot_n = y_dot;
    z_dot_n = z_dot;
    
    x_ddot = (k1[3] + 2*k2[3] + 2*k3[3] + k4[3]) / 6;
    y_ddot = (k1[4] + 2*k2[4] + 2*k3[4] + k4[4]) / 6;
    z_ddot = (k1[5] + 2*k2[5] + 2*k3[5] + k4[5]) / 6;
    
    rkt_mass_n = rkt_mass;
    
}

void rkt::rocket::set_table(vector<vector<double>> table, string name)
{
    if(name == "Table1Jxyz")
    {
        Table1Jxyz = table;
    }
    else if(name == "Table2Jxyz")
    {
        Table2Jxyz = table;
    }
    else if(name == "Table2Jxyz_fs")
    {
        Table2Jxyz_fs = table;
    }
    else if(name == "Table3Jxyz")
    {
        Table3Jxyz = table;
    }
    else if(name == "Table4Jxyz")
    {
        Table4Jxyz = table;
    }
    else if(name == "Table5Jxyz")
    {
        Table4Jxyz = table;
    }
}

void rkt::rocket::set_states(vector<double> Pi, vector<double> Vi, vector<double> steering_angle,double t, double mass)
{
    x = Pi[0];
    y = Pi[1]; 
    z = Pi[2]; 
    x_n = Pi[0]; 
    y_n = Pi[1]; 
    z_n = Pi[2];
    x_dot = Vi[0];
    y_dot = Vi[1];
    z_dot = Vi[2];
    x_dot_n = Vi[0];
    y_dot_n = Vi[1];
    z_dot_n = Vi[2];
    rkt_mass = mass;
    rkt_mass_n = mass;
    phi = steering_angle[0];
    psi = steering_angle[1];
    gamma = steering_angle[2];
    this-> t = t;

}
//double wind(double h, windtype w_type, vector<vector<double>> table)
//{
//    vector<size_t> = result;
//    size_t left,right;
//    double vw;
//    switch(w_type)
//    {
//        case W_min:
//            if (h < 1.0)
//            {
//                result = ascending_bi_search(table, 1, 1.5);
//            }
//            else if (h > 63.0)
//            {
//                result = ascending_bi_search(table, 1, 63);
//            }
//            else
//            {
//                result = ascending_bi_search(table, 1, h);
//            }
//            break;
//            
//        case Wq:
//            if (h < 1.0)
//            {
//                result = ascending_binary_search(table, 1, 1.5);
//            }
//            else if (h > 29.0)
//            {
//                result = ascending_binary_search(table, 1, 28.5);
//            }
//            else
//            {
//                ascending_binary_search(table, 1, h);
//            }
//            break;
//            
//        case W_max:
//            if(h < 1.0)
//            {
//                ascending_binary_search(table, 1, 1.5);
//            }
//            else if (h > 63.0)
//            {
//                ascending_binary_search(table , 1, 63);
//            }
//            else
//            {
//                ascending_binary_search(table, 1, h);
//            }
//            break;
//    }
//    left = result[0];
//    right = result[1];
//    vw = p_wind_table[left][1] + (p_wind_table[left][1] - p_wind_table[right][1])*(h - p_wind_table[left][0]) / (p_wind_table[left][0] - p_wind_table[right][0]);
//}
