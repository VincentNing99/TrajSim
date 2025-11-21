//
//  engine_model.hpp
//  FlightSim
//
//  Created by Vincent Ning on 8/7/24.
//

#ifndef engine_model_hpp
#define engine_model_hpp

#include "sim.hpp"
using flight_states = ::flight_states;
using windtype = ::windtype;

class engine_model {
    
public:
    engine_model() {initialize();};

    void initialize();
    
    std::vector<std::vector<double>> thrust(std::vector<double> servo_cmd, double flight_time, double P_a, flight_states f_states, double COM);
    
    std::vector<std::vector<double>> engine_inertia(flight_states f_states, double COM);
    
    inline std::vector<double> get_total_thrust(){return FC;};
    
    void thrust_vector(flight_states f_states, std::vector<double> servo_cmd, double COM);
private:
    
    //First stage engine nozzle exit area
    const double S_a1 = 0.772882;
    //Second stage engine nozzle exit area
    const double S_a2 = 7.4506;
    //Third stage engine nozzle exit area
    const double S_a3 = 0.1637;
    const double dm1 = 2920.3 / 9.0;
    const double dm2 = 309.079;
    const double dm3 = 0.973;
    //一级发动机推力
    const double P01 = 855000;
    //Second stage engine thrust force
    const double P02 = Vex2 * dm2;
    //Third stage engine thrust force
    const double P03 = Vex3 * dm3;
    //Rp thust diviation
    //理论顶点到实际顶点的距离
    const double delta_L = 412/1000;
    //一级发动机到实际尖点的距离1-8机
    const double delta_r = 70.119 - delta_L;
    //二级发动机到实际尖点的距离
    const double delta_r2 = 23.8255 - 0.412;
    //一级发动机到实际尖点的距离9机
    const double delta_r9 = 70.319 - delta_L;
    //一级发动机摆心到火箭纵轴距离0.9
    const double zr = 1.550;
    //二级推力
    const double F3d = 150;
    //抛罩前冷气推力,插值更为准确
    const double F_cg = 235;
    //rcs到中心的距离 second stage
    const double r2 = 1.6;
    //rcs到中心的距离三级
    const double r3 = 1.6;
    //一级rcs到质心的距离
    const double Lrcs1 = 12.169;
    //二级rcs到质心的距离
    const double Lrcs2 = 12.005;
    //一级单台发动机摆动部分的转动惯量
    const double Jr1 = 197.3;
    //一级发动机质心到摆轴的距离
    const double lr1 = 0.5902;
    //一级单台发动机摆动部分的质量
    const double Mr1 = 286;
    //二级单台发动机摆动部分的转动惯量
    const double Jr2 = 245;
    //二级发动机质心到摆轴的距离
    const double lr2 = 0.57;
    //二级单台发动机摆动部分的质量
    const double Mr2 = 330;
    //exit pressure of engine in Pascal
    double P_e = 101325;
    //pitch angular acceleration
    double delta_theta_ddot;
    //yaw angular accelaeration
    double delta_psi_ddot;
    //first stage engine force
    double Fx_Dt1, Fx_Dt2, Fx_Dt3, Fx_Dt4, Fx_Dt5, Fx_Dt6, Fx_Dt7, Fx_Dt8, Fx_Dt9;
    double Fy_Dt1, Fy_Dt2, Fy_Dt3, Fy_Dt4, Fy_Dt5, Fy_Dt6, Fy_Dt7, Fy_Dt8, Fy_Dt9;
    double Fz_Dt1, Fz_Dt2, Fz_Dt3, Fz_Dt4, Fz_Dt5, Fz_Dt6, Fz_Dt7, Fz_Dt8, Fz_Dt9;
    //total thrust
    double P;
    //G-load
    std::vector<double> n;
    std::vector<std::vector<double>> result;
    std::vector<double> F1, F2, F3, F4, F5, F6, F7, F8, F9, FC;
    std::vector<std::vector<double>> M_comb;
    
};
#endif /* engine_model_hpp */
