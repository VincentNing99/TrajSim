//
//  engine_model.cpp
//  FlightSim
//
//  Created by Vincent Ning on 8/7/24.
//

#include "engine_model.hpp"
#include "math.hpp"
using namespace std;
//engine exit pressure
//double P_e =;
vector<double> M_total(3,0), M1(3,0) , M2(3,0), M3(3,0), M4(3,0), M5(3,0), M6(3,0), M7(3,0), M8(3,0), M9(3,0), MC(3,0);
vector<double> arm(3,0);

double delta_1ab, delta_1abp,delta_2ab,delta_2abp,delta_3ab, delta_3abp,delta_4ab,delta_4abp,
delta_5ab, delta_5abp,delta_6ab,delta_6abp,delta_7ab, delta_7abp,delta_8ab,delta_8abp, delta_9ab, delta_9abp;
double delta_theta, delta_psi;


void engine_model::initialize()
{
    F1 = {0.0,0.0,0.0};
    F2 = {0.0,0.0,0.0};
    F3 = {0.0,0.0,0.0};
    F4 = {0.0,0.0,0.0};
    F5 = {0.0,0.0,0.0};
    F6 = {0.0,0.0,0.0};
    F7 = {0.0,0.0,0.0};
    F8 = {0.0,0.0,0.0};
    F9 = {0.0,0.0,0.0};
    FC = {0.0,0.0,0.0};
    MC = {0.0,0.0,0.0};
    // Initialize engine deflection angular accelerations
    delta_theta_ddot = 0.0;
    delta_psi_ddot = 0.0;
}
vector<vector<double>> engine_model::thrust(vector<double> servo_cmd, double t,double P_a, flight_states f_states, double COM)
{
    switch(f_states){
        case flight_states::YJFX:
            if (t <= 0.65) //TODO: add engine start transient behavior
            {
                P = 9.0*(P01 + S_a1*(P_e - P_a));
                
            }
            else if (t > 0.65)
            {
                P = 9.0*(P01 + S_a1*(P_e - P_a));
                
            }
            break;
        case flight_states::YJGJ:
            // TODO: add engine shutdown transient behavior
            P = 0.0;
            break;
        case flight_states::YJFL:
            P = 0.0; //TODO: add separation interference
            break;
        case flight_states::EJFX:
            P = P02;
            //delta_phi_psi = sqrt(delta_phi*delta_phi + delta_psi*delta_psi);
            //FC[0] = P*cos(delta_phi_psi);
            //FC[1] = P*sin(delta_phi);
            //FC[2] = -P*sin(delta_psi);
            //MC[1] = FC[2] * (delta_r2 - COM + delta_L);
            //MC[2] = -FC[1] * (delta_r2 - COM + delta_L);
            //TODO: add engine ignition transient behavior
            break;
        case flight_states::EJGJ:
            P = 0.0;
            break;
        case flight_states::SJFX:
            //滑行
            P = P03;
            break;
        default:
            break;
    }
    thrust_vector(f_states, servo_cmd, COM);
    result = {FC, MC};
    return result;
}

vector<vector<double>> engine_model::engine_inertia(flight_states f_states, double COM) {
    static vector<double> FI, MI;

    switch (f_states)
    {
    case flight_states::YJFX:
        FI[0] = 0.0;
        FI[1] = -9.0*Mr1*lr1*delta_theta_ddot;
        FI[2] = 9.0*Mr1*lr1*delta_psi_ddot; // 右手定则

        MI[0] = -8.0*(Jr1 * delta_theta_ddot + Mr1*lr1*zr*delta_theta_ddot);
        MI[1] = -9.0*(Jr1 * delta_theta_ddot + Mr1 * lr1 * (delta_r - COM + delta_L) * delta_theta_ddot + n[0] * g0 *Mr1*lr1*delta_theta);
        MI[2] = -9.0*(Jr1 * delta_psi_ddot + Mr1 * lr1 * (delta_r - COM + delta_L) * delta_psi_ddot + n[0] * g0*Mr1*lr1*delta_psi);
        break;
    case flight_states::EJFX:
    case flight_states::FS:
        FI[0] = 0.0;
        FI[1] = -Mr2*lr2*delta_theta_ddot;
        FI[2] = Mr2*lr2*delta_psi_ddot;

        MI[0] = 0.0;
        MI[1] = -(Jr2*delta_theta_ddot + Mr2 * lr2 * (delta_r2 - COM + delta_L)*delta_theta_ddot + n[0] * g0 * Mr2 * lr2 * delta_theta);
        MI[2] = -(Jr2*delta_psi_ddot + Mr2*lr2*(delta_r2 - COM + delta_L)*delta_psi_ddot + n[0] * g0*Mr2*lr2*delta_psi);
        break;
    case flight_states::SJFX:
        FI[0] = 0;
        FI[1] = 0;
        FI[2] = 0;

        MI[0] = 0;
        MI[1] = 0;
        MI[2] = 0;
        break;
    default:
        FI[0] = 0;
        FI[1] = 0;
        FI[2] = 0;

        MI[0] = 0;
        MI[1] = 0;
        MI[2] = 0;
        break;
    }
    result = {FI,MI};
    
    return result;
}

void engine_model::thrust_vector(flight_states f_states, vector<double> servo_cmd, double COM)
{
    
    switch (f_states)
    {
        case flight_states::YJFX:
        case flight_states::YJGJ:
        case flight_states::YJFL:
            delta_1ab = atan(sqrt(pow(tan(servo_cmd[0]), 2) + pow(tan(servo_cmd[1]), 2)));//发动机摆动角度
            delta_1abp = atan2(tan(servo_cmd[0]), tan(servo_cmd[1])); // 伺服夹角
            Fx_Dt1 = P / 9 * cos(delta_1ab);
            Fy_Dt1 = P / 9 * sin(delta_1ab)*(-sin(delta_1abp)*cos(71.5*D2R) - cos(delta_1abp)*sin(71.5*D2R));
            Fz_Dt1 = P / 9 * sin(delta_1ab)*(-sin(delta_1abp)*sin(71.5*D2R) + cos(delta_1abp)*cos(71.5*D2R));

            delta_2ab = atan(sqrt(pow(tan(servo_cmd[2]), 2) + pow(tan(servo_cmd[3]), 2)));
            delta_2abp = atan2(tan(servo_cmd[2]), tan(servo_cmd[3]));
            Fx_Dt2 = P / 9 * cos(delta_2ab);
            Fy_Dt2 = P / 9 * sin(delta_2ab)*( sin(delta_2abp)*cos(63.5*D2R) - cos(delta_2abp)*sin(63.5*D2R));
            Fz_Dt2 = P / 9 * sin(delta_2ab)*(-sin(delta_2abp)*sin(63.5*D2R) - cos(delta_2abp)*cos(63.5*D2R));

            delta_3ab = atan(sqrt(pow(tan(servo_cmd[4]), 2) + pow(tan(servo_cmd[5]), 2)));
            delta_3abp = atan2(tan(servo_cmd[4]), tan(servo_cmd[5]));
            Fx_Dt3 = P / 9 * cos(delta_3ab);
            Fy_Dt3 = P / 9 * sin(delta_3ab)*( sin(delta_3abp)*sin(71.5*D2R) - cos(delta_3abp)*cos(71.5*D2R));
            Fz_Dt3 = P / 9 * sin(delta_3ab)*(-sin(delta_3abp)*cos(71.5*D2R) - cos(delta_3abp)*sin(71.5*D2R));

            delta_4ab = atan(sqrt(pow(tan(servo_cmd[6]), 2) + pow(tan(servo_cmd[7]), 2)));
            delta_4abp = atan2(tan(servo_cmd[6]) , tan(servo_cmd[7]));
            Fx_Dt4 = P / 9 * cos(delta_4ab);
            Fy_Dt4 = P / 9 * sin(delta_4ab)*(sin(delta_4abp)*sin(63.5*D2R) + cos(delta_4abp)*cos(63.5*D2R));
            Fz_Dt4 = P / 9 * sin(delta_4ab)*(sin(delta_4abp)*cos(63.5*D2R) - cos(delta_4abp)*sin(63.5*D2R));

            delta_5ab = atan(sqrt(pow(tan(servo_cmd[8]), 2) + pow(tan(servo_cmd[9]), 2)));
            delta_5abp = atan2(tan(servo_cmd[8]) , tan(servo_cmd[9]));
            Fx_Dt5 = P / 9 * cos(delta_5ab);
            Fy_Dt5 = P / 9 * sin(delta_5ab)*(sin(delta_5abp)*cos(71.5*D2R) + cos(delta_5abp)*sin(71.5*D2R));
            Fz_Dt5 = P / 9 * sin(delta_5ab)*(sin(delta_5abp)*sin(71.5*D2R) - cos(delta_5abp)*cos(71.5*D2R));

            delta_6ab = atan(sqrt(pow(tan(servo_cmd[10]), 2) + pow(tan(servo_cmd[11]), 2)));
            delta_6abp = atan2(tan(servo_cmd[10]) , tan(servo_cmd[11]));
            Fx_Dt6 = P / 9 * cos(delta_6ab);
            Fy_Dt6 = P / 9 * sin(delta_6ab)*(-sin(delta_6abp)*cos(63.5*D2R) + cos(delta_6abp)*sin(63.5*D2R));
            Fz_Dt6 = P / 9 * sin(delta_6ab)*(sin(delta_6abp)*sin(63.5*D2R) + cos(delta_6abp)*cos(63.5*D2R));

            delta_7ab = atan(sqrt(pow(tan(servo_cmd[12]), 2) + pow(tan(servo_cmd[13]), 2)));
            delta_7abp = atan2(tan(servo_cmd[12]) , tan(servo_cmd[13]));
            Fx_Dt7 = P / 9 * cos(delta_7ab);
            Fy_Dt7 = P / 9 * sin(delta_7ab)*(-sin(delta_7abp)*sin(71.5*D2R) + cos(delta_7abp)*cos(71.5*D2R));
            Fz_Dt7 = P / 9 * sin(delta_7ab)*(sin(delta_7abp)*cos(71.5*D2R) + cos(delta_7abp)*sin(71.5*D2R));


            delta_8ab = atan(sqrt(pow(tan(servo_cmd[14]), 2) + pow(tan(servo_cmd[15]), 2)));
            delta_8abp = atan2(tan(servo_cmd[14]) , tan(servo_cmd[15]));
            Fx_Dt8 = P / 9 * cos(delta_8ab);
            Fy_Dt8 = P / 9 * sin(delta_8ab)*(-sin(delta_8abp)*sin(63.5*D2R) - cos(delta_8abp)*cos(63.5*D2R));
            Fz_Dt8 = P / 9 * sin(delta_8ab)*(-sin(delta_8abp)*cos(63.5*D2R) + cos(delta_8abp)*sin(63.5*D2R));

            delta_9ab = atan(sqrt(pow(tan(servo_cmd[16]), 2) + pow(tan(servo_cmd[17]), 2)));
            delta_9abp = atan2(tan(servo_cmd[16]) , tan(servo_cmd[17]));
            Fx_Dt9 = P / 9 * cos(delta_9ab);
            Fy_Dt9 = P / 9 * sin(delta_9ab)*(-sin(delta_9abp)*cos(47.0*D2R) + cos(delta_9abp)*sin(47.0*D2R));
            Fz_Dt9 = P / 9 * sin(delta_9ab)*(sin(delta_9abp)*sin(47.0*D2R) + cos(delta_9abp)*cos(47.0*D2R));

            FC[0] = Fx_Dt1 + Fx_Dt2 + Fx_Dt3 + Fx_Dt4 + Fx_Dt5 + Fx_Dt6 + Fx_Dt7 + Fx_Dt8 + Fx_Dt9;
            FC[1] = Fy_Dt1 + Fy_Dt2 + Fy_Dt3 + Fy_Dt4 + Fy_Dt5 + Fy_Dt6 + Fy_Dt7 + Fy_Dt8 + Fy_Dt9;
            FC[2] = Fz_Dt1 + Fz_Dt2 + Fz_Dt3 + Fz_Dt4 + Fz_Dt5 + Fz_Dt6 + Fz_Dt7 + Fz_Dt8 + Fz_Dt9;
            
            //发动机喷管作用点在质心坐标系的 位置坐标
            //#1机
            arm[0] = COM - delta_L - delta_r;
            arm[1] = -zr * cos(22.5*D2R);
            arm[2] = -zr * sin(22.5*D2R);
            F1 = {Fx_Dt1, Fy_Dt1, Fz_Dt1};
            M1 = cross_product33(arm, F1);
            //#2机
            arm[1] = -zr * sin(22.5*D2R);
            arm[2] = -zr * cos(22.5*D2R);
            F2 = {Fx_Dt2, Fy_Dt2, Fz_Dt2};
            M2 = cross_product33(arm, F2);
            //#3机
            arm[1] = zr*sin(22.5*D2R);
            arm[2] = -zr*cos(22.5*D2R);
            F3 = {Fx_Dt3, Fy_Dt3, Fz_Dt3};
            M3 = cross_product33(arm, F3);
            //#4机
            arm[1] = zr*cos(22.5*D2R);
            arm[2] = -zr*sin(22.5*D2R);
            F4 = {Fx_Dt4, Fy_Dt4, Fz_Dt4};
            M4 = cross_product33(arm, F4);
            //#5机

            arm[1] = zr*cos(22.5*D2R);
            arm[2] = zr*sin(22.5*D2R);
            F5 = {Fx_Dt5, Fy_Dt5, Fz_Dt5};
            M5 = cross_product33(arm, F5);
            //#6机
            arm[1] = zr*sin(22.5*D2R);
            arm[2] = zr*cos(22.5*D2R);
            F6 = {Fx_Dt6, Fy_Dt6, Fz_Dt6};
            M6 = cross_product33(arm, F6);
            //#7机
            arm[1] = -zr*sin(22.5*D2R);
            arm[2] = zr*cos(22.5*D2R);
            F7 = {Fx_Dt7, Fy_Dt7, Fz_Dt7};
            M7 = cross_product33(arm, F7);
            //#8机
            arm[1] = -zr*cos(22.5*D2R);
            arm[2] = zr*sin(22.5*D2R);
            F8 = {Fx_Dt8, Fy_Dt8, Fz_Dt8};
            M8 = cross_product33(arm, F8);
            //#9机
            arm[0] = COM - delta_L - delta_r9;
            arm[1] = 0;
            arm[2] = 0;
            F9 = {Fx_Dt9, Fy_Dt9, Fz_Dt9};
            M9 = cross_product33(arm, F9);
            
            M_comb = {M1, M2 ,M3, M4, M5, M6, M7, M8, M9};
            M_total = add_vectors(M_comb);
            MC = M_total;
            break;
        case flight_states::EJFX:
        case flight_states::FS:
        case flight_states::EJGJ:
            FC[0] = P02 * cos(delta_theta) * cos(delta_psi);
            FC[1] = P02 * cos(delta_psi) * sin(delta_theta);
            FC[2] = -P02 * sin(delta_psi);
            MC[0] = 0;
            MC[1] = FC[2] * (delta_r2 - COM + delta_L);
            MC[2] = FC[1] * (delta_r2 - COM + delta_L);
            break;
        case flight_states::SJFX:
        case flight_states::SJGJ:
            FC[0] = P03 * cos(delta_theta) * cos(delta_psi);
            FC[1] = P03 * cos(delta_psi) * sin(delta_theta);
            FC[2] = -P03 * sin(delta_psi);
            MC[0] = 0;
            MC[1] = FC[2] * (delta_r2 - COM + delta_L);
            MC[2] = FC[1] * (delta_r2 - COM + delta_L);
            break;
    }
}