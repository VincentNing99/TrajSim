//
//  constants.h
//  FlightSim
//
//  Created by Vincent Ning on 8/7/24.
//
#ifndef constants_h
#define constants_h

#include <vector>
#include <cmath>
#include "math.hpp"
//pi
const double PI = 3.141592653589793;
//rad to degrees
const double R2D = 180/PI;
//degrees to radians
const double D2R = PI/180;
//地球旋转角速度 rad/s
const double omega_e = 7.292115e-5;
//gravitational constant
const double GM = 3.986005e14L;  // 'L' for long double
//Mass of the Earth -- kg
const double M = 5.972e24;
//地球扁率
const double ae = 1/298.257223563;
//地球赤道半径
const double a_e = 6378140;
//地球长半轴
const double a = 6378140.0;
//地球短半轴
const double b = a * (1 - ae);
//二阶带谐系数
const double J2 = 1.08263e-3;
//四阶带谐系数
const double J4 = -2.37091e-6;
//J = 3/4 * J2
const double J = (3.0/2.0) * J2;
//rk4 step 1ms
const double step = 0.001;
//guidance cycle time
const double guidance_cycle = 0.01;
//minimum value
const double Epsilon0 = 1e-10;
//gravatational acceleration
const double g0 = 9.806650;
//radius
const double r0 = 6356752.0;
//gamma
const double gamma = 1.4;
//gas constant
const double R_gas = 287.05287;
//第一偏心率
const double e_2 = 0.00669437999013;
//第二偏心率
const double e_22 = 0.00673949674227;
//Sea level pressure
const double Pa0 = 101325.0;
//sea level temperature k
const double T0 = 288;
//tolerance
const double tolerance = 1e-10;
//vaccum engine parameters
const double Isp2 = 329.776;
//seal level engine parameters
const double Isp_SL = 271.490;

const double Vex2 = Isp2 * g0;
//Third stage engine
const double Isp3 = 315;
const double Vex3 = Isp3 * g0;
//initial mass
const double m_i1 = 581499.9;
const double m_i2 = 121175.5;
//rocket length
const double L = 70.582;
const double EffectiveArea = 11.341149;

const double m_fuel_2 = 30356.8;
const double m_ox_2 = 75281.6;
const double m_dot_2 = 309.079;

const double IGM_stop_time = 2;

#endif /* constants_h */
