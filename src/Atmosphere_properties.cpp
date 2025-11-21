//
//  Atmosphere_properties.cpp
//  FlightSim
//
//  Created by Vincent Ning on 8/7/24.
//

#include "Atmosphere_properties.hpp"
#include "math.hpp"
using namespace std;

void atmosphere_model::initialize()
{
    T = 288;
    rho = 1.225;
    a_sound = 343;
    Vb_mag = 0.0;
    q_b = 0.0;
    Ma = 0.0;
    R_rho = 2.0 * rand() / RAND_MAX - 1.0;
    P_a = Pa0;
    H = h / (1 + h / r0);
}
void atmosphere_model::atmosphere_para(const vector<double> &Vb, double Z)
{
    H = Z / (1 + Z / r0) ;
    if (H < 11000.0)
    {
        H_B = 0.0;
        T_B = 288.15;
        H_l = -6.5e-3;
        P_B = 0.101325e6;
    }
    else if ((11000.0 <= H) && (H < 20000.0))
    {
        H_B = 11000.0;
        T_B = 216.65;
        H_l = 0.0;
        P_B = 0.226321e5;
    }
    else if ((20000.0 <= H) && (H < 32000.0))
    {
        H_B = 20000.0;
        T_B = 216.65;
        H_l = 0.001;
        P_B = 0.547488e4;
    }
    else if ((32000.0 <= H) && (H < 47000.0))
    {
        H_B = 32000.0;
        T_B = 228.65;
        H_l = 0.0028;
        P_B = 0.868018e3;
    }
    else if ((47000.0 <= H) && (H < 51000.0))
    {
        H_B = 47000.0;
        T_B = 270.65;
        H_l = 0.0;
        P_B = 0.110906e3;
    }
    else if ((51000.0 <= H) && (H < 71000.0))
    {
        H_B = 51000.0;
        T_B = 270.65;
        H_l = -0.0028;
        P_B = 0.669387e2;
    }
    else
    {
        H_B = 71000.0;
        T_B = 214.65;
        H_l = -0.0020;
        P_B = 0.395641e1;
    }

    if ((-Epsilon0 < H_l) && (H_l < Epsilon0))
    {
        // Isothermal layer
        if (fabs(R_gas * T_B) < Epsilon0) {
            P_a = P_B;  // Fallback if temperature is invalid
        } else {
            double exponent = -g0 * (H - H_B) / (R_gas * T_B);
            exponent = clamp(exponent, -100.0, 100.0);  // Prevent exp overflow
            P_a = P_B * exp(exponent);
        }
    }
    else
    {
        // Gradient layer
        if (P_B <= Epsilon0) {
            P_a = Epsilon0;  // Prevent log of zero/negative
        } else {
            double temp_factor = 1 + H_l * (H - H_B) / T_B;
            if (temp_factor <= Epsilon0) {
                temp_factor = Epsilon0;  // Prevent log of zero/negative
            }
            double exponent = log(P_B) - g0 * log(temp_factor) / (R_gas * H_l);
            exponent = clamp(exponent, -100.0, 100.0);  // Prevent exp overflow
            P_a = exp(exponent);
        }
    }
    
    T = T_B + H_l * (H - H_B);
    if (T <= Epsilon0) {
        T = Epsilon0;  // Prevent negative/zero temperature
    }
    rho = P_a / (R_gas*T);
    a_sound = sqrt(gamma * R_gas * T);
    if (a_sound < Epsilon0) {
        a_sound = Epsilon0;  // Prevent division by zero in Mach calculation
    }
    Vb_mag = vector_mag(Vb);
//    q_b = 0.5 *(1.0 + 0.13 * R_rho)*rho*Vb_mag*Vb_mag;
    q_b = 0.5 * rho * Vb_mag * Vb_mag;
    Ma = Vb_mag / a_sound;
}
