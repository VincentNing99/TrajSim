//
//  Atmosphere_properties.hpp
//  FlightSim
//
//  Created by Vincent Ning on 8/7/24.
//

#ifndef Atmosphere_properties_hpp
#define Atmosphere_properties_hpp

#include "sim.hpp"

class atmosphere_model {
    friend class rocket_model;
    friend class engine_model;
public:
    atmosphere_model(double height):h{height}{initialize();};
    void initialize();
    inline double get_air_pressure(){return P_a;};
    inline double get_density(){return rho;};
    inline double get_mach_number(){return Ma;};
    inline double get_q(){return q_b;};
    inline double get_a_local(){return a_sound;};
    inline double get_temp(){return T;};
    void atmosphere_para(const std::vector<double> &Vb , double Z);
private:
    //temperatrue lapse rate K/m
    const double L = -0.0065;
    // altitude m
    double h;
    // temperature at sea level in kelvin
    double T0 = 288.15;
    // molar mass of dry air kg/mol
    const double molar_M = 0.0289644;
    // universal gas constant J/(mol k)
    const double R = 8.31447;
    //地势高度
    double H;
    double H_B, T_B, H_l, P_B, P_a, T, R_rho, q_b, rho, a_sound, Ma, Vb_mag, K;
};
#endif /* Atmosphere_properties_hpp */
