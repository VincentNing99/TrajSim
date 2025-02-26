//
//  gravity.hpp
//  FlightSim
//
//  Created by Vincent Ning on 8/7/24.
//

#pragma once

#ifndef gravity_hpp
#define gravity_hpp

#include "sim.hpp"

class gravity_model {
    public:
    gravity_model(std::vector<double> coordinates, std::vector<double> initial_position, double A0, double B0) {
        initialize(initial_position, A0, B0, coordinates);
    };
    
    void initialize(std::vector<double> P, double, double, std::vector<double>);
    std::vector<double> calc_g(std::vector<double> P);
    double get_altitude(std::vector<double> Pi);
    inline std::vector<double> get_R_vec() const {return R_vec;};
    double get_R_mag(std::vector<double> Pi);

    std::vector<double> get_g_terminal_guidance(std::vector<double> Pi, std::vector<double> Pi_terminal, std::vector<std::vector<double>> C_et);

    private:
    double A0, B0;
    double e_b; //second eccentricity
    double e_a; // first eccentricity
    double phi; //地心纬度
    double R0;
    double longitude, latitude, height;


    std::vector<double> R_vec;
    double R_mag;
    std::vector<double> R0_vec{0.0, 0.0, 0.0};
    std::vector<double> omega_e0;
    std::vector<double> R_norm{0.0, 0.0, 0.0};

    double sine_phi; // sin(phi)
    std::vector<std::vector<double>> M_ECSF_to_terminal_guidance;
};

#endif /* gravity_hpp */
