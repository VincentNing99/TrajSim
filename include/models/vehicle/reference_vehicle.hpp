#pragma once

#include <stdexcept>

struct Config {
    // Engine Parameters
    double ISP_STAGE_1 = 267.618; // [s]
    double ISP_STAGE_2 = 329.776; // [s]
    double ISP_STAGE_3 = 315.0; // [s]

    double MASS_FLOW_STAGE_1 = 323.8790; // [kg/s]
    double MASS_FLOW_STAGE_2 = 316.945; // [kg/s]
    double MASS_FLOW_STAGE_3 = 0.973;  // [kg/s]

    //First stage engine nozzle exit area
    double S_EX_1 = 0.772882;
    //Second stage
    double S_EX_2 = 7.4506;
    //Third stage
    double S_EX_3 = 0.1637;

    // Engine thrust force
    double THRUST_1 = 850000;
    double THRUST_2 = 1025000;
    double THRUST_3 = 3000;

    // Number of Engines
    int N_FIRST = 9;
    int N_SECOND = 1;
    int N_THIRD = 1;
    //Engine exit pressure
    double P_EXIT = 101325;
};