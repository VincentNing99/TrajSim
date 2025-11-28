//
//  sim.hpp
//  FlightSim
//
//  Created by Vincent Ning on 8/7/24.
//
#ifndef sim_hpp
#define sim_hpp

enum class flight_states {
    FIRST_STAGE_FLIGHT,      // First stage powered flight (was YJFX)
    FIRST_STAGE_CUTOFF,      // First stage engine cutoff (was YJGJ)
    FIRST_STAGE_SEPARATION,  // First stage separation (was YJFL)
    SECOND_STAGE_FLIGHT,     // Second stage powered flight (was EJFX)
    SECOND_STAGE_CUTOFF,     // Second stage engine cutoff (was EJGJ)
    THIRD_STAGE_FLIGHT,      // Third stage powered flight (was SJFX)
    THIRD_STAGE_CUTOFF,      // Third stage engine cutoff (was SJGJ)
    COASTING,                // Coasting phase (was HX)
    REENTRY                  // Reentry phase (was FS)
};
enum class windtype {W_min, Wq, W_max};
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <map>
#include <vector>
#include "constants.h"

#endif /* sim_hpp */
