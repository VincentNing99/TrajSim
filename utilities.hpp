//
//  utilities.hpp
//  FlightSim
//
//  Created by Vincent Ning on 8/28/24.
//
#pragma once
#ifndef utilities_hpp
#define utilities_hpp

#include <stdio.h>
#include "sim.hpp"

extern std::vector<std::vector<double>> read_table(std::string filename);

// Linear interpolation utility for table data
extern double interpolate_column(const std::vector<std::vector<double>>& table,
                                 const std::vector<size_t>& range,
                                 size_t col,
                                 double t);

#endif /* utilities_hpp */

