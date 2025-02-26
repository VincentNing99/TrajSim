//
//  aerodynamics.hpp
//  FlightSim
//
//  Created by Vincent Ning on 8/18/24.
//

#ifndef aerodynamics_hpp
#define aerodynamics_hpp

#include "sim.hpp"
class aerodynamics
{
    public:
    aerodynamics(std::vector<std::vector<double>> high_speed_t, std::vector<std::vector<double>> low_speed_t, double EffectiveArea, double length):high_speed{high_speed_t},low_speed{low_speed_t}, EffectiveArea{EffectiveArea}, L{length}
    {
        build_map();
    };
    void build_map();
    std::vector<std::vector<double>> calc_aerodynamics(double M, double beta, double alpha, double q);
    std::vector<std::vector<std::vector<size_t>>> search_range(std::vector<std::vector<double>> table, std::map<double,std::vector<size_t>> M_map, std::map<double,std::vector<size_t>> B_map);
    std::vector<double>interp_3d(std::vector<std::vector<std::vector<size_t>>> range, std::vector<std::vector<double>> table);
    std::vector<double>calc_coef(double M, double beta, double alpha);
    inline std::vector<double> get_moment_coef(){std::vector<double> result; result = {CMx ,CMy, CMz}; return result;};
    inline std::vector<double> get_force_coef(){std::vector<double> result; result = {Cx ,Cy, Cz}; return result;};
    inline void set_high_speed_table(std::vector<std::vector<double>> table) {high_speed = table;};
    inline void set_low_speed_table(std::vector<std::vector<double>> table) {low_speed = table;};
    inline std::vector<std::vector<double>> get_high_speed_table() {return high_speed;};
    inline std::vector<std::vector<double>> get_low_speed_table() {return low_speed;};
    
    
    private:
    
    std::vector<double> value;
    std::map<double,std::vector<size_t>> M_map, B_map, M_map_low, B_map_low;
    double M;
    double alpha;
    double beta;
    double Cx, Cy, Cz, CMx, CMy, CMz;
    std::vector<std::vector<double>> low_speed, high_speed;
    double EffectiveArea;
    double L;
};
#endif /* aerodynamics_hpp */
