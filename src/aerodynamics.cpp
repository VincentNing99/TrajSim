//
//  aerodynamics.cpp
//  FlightSim
//
//  Created by Vincent Ning on 8/18/24.
//

#include "aerodynamics.hpp"
#include "math.hpp"
using namespace std;
void aerodynamics::build_map()
{
    size_t left_m = 0;
    size_t left_a = 0;
    size_t right_m = 0;
    size_t right_a = 0;
    vector<size_t> M_bound = {left_m, right_m};
    vector<size_t> B_bound = {left_a, right_a};
    for(int i = 0; i < high_speed.size(); ++i)
    {
        double key_M = high_speed[i][1];

        auto it = M_map.find(key_M);
        
        if(it != M_map.end())
        {
            right_m++;
        }
        else
        {
            left_m = i;
            right_m = left_m;
        }
        M_bound = {left_m, right_m};
        M_map[key_M] = M_bound;
    }
    size_t upperb = M_map[high_speed[0][1]][1];
    for(size_t i = 0; i <= upperb; ++i)
    {
        double key_B = high_speed[i][2];
        auto it1 = B_map.find(key_B);
        if(it1 != B_map.end())
        {
            
            right_a++;
            
        }
        else
        {
            left_a = i;
            right_a = left_a;
        }
        B_bound = {left_a, right_a};
        B_map[key_B] = B_bound;
    }
    
    
    left_m = 0;
    left_a = 0;
    right_m = 0;
    right_a = 0;
    M_bound = {left_m, right_m};
    B_bound = {left_a, right_a};
    for(int i = 0; i < low_speed.size(); ++i)
    {
        double key_M = low_speed[i][1];

        auto it = M_map_low.find(key_M);
        
        if(it != M_map_low.end())
        {
            right_m++;
        }
        else
        {
            left_m = i;
            right_m = left_m;
        }
        M_bound = {left_m, right_m};
        M_map_low[key_M] = M_bound;
    }
    upperb = M_map_low[low_speed[0][1]][1];
    for(size_t i = 0; i <= upperb; ++i)
    {
        double key_B = low_speed[i][2];
        auto it1 = B_map_low.find(key_B);
        if(it1 != B_map_low.end())
        {
            
            right_a++;
            
        }
        else
        {
            left_a = i;
            right_a = left_a;
        }
        B_bound = {left_a, right_a};
        B_map_low[key_B] = B_bound;
    }
    
    
}

vector<vector<vector<size_t>>> aerodynamics::search_range(vector<vector<double>> table,  map<double,vector<size_t>> M_map, map<double,vector<size_t>> B_map)
{
    
    vector<vector<vector<size_t>>> result_final;
    vector<vector<size_t>> result;
    value.clear();
    auto it = M_map.lower_bound(M);
    if(it->first != M)
    {
        auto it_prev = prev(it);
        value.push_back(it_prev->first);
        result.push_back(M_map[it_prev->first]);
    }
    else
    {
        value.push_back(it->first);
        result.push_back(M_map[it->first]);
    }
    value.push_back(it->first);
    result.push_back(M_map[it->first]);
    result_final.push_back(result);
    result.clear();
    
    it = B_map.lower_bound(beta);
    if(it->first != beta)
    {
        auto it_prev = prev(it);
        value.push_back(it_prev->first);
        result.push_back(B_map[it_prev->first]);
    }
    else
    {
        value.push_back(it->first);
        result.push_back(B_map[it->first]);
    }
    result.push_back(B_map[it->first]);
    value.push_back(it->first);
    result_final.push_back(result);
    
    result.clear();
    
    it = B_map.begin();
    size_t right_b = it->second[1];
    result.push_back(ascending_binary_search_bounded(table, 4, alpha, 0, right_b));
    result_final.push_back(result);
    return result_final;
}

vector<double>aerodynamics::interp_3d(vector<vector<vector<size_t>>> range, vector<vector<double>> table)
{
    
    
    double C = 0;
    vector<vector<double>> result, result_beta;
    vector<double> result_1, final_result;
    size_t row_l,row_r;
    
    result.clear();
    result_beta.clear();
    result_1.clear();
    final_result.clear();
    
    for(int i = 0; i < range[0].size(); ++i)
    {
        for(int j = 0; j < range[1].size(); ++j)
        {
            row_l = range[0][i][0] + range[1][j][0] + range[2][0][0];
            row_r = range[0][i][0] + range[1][j][0] + range[2][0][1];
            for(int c = 4; c < 10; ++c)
            {
                if(row_l == row_r)
                {
                    C = table[row_l][c];
                }
                else
                {
                    double d_alpha = table[row_r][3] - table[row_l][3];
                    if (fabs(d_alpha) < Epsilon0) {
                        C = table[row_l][c];  // Use left value if alphas are identical
                    } else {
                        C = table[row_l][c] + (table[row_r][c] - table[row_l][c]) / d_alpha * (alpha - table[row_l][3]);
                    }
                }
                result_1.push_back(C);
            }
            result.push_back(result_1);
            result_1.clear();
        }
        for(int c = 0; c < result[0].size(); ++c)
        {
            double d_beta = value[3] - value[2];
            if(fabs(d_beta) < Epsilon0)
            {
                C = result[0][c];
            }
            else
            {
                C = result[0][c] + (result[1][c] - result[0][c]) / d_beta * (beta - value[2]);
            }
            result_1.push_back(C);
        }
        result_beta.push_back(result_1);
        result_1.clear();
        result.clear();
    }
    for(int c = 0; c < result_beta[0].size(); ++c)
    {
        double d_M = value[1] - value[0];
        if(fabs(d_M) < Epsilon0)
        {
            C = result_beta[0][c];
        }
        else
        {
            C = result_beta[0][c] + (result_beta[1][c] - result_beta[0][c]) / d_M * (M - value[0]);
        }
        result_1.push_back(C);
    }
    final_result = result_1;
    return final_result;
}

vector<double> aerodynamics::calc_coef(double M, double beta, double alpha)
{
    vector<double> result;
    if(M > 8.0)
    {
        this->M = 8.0;
    }
    else if(alpha > 90.0 || alpha < -90.0)
    {
        cerr << "attack angle out of range";
    }
    else if(beta > 90.0 || beta < -90.0)
    {
        cerr << "sideslip angle out of range";
    }
    if(M < 0.1)
    {
        this->M = 0.1;
    }
    if(M < 0.3)
    {
        result = interp_3d(search_range(low_speed, M_map_low, B_map_low), low_speed);
    }
    else
    {
        result = interp_3d(search_range(high_speed, M_map, B_map), high_speed);
    }
    return result;
}


vector<vector<double>> aerodynamics::calc_aerodynamics(double M, double beta, double alpha, double q)
{
    // Check for NaN or infinity in input parameters
    if (isnan(M) || isnan(beta) || isnan(alpha) || isnan(q) ||
        isinf(M) || isinf(beta) || isinf(alpha) || isinf(q)) {
        throw invalid_argument("Input parameters contain NaN or infinity");
    }

    // Check for valid ranges of input parameters
    if (M < 0 || q < 0) {
        throw invalid_argument("Mach number and dynamic pressure must be non-negative");
    }
    if (abs(beta) > 90.0 || abs(alpha) > 90.0) {
        throw invalid_argument("Beta and alpha must be within ±90 degrees");
    }

    this->alpha = alpha;
    this->beta = beta;
    this->M = M;
    vector<vector<double>> result;
    double Fx, Fy, Fz, Mx, My, Mz;
    vector<double> coef = calc_coef(M, beta, alpha);

    // Check if calc_coef returned valid coefficients
    if (coef.size() != 6) {
        throw runtime_error("Invalid number of coefficients returned from calc_coef");
    }

    // Calculate forces and moments
    Fx = coef[0] * q * EffectiveArea;
    Fy = coef[1] * q * EffectiveArea;
    Fz = coef[2] * q * EffectiveArea;
    Mx = coef[3] * q * EffectiveArea * L;
    My = coef[4] * q * EffectiveArea * L;
    Mz = coef[5] * q * EffectiveArea * L;

    // Check for NaN or infinity in results
    if (isnan(Fx) || isnan(Fy) || isnan(Fz) || 
        isnan(Mx) || isnan(My) || isnan(Mz) ||
        isinf(Fx) || isinf(Fy) || isinf(Fz) || 
        isinf(Mx) || isinf(My) || isinf(Mz)) {
        throw runtime_error("Results contain NaN or infinity");
    }

    Cx = coef[0];
    Cy = coef[1];
    Cz = coef[2];
    CMx = coef[3];
    CMy = coef[4];
    CMz = coef[5];
    result = {{Fx,Fy,Fz}, {Mx, My, Mz}};
    return result;
}
