//
//  utilities.cpp
//  FlightSim
//
//  Created by Vincent Ning on 8/28/24.
//

#include "utilities.hpp"
using namespace std;
vector<vector<double>> read_table(string filename)
{
    // read data
    ifstream data(filename);
    if(!data.is_open()) {
        cerr << "file failed to open!" << endl;
        exit(1);
    }
    string line;
    vector<double> num;
    vector<vector<double>> result;
    while (getline(data, line))
    {
        stringstream ss(line);
        string cell;
        while(getline(ss, cell, ','))
        {
            
            num.push_back(stod(cell));
        }
        
        result.push_back(num);
        num.clear();
    }
    return result;
}

double interpolate_column(const vector<vector<double>>& table,
                         const vector<size_t>& range,
                         size_t col,
                         double t)
{
    // If indices are the same, no interpolation needed
    if (range[0] == range[1]) {
        return table[range[0]][col];
    }

    // Linear interpolation: v0 + (v1 - v0) / (t1 - t0) * (t - t0)
    double t0 = table[range[0]][0];
    double t1 = table[range[1]][0];
    double v0 = table[range[0]][col];
    double v1 = table[range[1]][col];

    return v0 + (v1 - v0) / (t1 - t0) * (t - t0);
}
