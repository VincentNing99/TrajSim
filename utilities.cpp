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
