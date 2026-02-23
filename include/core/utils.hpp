// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Vincent Ning

/// @file utils.hpp
/// @brief useful functions for simulation
/// @author Vincent Ning
/// @date 2025
#pragma once
#include <fstream>
#include <numbers>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
namespace trajsim {

    /// @brief Reads a CSV file into a 2D vector of doubles.
    /// @param filepath Path to the CSV file.
    /// @return 2D vector where each inner vector is one row.
    /// @throws std::runtime_error If file cannot be opened.
    /// @throws std::invalid_argument If cell cannot be parsed as double.
    [[nodiscard]] inline std::vector<std::vector<double>> readCsv (const std::string& file)
    {
        std::ifstream data(file);
        if(!data.is_open())
        {
            throw std::runtime_error("Failed to open file: " + file);
        }

        std::vector<std::vector<double>> result;
        std::string line;

        while (std::getline(data, line))
        {
            std::vector<double> row;
            std::stringstream ss(line);
            std::string cell;

            while(std::getline(ss, cell, ','))
            {
                row.push_back(std::stod(cell));
            }

            result.push_back(row);
        }
        return result;
    }

    constexpr double radToDeg = 180.0 / std::numbers::pi;
    constexpr double degToRad = std::numbers::pi / 180.0;
}
