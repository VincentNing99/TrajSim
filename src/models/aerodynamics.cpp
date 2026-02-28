#include "models/vehicle/aerodynamics.hpp"
#include <set>
#include <sstream>
#include <algorithm>
using namespace trajsim;
Aerodynamics::Aerodynamics(const Config& config,
                           const std::vector<std::vector<double>>& highSpeedTable,
                           const std::vector<std::vector<double>>& lowSpeedTable)
    : cfg(config)
    , highSpeedTable(highSpeedTable)
    , lowSpeedTable(lowSpeedTable)
    , refArea{config.refArea}
    , refLength{config.refLength}
{
    cfg.validate();
    validateTable(highSpeedTable, "highSpeedTable");
    validateTable(lowSpeedTable, "lowSpeedTable");
    buildGrid(highSpeedTable, highSpeedGrid, "highSpeedTable");
    buildGrid(lowSpeedTable, lowSpeedGrid, "lowSpeedTable");
}

void Aerodynamics::validateTable(const std::vector<std::vector<double>>& table,
                                   const std::string& tableName) const
{
    if (table.empty()) {
        throw std::invalid_argument(tableName + " cannot be empty");
    }

    for (size_t i = 0; i < table.size(); ++i) {
        if (table[i].size() < AeroGrid::NUM_COLS) {
            std::ostringstream oss;
            oss << tableName << " row " << i << " has " << table[i].size()
                << " columns, expected " << AeroGrid::NUM_COLS;
            throw std::invalid_argument(oss.str());
        }

        // Check for NaN/Inf in table data
        for (size_t j = 0; j < AeroGrid::NUM_COLS; ++j) {
            if (std::isnan(table[i][j]) || std::isinf(table[i][j])) {
                std::ostringstream oss;
                oss << tableName << " contains NaN/Inf at row " << i << ", col " << j;
                throw std::invalid_argument(oss.str());
            }
        }
    }
}

void Aerodynamics::buildGrid(const std::vector<std::vector<double>>& table,
                               AeroGrid& grid,
                               const std::string& tableName) const
{
    std::set<double> machSet, betaSet, alphaSet;

    for (const auto& row : table) {
        machSet.insert(row[AeroGrid::COL_MACH]);
        betaSet.insert(row[AeroGrid::COL_BETA]);
        alphaSet.insert(row[AeroGrid::COL_ALPHA]);
    }

    grid.machVals.assign(machSet.begin(), machSet.end());
    grid.betaVals.assign(betaSet.begin(), betaSet.end());
    grid.alphaVals.assign(alphaSet.begin(), alphaSet.end());
    grid.computeStrides();

    // Verify complete grid
    size_t expectedSize = grid.machVals.size() * grid.betaVals.size() * grid.alphaVals.size();
    if (table.size() != expectedSize) {
        std::ostringstream oss;
        oss << tableName << " has " << table.size() << " rows, expected " << expectedSize
            << " (mach: " << grid.machVals.size()
            << " x beta: " << grid.betaVals.size()
            << " x alpha: " << grid.alphaVals.size() << ")";
        throw std::invalid_argument(oss.str());
    }

    // Verify at least 2 values per axis for interpolation
    if (grid.machVals.size() < 2 || grid.betaVals.size() < 2 || grid.alphaVals.size() < 2) {
        std::ostringstream oss;
        oss << tableName << " requires at least 2 values per axis for interpolation. "
            << "Got mach: " << grid.machVals.size()
            << ", beta: " << grid.betaVals.size()
            << ", alpha: " << grid.alphaVals.size();
        throw std::invalid_argument(oss.str());
    }
}

AeroCoefficients Aerodynamics::interpolate(const AeroGrid& grid,
                                            const std::vector<std::vector<double>>& table,
                                            double m, double beta, double alpha) const
{
    auto getBound = [](const std::vector<double>& axis, double val, size_t& idx, double& t) {
        auto it = std::lower_bound(axis.begin(), axis.end(), val);

        if (it == axis.begin()) {
            idx = 0;
            t = 0.0;
        }
        else if (it == axis.end()) {
            idx = axis.size() - 2;
            t = 1.0;
        }
        else {
            idx = static_cast<size_t>(it - axis.begin()) - 1;
            double x0 = axis[idx];
            double x1 = axis[idx + 1];
            t = (std::abs(x1 - x0) < 1e-10) ? 0.0 : (val - x0) / (x1 - x0);
        }
    };

    size_t i, j, k;
    double tm, tb, ta;
    getBound(grid.machVals, m, i, tm);
    getBound(grid.betaVals, beta, j, tb);
    getBound(grid.alphaVals, alpha, k, ta);

    size_t baseIdx = i * grid.strideMach + j * grid.strideBeta + k;
    size_t nextM = grid.strideMach;
    size_t nextB = grid.strideBeta;
    size_t nextA = 1;

    size_t maxIdx = baseIdx + nextM + nextB + nextA;
    if (maxIdx >= table.size()) {
        std::ostringstream oss;
        oss << "Interpolation index " << maxIdx << " out of bounds (table size: " << table.size() << ")";
        throw std::out_of_range(oss.str());
    }

    auto interp = [&](size_t col) -> double {
        double c000 = table[baseIdx][col];
        double c001 = table[baseIdx + nextA][col];
        double c010 = table[baseIdx + nextB][col];
        double c011 = table[baseIdx + nextB + nextA][col];
        double c100 = table[baseIdx + nextM][col];
        double c101 = table[baseIdx + nextM + nextA][col];
        double c110 = table[baseIdx + nextM + nextB][col];
        double c111 = table[baseIdx + nextM + nextB + nextA][col];

        double c00 = c000 * (1 - ta) + c001 * ta;
        double c01 = c010 * (1 - ta) + c011 * ta;
        double c10 = c100 * (1 - ta) + c101 * ta;
        double c11 = c110 * (1 - ta) + c111 * ta;

        double c0 = c00 * (1 - tb) + c01 * tb;
        double c1 = c10 * (1 - tb) + c11 * tb;

        return c0 * (1 - tm) + c1 * tm;
    };

    return AeroCoefficients{
        .forceCoef  = {interp(AeroGrid::COL_CX), interp(AeroGrid::COL_CY), interp(AeroGrid::COL_CZ)},
        .momentCoef = {interp(AeroGrid::COL_CMX), interp(AeroGrid::COL_CMY), interp(AeroGrid::COL_CMZ)}
    };
}

AeroResult Aerodynamics::compute(double mach, double alpha, double beta, double q, double stage) const
{
    // Validate inputs with descriptive errors
    if (std::isnan(mach)) throw std::invalid_argument("mach is NaN");
    if (std::isnan(alpha)) throw std::invalid_argument("alpha is NaN");
    if (std::isnan(beta)) throw std::invalid_argument("beta is NaN");
    if (std::isnan(q)) throw std::invalid_argument("dynamicPressure is NaN");

    if (q < 0) {
        throw std::invalid_argument("dynamicPressure must be non-negative, got " + std::to_string(q));
    }
    if (std::abs(alpha) > 90.0) {
        throw std::invalid_argument("alpha must be within ±90°, got " + std::to_string(alpha));
    }
    if (std::abs(beta) > 90.0) {
        throw std::invalid_argument("beta must be within ±90°, got " + std::to_string(beta));
    }

    double m = std::clamp(mach, cfg.machMin, cfg.machMax);

    const auto& table = (m < cfg.machTransition) ? lowSpeedTable : highSpeedTable;
    const auto& grid  = (m < cfg.machTransition) ? lowSpeedGrid : highSpeedGrid;

    AeroCoefficients coef = interpolate(grid, table, m, beta, alpha);

    double qS = q * getRefArea(stage);
    double qSL = qS * getRefLength(stage);

    return AeroResult{
        .force  = coef.forceCoef * qS,
        .moment = coef.momentCoef * qSL,
        .coef   = coef
    };
}

