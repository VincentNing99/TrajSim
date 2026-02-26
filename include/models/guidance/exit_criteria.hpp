#pragma once
// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Vincent Ning

/// @file exit_criteria.hpp
/// @brief Shared expression-tree exit criteria for all guidance algorithms.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>
#include <vector>

namespace trajsim {

// Forward declare so ExitContext can reference ExitNode::Parameter
struct ExitNode;

/// @brief Evaluation context for exit expression trees.
///
/// Each algorithm computes the relevant parameters from its state and fills
/// in the computed/reference arrays. The expression tree evaluator indexes
/// into these arrays by Parameter enum value.
struct ExitContext {
    static constexpr size_t NUM_PARAMS = 5;
    double computed[NUM_PARAMS] = {};
    double reference[NUM_PARAMS] = {};

    void set(size_t paramIdx, double comp, double ref = 0.0) {
        computed[paramIdx] = comp;
        reference[paramIdx] = ref;
    }
};

/// @brief Expression tree node for composable exit conditions.
///
/// Leaf nodes compare a computed parameter against a reference value.
/// And/Or nodes combine children with boolean logic.
struct ExitNode {
    enum class Type : uint8_t { Leaf, And, Or };
    enum class Parameter : uint8_t {
        SemiMajorAxis, Eccentricity, Inclination,
        Range, Altitude
    };
    enum class Comparison : uint8_t { WithinTolerance, ExceedsBy };

    Type type;
    Parameter parameter{};    ///< Leaf only
    Comparison comparison{};  ///< Leaf only
    double value{};           ///< Tolerance or threshold (leaf only)
    std::vector<ExitNode> children;  ///< And/Or only

    static ExitNode leaf(Parameter p, Comparison c, double v) {
        ExitNode n;
        n.type = Type::Leaf;
        n.parameter = p;
        n.comparison = c;
        n.value = v;
        return n;
    }
    static ExitNode andOf(std::vector<ExitNode> ch) {
        ExitNode n;
        n.type = Type::And;
        n.children = std::move(ch);
        return n;
    }
    static ExitNode orOf(std::vector<ExitNode> ch) {
        ExitNode n;
        n.type = Type::Or;
        n.children = std::move(ch);
        return n;
    }

    [[nodiscard]] bool evaluate(const ExitContext& ctx) const {
        switch (type) {
        case Type::Leaf: {
            auto idx = static_cast<size_t>(parameter);
            double c = ctx.computed[idx];
            double r = ctx.reference[idx];
            if (comparison == Comparison::WithinTolerance)
                return std::fabs(c - r) <= value;
            else
                return (c - r) > value;
        }
        case Type::And:
            return std::all_of(children.begin(), children.end(),
                [&](const ExitNode& child) { return child.evaluate(ctx); });
        case Type::Or:
            return std::any_of(children.begin(), children.end(),
                [&](const ExitNode& child) { return child.evaluate(ctx); });
        }
        return false;
    }
};

/// @brief Exit criteria wrapping an optional expression tree root.
struct ExitCriteria {
    std::optional<ExitNode> root;  ///< nullopt = never exit
};

}  // namespace trajsim
