#pragma once

#include <string>
#include <vector>
#include <unordered_map>
#include <variant>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <algorithm>
#include "third_party/nlohmann/json.hpp"

namespace arithmetic {

using json = nlohmann::json;

// =============================================================================
// FORMAL APPLICATION & CAPABILITY MODEL
// Section 3: Formal Application Model A = (S, C, S_I, G, R, K)
// Section 4: Formal Capability Model C_i = (T_i, I_i, O_i, P_i, E_i, K_i, R_i, Q_i, Rel_i, A_i, M_i)
//
// CRITICAL CONCEPT:
// CAPABILITIES ARE TRANSITIONS (NOT STATES)!
// - A State is a numerical snapshot: S = {(x, v_x), (y, v_y), ...}
// - A Capability is a Transition / Operation / Arrow: S -> S' that transforms state variables.
// =============================================================================

// Universal numerical/symbolic state variable value
struct Value {
    double numVal = 0.0;
    std::string strVal = "";
    bool isString = false;

    Value() = default;
    Value(double d) : numVal(d), isString(false) {}
    Value(int i) : numVal(static_cast<double>(i)), isString(false) {}
    Value(int64_t i) : numVal(static_cast<double>(i)), isString(false) {}
    Value(bool b) : numVal(b ? 1.0 : 0.0), isString(false) {}
    Value(const std::string& s) : strVal(s), isString(true) {}
    Value(const char* s) : strVal(s), isString(true) {}

    [[nodiscard]] double toNumeric() const { return numVal; }
    [[nodiscard]] std::string asString() const {
        if (isString) return strVal;
        std::ostringstream ss;
        if (std::floor(numVal) == numVal) ss << static_cast<int64_t>(numVal);
        else ss << std::fixed << std::setprecision(2) << numVal;
        return ss.str();
    }

    bool operator==(const Value& other) const {
        if (isString || other.isString) return asString() == other.asString();
        return std::abs(numVal - other.numVal) < 1e-6;
    }
};

// Application State: S = {(x_1, v_1), ..., (x_n, v_n)}
struct State {
    std::unordered_map<std::string, Value> vars;

    [[nodiscard]] bool has(const std::string& key) const {
        return vars.find(key) != vars.end();
    }

    [[nodiscard]] Value getOr(const std::string& key, const Value& def) const {
        auto it = vars.find(key);
        return (it != vars.end()) ? it->second : def;
    }

    void set(const std::string& key, const Value& val) {
        vars[key] = val;
    }

    [[nodiscard]] double getNum(const std::string& key, double def = 0.0) const {
        auto it = vars.find(key);
        return (it != vars.end()) ? it->second.toNumeric() : def;
    }
};

// Condition / Predicate on a state variable
struct Condition {
    std::string variable;
    std::string op; // "==", "!=", ">", "<", ">=", "<="
    Value expectedValue;

    [[nodiscard]] bool evaluate(const State& s) const {
        if (!s.has(variable)) return false;
        double cur = s.getNum(variable);
        double exp = expectedValue.toNumeric();
        if (op == "==") return std::abs(cur - exp) < 1e-6;
        if (op == "!=") return std::abs(cur - exp) >= 1e-6;
        if (op == ">")  return cur > exp;
        if (op == "<")  return cur < exp;
        if (op == ">=") return cur >= exp;
        if (op == "<=") return cur <= exp;
        return false;
    }
};

// Effect of a Capability Transition: S' = Apply(S, E_i)
struct Effect {
    std::string variable;
    std::string op; // "SET", "ADD", "SUB", "MUL", "DIV"
    Value delta;

    void apply(State& s) const {
        double cur = s.getNum(variable, 0.0);
        double d = delta.toNumeric();
        if (op == "SET") s.set(variable, Value(d));
        else if (op == "ADD") s.set(variable, Value(cur + d));
        else if (op == "SUB") s.set(variable, Value(cur - d));
        else if (op == "MUL") s.set(variable, Value(cur * d));
        else if (op == "DIV") {
            if (std::abs(d) > 1e-6) s.set(variable, Value(cur / d));
        }
    }
};

// Goal Specification: G = {g_1, g_2, ..., g_m}
struct Goal {
    std::vector<Condition> conditions;

    [[nodiscard]] bool isSatisfied(const State& s) const {
        for (const auto& cond : conditions) {
            if (!cond.evaluate(s)) return false;
        }
        return true;
    }
};

// Operational Quality Attributes Q_i = (C_time, C_resource, C_money, C_risk, C_energy)
struct QualityAttributes {
    double timeCost = 1.0;
    double moneyCost = 0.0;
    double resourceCost = 1.0;
    double risk = 0.0;

    [[nodiscard]] double totalCost() const {
        return timeCost + moneyCost + resourceCost + (risk * 5.0);
    }
};

// Formal Capability Representation (11-tuple)
// C_i = (T_i, I_i, O_i, P_i, E_i, K_i, R_i, Q_i, Rel_i, A_i, M_i)
// Note: In this domain, a capability is an ARITHMETIC STATE TRANSITION.
struct Capability {
    std::string id;
    std::string name;
    std::string description;

    // 11-Tuple Components:
    std::string type = "COMPUTATION";             // T_i
    std::vector<std::string> inputs;               // I_i
    std::vector<std::string> outputs;              // O_i
    std::vector<Condition> preconditions;          // P_i
    std::vector<Effect> effects;                   // E_i
    std::vector<Condition> constraints;            // K_i
    std::vector<std::string> resources;            // R_i
    QualityAttributes qos;                         // Q_i
    double reliability = 1.0;                      // Rel_i in [0, 1]
    double availability = 1.0;                     // A_i in {0, 1}
    std::string mechanism = "CPU_ALU";             // M_i

    [[nodiscard]] bool isApplicable(const State& s) const {
        for (const auto& pre : preconditions) {
            if (!pre.evaluate(s)) return false;
        }
        for (const auto& cons : constraints) {
            if (!cons.evaluate(s)) return false;
        }
        return true;
    }

    [[nodiscard]] State apply(const State& s) const {
        State nextState = s;
        for (const auto& eff : effects) {
            eff.apply(nextState);
        }
        // Increment step count
        double steps = nextState.getNum("step_count", 0.0);
        nextState.set("step_count", Value(steps + 1.0));
        return nextState;
    }
};

// Formal Application Model: A = (S, C, S_I, G, R, K)
struct ApplicationProblem {
    std::string problemName;
    std::string description;
    State initialState;                            // S_I
    Goal goal;                                     // G
    std::vector<Capability> capabilities;          // C
    std::vector<std::string> availableResources;   // R
    std::vector<Condition> globalConstraints;      // K (Hazards/Invariants)
};

} // namespace arithmetic
