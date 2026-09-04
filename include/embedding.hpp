#pragma once

#include "types.hpp"
#include <cmath>
#include <numeric>
#include <iomanip>

namespace arithmetic {

// =============================================================================
// FORMAL VECTOR EMBEDDING ENGINE
// Section 6: Embedding Design Problem
// Deliverable 2: encode(state), encode(goal), encode(capability), compose(), similarity()
//
// GEOMETRIC VECTOR SPACE FORMULATION:
// - State Vector phi_S(S) = [x, y, step_count] in R^3
// - Goal Vector phi_G(G) = [x*, y*, ...] in R^3
// - Capability Vector phi_C(C) = [Delta x, Delta y, bounds, QoS] in R^9
//
// Vector Homomorphism of Capability Composition:
//   v_eff(C_2 o C_1) = v_eff(C_1) + v_eff(C_2)
// Goal Alignment:
//   Relevance(C, S, G) = cos(theta) between Delta s and (phi_G(G) - phi_S(S))
// =============================================================================

class VectorEmbeddingEngine {
public:
    std::unordered_map<std::string, size_t> varIndex;
    std::vector<std::string> indexVar;

    size_t registerVariable(const std::string& varName) {
        auto it = varIndex.find(varName);
        if (it != varIndex.end()) return it->second;
        size_t idx = indexVar.size();
        varIndex[varName] = idx;
        indexVar.push_back(varName);
        return idx;
    }

    [[nodiscard]] size_t stateDimension() const {
        return indexVar.size();
    }

    void initialize(const ApplicationProblem& app) {
        for (const auto& kv : app.initialState.vars) registerVariable(kv.first);
        for (const auto& cond : app.goal.conditions) registerVariable(cond.variable);
        for (const auto& cap : app.capabilities) {
            for (const auto& pre : cap.preconditions) registerVariable(pre.variable);
            for (const auto& eff : cap.effects) registerVariable(eff.variable);
        }
    }

    // -------------------------------------------------------------------------
    // Deliverable 2: encode(state) -> phi_S(S) in R^{d_s}
    // -------------------------------------------------------------------------
    [[nodiscard]] std::vector<double> encode(const State& s) const {
        std::vector<double> vec(stateDimension(), 0.0);
        for (size_t i = 0; i < stateDimension(); ++i) {
            vec[i] = s.getNum(indexVar[i], 0.0);
        }
        return vec;
    }

    [[nodiscard]] std::vector<double> encodeState(const State& s) const {
        return encode(s);
    }

    // -------------------------------------------------------------------------
    // Deliverable 2: encode(goal) -> phi_G(G) in R^{d_g}
    // -------------------------------------------------------------------------
    [[nodiscard]] std::vector<double> encode(const Goal& g) const {
        std::vector<double> vec(stateDimension(), 0.0);
        for (const auto& cond : g.conditions) {
            auto it = varIndex.find(cond.variable);
            if (it != varIndex.end()) {
                vec[it->second] = cond.expectedValue.toNumeric();
            }
        }
        return vec;
    }

    [[nodiscard]] std::vector<double> encodeGoal(const Goal& g) const {
        return encode(g);
    }

    // Transition Displacement Vector: Delta s in R^{d_s}
    [[nodiscard]] std::vector<double> encodeDisplacement(const Capability& c) const {
        std::vector<double> vec(stateDimension(), 0.0);
        for (const auto& eff : c.effects) {
            auto it = varIndex.find(eff.variable);
            if (it != varIndex.end()) {
                double val = eff.delta.toNumeric();
                if (eff.op == "SUB") val = -val;
                vec[it->second] = val;
            }
        }
        return vec;
    }

    // -------------------------------------------------------------------------
    // Deliverable 2: encode(capability) -> phi_C(C) in R^{d_c}
    // -------------------------------------------------------------------------
    [[nodiscard]] std::vector<double> encode(const Capability& c) const {
        std::vector<double> vec = encodeDisplacement(c);
        // Precondition bound indicators
        for (size_t i = 0; i < stateDimension(); ++i) {
            double bound = 0.0;
            for (const auto& pre : c.preconditions) {
                if (pre.variable == indexVar[i]) {
                    bound = pre.expectedValue.toNumeric();
                }
            }
            vec.push_back(bound);
        }
        // Operational metrics (Q_i, Rel_i)
        vec.push_back(c.qos.timeCost);
        vec.push_back(c.qos.resourceCost);
        vec.push_back(c.reliability);
        return vec;
    }

    [[nodiscard]] std::vector<double> encodeCapability(const Capability& c) const {
        return encode(c);
    }

    // -------------------------------------------------------------------------
    // Deliverable 2: similarity(x, y) -> Cosine similarity in R^d
    // -------------------------------------------------------------------------
    static double similarity(const std::vector<double>& v1, const std::vector<double>& v2) {
        if (v1.size() != v2.size() || v1.empty()) return 0.0;
        double dot = 0.0, n1 = 0.0, n2 = 0.0;
        for (size_t i = 0; i < v1.size(); ++i) {
            dot += v1[i] * v2[i];
            n1 += v1[i] * v1[i];
            n2 += v2[i] * v2[i];
        }
        if (n1 < 1e-9 || n2 < 1e-9) return 0.0;
        return dot / (std::sqrt(n1) * std::sqrt(n2));
    }

    static double cosineSimilarity(const std::vector<double>& v1, const std::vector<double>& v2) {
        return similarity(v1, v2);
    }

    // Precondition-Effect Compatibility
    [[nodiscard]] double calculateCompatibility(const Capability& c1, const Capability& c2, const State& probeState) const {
        State afterC1 = c1.apply(probeState);
        return c2.isApplicable(afterC1) ? 1.0 : 0.0;
    }

    // Goal Relevance: Cosine alignment between capability step and goal direction vector
    [[nodiscard]] double goalRelevance(const Capability& c, const State& s, const Goal& g) const {
        auto delta = encodeDisplacement(c);
        auto sVec = encode(s);
        auto gVec = encode(g);

        double dot = 0.0, nDelta = 0.0, nGoalDir = 0.0;
        for (size_t i = 0; i < stateDimension(); ++i) {
            double reqStep = gVec[i] - sVec[i];
            dot += delta[i] * reqStep;
            nDelta += delta[i] * delta[i];
            nGoalDir += reqStep * reqStep;
        }
        if (nDelta < 1e-9 || nGoalDir < 1e-9) return 0.0;
        return dot / (std::sqrt(nDelta) * std::sqrt(nGoalDir));
    }

    // -------------------------------------------------------------------------
    // Deliverable 2: compose(c1, c2) -> C_12 = C_2 o C_1 (Section 5)
    // -------------------------------------------------------------------------
    [[nodiscard]] Capability compose(const Capability& c1, const Capability& c2) const {
        Capability comp;
        comp.id = c1.id + "_THEN_" + c2.id;
        comp.name = "Composite: (" + c2.name + " o " + c1.name + ")";
        comp.type = "COMPOSITE_TRANSITION";
        comp.description = "Composed algebraic transition: executes " + c1.id + " followed by " + c2.id;

        // Composed effects: accumulate deltas for additive/subtractive transitions
        std::unordered_map<std::string, double> netDelta;
        for (const auto& e : c1.effects) {
            double d = e.delta.toNumeric();
            if (e.op == "SUB") d = -d;
            netDelta[e.variable] += d;
        }
        for (const auto& e : c2.effects) {
            double d = e.delta.toNumeric();
            if (e.op == "SUB") d = -d;
            netDelta[e.variable] += d;
        }

        for (const auto& [var, d] : netDelta) {
            Effect eff;
            eff.variable = var;
            if (d >= 0.0) {
                eff.op = "ADD";
                eff.delta = Value(d);
            } else {
                eff.op = "SUB";
                eff.delta = Value(-d);
            }
            comp.effects.push_back(eff);
        }

        // Composed preconditions
        comp.preconditions = c1.preconditions;
        for (const auto& p2 : c2.preconditions) {
            double shift = netDelta[p2.variable];
            Condition pAdj = p2;
            pAdj.expectedValue = Value(p2.expectedValue.toNumeric() - shift);
            comp.preconditions.push_back(pAdj);
        }

        // Operational attributes
        comp.qos.timeCost = c1.qos.timeCost + c2.qos.timeCost;
        comp.qos.resourceCost = std::max(c1.qos.resourceCost, c2.qos.resourceCost);
        comp.qos.moneyCost = c1.qos.moneyCost + c2.qos.moneyCost;
        comp.qos.risk = 1.0 - ((1.0 - c1.qos.risk) * (1.0 - c2.qos.risk));
        comp.reliability = c1.reliability * c2.reliability;

        return comp;
    }

    [[nodiscard]] Capability composeCapabilities(const Capability& c1, const Capability& c2) const {
        return compose(c1, c2);
    }
};

} // namespace arithmetic
