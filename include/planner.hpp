#pragma once

#include "types.hpp"
#include "embedding.hpp"
#include <queue>
#include <chrono>
#include <set>

namespace arithmetic {

struct PlanStep {
    Capability capability;
    State stateBefore;
    State stateAfter;
};

struct PlanNode {
    State state;
    std::vector<std::string> path; // Sequence of capability IDs
    std::vector<State> stateHistory;
    double gCost = 0.0;
    double hCost = 0.0;

    [[nodiscard]] double fCost() const { return gCost + hCost; }

    bool operator>(const PlanNode& other) const {
        return fCost() > other.fCost();
    }
};

struct PlanResult {
    bool success = false;
    std::vector<PlanStep> steps;
    State finalState;
    double totalCost = 0.0;
    double planningLatencyMicroseconds = 0.0;
    size_t nodesExpanded = 0;
};

class StateTransitionPlanner {
private:
    const ApplicationProblem& app;
    const VectorEmbeddingEngine& embedding;

    // Vector-space heuristic distance || phi_S(s) - phi_G(g) ||_1
    [[nodiscard]] double heuristic(const State& s) const {
        auto sVec = embedding.encodeState(s);
        auto gVec = embedding.encodeGoal(app.goal);
        double dist = 0.0;
        for (const auto& cond : app.goal.conditions) {
            auto it = embedding.varIndex.find(cond.variable);
            if (it != embedding.varIndex.end()) {
                size_t idx = it->second;
                dist += std::abs(sVec[idx] - gVec[idx]);
            }
        }
        return dist;
    }

    [[nodiscard]] bool violatesHazards(const State& s) const {
        for (const auto& hazard : app.globalConstraints) {
            if (hazard.evaluate(s)) return true;
        }
        return false;
    }

    [[nodiscard]] std::string stateKey(const State& s) const {
        std::ostringstream ss;
        ss << "x=" << s.getNum("x") << ",y=" << s.getNum("y");
        return ss.str();
    }

public:
    StateTransitionPlanner(const ApplicationProblem& problem, const VectorEmbeddingEngine& emb)
        : app(problem), embedding(emb) {}

    PlanResult findPlan() {
        auto startTime = std::chrono::high_resolution_clock::now();
        PlanResult result;

        if (violatesHazards(app.initialState)) {
            return result;
        }

        if (app.goal.isSatisfied(app.initialState)) {
            result.success = true;
            result.finalState = app.initialState;
            return result;
        }

        std::priority_queue<PlanNode, std::vector<PlanNode>, std::greater<PlanNode>> openSet;
        openSet.push({app.initialState, {}, {app.initialState}, 0.0, heuristic(app.initialState)});

        std::set<std::string> visited;

        std::unordered_map<std::string, Capability> capMap;
        for (const auto& c : app.capabilities) {
            capMap[c.id] = c;
        }

        while (!openSet.empty()) {
            PlanNode cur = openSet.top();
            openSet.pop();
            result.nodesExpanded++;

            std::string sk = stateKey(cur.state);
            if (visited.find(sk) != visited.end()) continue;
            visited.insert(sk);

            if (app.goal.isSatisfied(cur.state)) {
                auto endTime = std::chrono::high_resolution_clock::now();
                result.success = true;
                result.finalState = cur.state;
                result.totalCost = cur.gCost;
                result.planningLatencyMicroseconds = std::chrono::duration<double, std::micro>(endTime - startTime).count();

                for (size_t i = 0; i < cur.path.size(); ++i) {
                    PlanStep step;
                    step.capability = capMap[cur.path[i]];
                    step.stateBefore = cur.stateHistory[i];
                    step.stateAfter = cur.stateHistory[i + 1];
                    result.steps.push_back(step);
                }
                return result;
            }

            for (const auto& cap : app.capabilities) {
                if (!cap.isApplicable(cur.state)) continue;

                State nextState = cap.apply(cur.state);
                if (violatesHazards(nextState)) continue;

                std::string nsk = stateKey(nextState);
                if (visited.find(nsk) != visited.end()) continue;

                PlanNode nextNode;
                nextNode.state = nextState;
                nextNode.path = cur.path;
                nextNode.path.push_back(cap.id);
                nextNode.stateHistory = cur.stateHistory;
                nextNode.stateHistory.push_back(nextState);
                nextNode.gCost = cur.gCost + cap.qos.totalCost();
                nextNode.hCost = heuristic(nextState);

                openSet.push(nextNode);
            }

            if (result.nodesExpanded > 20000) break;
        }

        auto endTime = std::chrono::high_resolution_clock::now();
        result.planningLatencyMicroseconds = std::chrono::duration<double, std::micro>(endTime - startTime).count();
        return result;
    }
};

} // namespace arithmetic
