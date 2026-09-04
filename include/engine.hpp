#pragma once

#include "types.hpp"
#include "embedding.hpp"
#include "planner.hpp"
#include <fstream>

namespace arithmetic {

class ArithmeticEngine {
public:
    static ApplicationProblem loadFromJson(const std::string& filePath) {
        std::ifstream f(filePath);
        if (!f.is_open()) {
            throw std::runtime_error("Cannot open problem specification: " + filePath);
        }
        json j;
        f >> j;

        ApplicationProblem app;
        app.problemName = j.value("problemName", "Arithmetic_State_Transition");
        app.description = j.value("description", "Arithmetic transitions between (x, y) coordinates");

        if (j.contains("initialState") && j["initialState"].is_object()) {
            for (auto& [k, v] : j["initialState"].items()) {
                if (v.is_number()) app.initialState.set(k, Value(v.get<double>()));
                else if (v.is_boolean()) app.initialState.set(k, Value(v.get<bool>()));
                else if (v.is_string()) app.initialState.set(k, Value(v.get<std::string>()));
            }
        }

        if (j.contains("goal") && j["goal"].is_array()) {
            for (const auto& gItem : j["goal"]) {
                Condition cond;
                cond.variable = gItem.value("variable", "");
                cond.op = gItem.value("op", "==");
                if (gItem.contains("expectedValue")) {
                    const auto& ev = gItem["expectedValue"];
                    if (ev.is_number()) cond.expectedValue = Value(ev.get<double>());
                    else if (ev.is_boolean()) cond.expectedValue = Value(ev.get<bool>());
                }
                app.goal.conditions.push_back(cond);
            }
        }

        if (j.contains("hazards") && j["hazards"].is_array()) {
            for (const auto& hItem : j["hazards"]) {
                Condition cond;
                cond.variable = hItem.value("variable", "");
                cond.op = hItem.value("op", "==");
                if (hItem.contains("expectedValue")) {
                    const auto& ev = hItem["expectedValue"];
                    if (ev.is_number()) cond.expectedValue = Value(ev.get<double>());
                    else if (ev.is_boolean()) cond.expectedValue = Value(ev.get<bool>());
                }
                app.globalConstraints.push_back(cond);
            }
        }

        if (j.contains("capabilities") && j["capabilities"].is_array()) {
            for (const auto& cItem : j["capabilities"]) {
                Capability cap;
                cap.id = cItem.value("id", "op_" + std::to_string(app.capabilities.size()));
                cap.name = cItem.value("name", cap.id);
                cap.description = cItem.value("description", "");
                cap.type = cItem.value("type", "COMPUTATION");
                cap.reliability = cItem.value("reliability", 1.0);
                cap.availability = cItem.value("availability", 1.0);

                if (cItem.contains("qos")) {
                    cap.qos.timeCost = cItem["qos"].value("timeCost", 1.0);
                    cap.qos.moneyCost = cItem["qos"].value("moneyCost", 0.0);
                    cap.qos.resourceCost = cItem["qos"].value("resourceCost", 1.0);
                    cap.qos.risk = cItem["qos"].value("risk", 0.0);
                }

                if (cItem.contains("preconditions") && cItem["preconditions"].is_array()) {
                    for (const auto& pItem : cItem["preconditions"]) {
                        Condition cond;
                        cond.variable = pItem.value("variable", "");
                        cond.op = pItem.value("op", "==");
                        if (pItem.contains("expectedValue")) {
                            cond.expectedValue = Value(pItem["expectedValue"].get<double>());
                        }
                        cap.preconditions.push_back(cond);
                    }
                }

                if (cItem.contains("effects") && cItem["effects"].is_array()) {
                    for (const auto& eItem : cItem["effects"]) {
                        Effect eff;
                        eff.variable = eItem.value("variable", "");
                        eff.op = eItem.value("op", "ADD");
                        if (eItem.contains("delta")) {
                            eff.delta = Value(eItem["delta"].get<double>());
                        } else if (eItem.contains("value")) {
                            eff.delta = Value(eItem["value"].get<double>());
                        }
                        cap.effects.push_back(eff);
                    }
                }

                app.capabilities.push_back(cap);
            }
        }

        return app;
    }

    static void exportExecutionText(const std::string& filePath, const ApplicationProblem& app, const PlanResult& plan) {
        std::ofstream out(filePath);
        if (!out.is_open()) return;

        out << "======================================================================\n";
        out << "      ARITHMETIC CAPABILITY TRANSITION EXECUTION SUMMARY             \n";
        out << "      Problem: " << app.problemName << "\n";
        out << "======================================================================\n\n";

        out << "Initial State S_I:\n";
        out << "  x = " << app.initialState.getNum("x") << ", y = " << app.initialState.getNum("y") << "\n\n";

        out << "Goal State G:\n";
        for (const auto& g : app.goal.conditions) {
            out << "  " << g.variable << " " << g.op << " " << g.expectedValue.asString() << "\n";
        }
        out << "\n";

        out << "Execution Status:  " << (plan.success ? "SUCCESS" : "FAILED") << "\n";
        out << "Total Steps:       " << plan.steps.size() << " transitions\n";
        out << "Total Cost:        " << plan.totalCost << "\n";
        out << "Planning Latency:  " << plan.planningLatencyMicroseconds << " microseconds\n\n";

        out << "Step-by-Step State Evolution (S_i -> C_k -> S_{i+1}):\n";
        out << "----------------------------------------------------------------------\n";
        for (size_t i = 0; i < plan.steps.size(); ++i) {
            const auto& step = plan.steps[i];
            out << "Step " << (i + 1) << ":\n";
            out << "  State Before:  (x=" << step.stateBefore.getNum("x") << ", y=" << step.stateBefore.getNum("y") << ")\n";
            out << "  Capability:    " << step.capability.id << " [" << step.capability.name << "]\n";
            out << "  State After:   (x=" << step.stateAfter.getNum("x") << ", y=" << step.stateAfter.getNum("y") << ")\n\n";
        }
        out << "======================================================================\n";
    }

    static void exportVisualizerManifest(const std::string& filePath, const ApplicationProblem& app, const PlanResult& plan) {
        json j;
        j["manifestVersion"] = "2.0";
        j["domain"] = "ArithmeticTransitions";
        j["problemName"] = app.problemName;
        j["success"] = plan.success;
        j["planningLatencyMicroseconds"] = plan.planningLatencyMicroseconds;

        // Path of coordinates visited
        json pathArr = json::array();
        for (size_t i = 0; i < plan.steps.size(); ++i) {
            json pt;
            pt["step"] = i + 1;
            pt["capability"] = plan.steps[i].capability.id;
            pt["x"] = plan.steps[i].stateAfter.getNum("x");
            pt["y"] = plan.steps[i].stateAfter.getNum("y");
            pathArr.push_back(pt);
        }
        j["trajectory"] = pathArr;

        std::ofstream out(filePath);
        if (out.is_open()) {
            out << j.dump(2) << "\n";
        }
    }
};

} // namespace arithmetic
