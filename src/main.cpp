#include "include/types.hpp"
#include "include/embedding.hpp"
#include "include/planner.hpp"
#include "include/engine.hpp"
#include <iostream>
#include <iomanip>

using namespace arithmetic;

void printHeader() {
    std::cout << "======================================================================\n";
    std::cout << "  EXAMPLE 2: ARITHMETIC STATE TRANSITION PLANNER                     \n";
    std::cout << "  Formal Capability Transitions, Vector Embeddings, & Ordered Planning\n";
    std::cout << "======================================================================\n\n";
}

void printUsage(const char* prog) {
    std::cout << "Usage:\n";
    std::cout << "  " << prog << " <problem_spec.json> [options]\n\n";
    std::cout << "Options:\n";
    std::cout << "  --vectors         Display vector embeddings for states, goals, & capabilities\n";
    std::cout << "  --compatibility   Display precondition-effect compatibility matrix\n";
    std::cout << "  --compose         Demonstrate formal capability composition (C_2 o C_1)\n";
    std::cout << "  --help            Show this help message\n\n";
}

int main(int argc, char* argv[]) {
    printHeader();

    if (argc < 2) {
        printUsage(argv[0]);
        std::cout << "Defaulting to: arithmetic_xy_reach_target.json\n\n";
    }

    std::string jsonPath = (argc >= 2 && argv[1][0] != '-') ? argv[1] : "arithmetic_xy_reach_target.json";

    bool showVectors = false;
    bool showCompat = false;
    bool showCompose = false;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--vectors") showVectors = true;
        if (arg == "--compatibility") showCompat = true;
        if (arg == "--compose") showCompose = true;
        if (arg == "--help") {
            printUsage(argv[0]);
            return 0;
        }
    }

    try {
        std::cout << "Loading formal application specification: " << jsonPath << " ...\n";
        ApplicationProblem app = ArithmeticEngine::loadFromJson(jsonPath);

        std::cout << "  -> Problem Name:    " << app.problemName << "\n";
        std::cout << "  -> Description:     " << app.description << "\n";
        std::cout << "  -> Initial State:   (x = " << app.initialState.getNum("x") << ", y = " << app.initialState.getNum("y") << ")\n";
        std::cout << "  -> Target Goal:     ";
        for (size_t i = 0; i < app.goal.conditions.size(); ++i) {
            std::cout << app.goal.conditions[i].variable << " " << app.goal.conditions[i].op << " " 
                      << app.goal.conditions[i].expectedValue.asString()
                      << (i + 1 < app.goal.conditions.size() ? ", " : "");
        }
        std::cout << "\n";
        std::cout << "  -> Capabilities:    " << app.capabilities.size() << " available transitions in C\n";
        std::cout << "  -> Hazard Barriers: " << app.globalConstraints.size() << " constraints in K\n\n";

        // Initialize Vector Embedding Engine
        VectorEmbeddingEngine embedding;
        embedding.initialize(app);
        size_t d_s = embedding.stateDimension();
        size_t d_c = d_s * 2 + 3; // Displacement + Precondition bounds + QoS/Rel

        std::cout << "[EMBEDDING SPACE DEFINITION]\n";
        std::cout << "  State Space Dimension   d_s = " << d_s << " [variables: ";
        for (size_t i = 0; i < embedding.indexVar.size(); ++i) {
            std::cout << embedding.indexVar[i] << (i + 1 < embedding.indexVar.size() ? ", " : "");
        }
        std::cout << "]\n";
        std::cout << "  Goal Space Dimension    d_g = " << d_s << "\n";
        std::cout << "  Capability Dimension    d_c = " << d_c << " [Delta s in R^" << d_s << ", bounds in R^" << d_s << ", Q/Rel in R^3]\n\n";

        // Display Vector Embeddings
        if (showVectors) {
            std::cout << "----------------------------------------------------------------------\n";
            std::cout << "1. FORMAL VECTOR REPRESENTATIONS (phi_S, phi_G, phi_C)\n";
            std::cout << "----------------------------------------------------------------------\n";

            auto s0Vec = embedding.encodeState(app.initialState);
            std::cout << "Initial State Vector phi_S(S_I) in R^" << d_s << ":\n  [";
            for (size_t i = 0; i < s0Vec.size(); ++i) {
                std::cout << std::fixed << std::setprecision(1) << s0Vec[i] << (i + 1 < s0Vec.size() ? ", " : "");
            }
            std::cout << "]\n\n";

            auto gVec = embedding.encodeGoal(app.goal);
            std::cout << "Goal Target Vector phi_G(G) in R^" << d_s << ":\n  [";
            for (size_t i = 0; i < gVec.size(); ++i) {
                std::cout << std::fixed << std::setprecision(1) << gVec[i] << (i + 1 < gVec.size() ? ", " : "");
            }
            std::cout << "]\n\n";

            std::cout << "Capability Transition Vectors phi_C(C_i) in R^" << d_c << ":\n";
            for (const auto& c : app.capabilities) {
                auto cVec = embedding.encodeCapability(c);
                std::cout << "  " << std::left << std::setw(20) << c.id << ": [";
                for (size_t i = 0; i < cVec.size(); ++i) {
                    std::cout << std::fixed << std::setprecision(1) << cVec[i] << (i + 1 < cVec.size() ? ", " : "");
                }
                std::cout << "]\n";
            }
            std::cout << "\n";
        }

        // Display Compatibility Matrix
        if (showCompat) {
            std::cout << "----------------------------------------------------------------------\n";
            std::cout << "2. TRANSITION COMPATIBILITY MATRIX (Probe State S_I)\n";
            std::cout << "----------------------------------------------------------------------\n";
            size_t nShow = std::min(app.capabilities.size(), size_t(6));
            std::cout << std::left << std::setw(16) << "From \\ To";
            for (size_t j = 0; j < nShow; ++j) {
                std::cout << std::setw(12) << app.capabilities[j].id;
            }
            std::cout << "\n";
            for (size_t i = 0; i < nShow; ++i) {
                std::cout << std::left << std::setw(16) << app.capabilities[i].id;
                for (size_t j = 0; j < nShow; ++j) {
                    double comp = embedding.calculateCompatibility(app.capabilities[i], app.capabilities[j], app.initialState);
                    std::cout << std::fixed << std::setprecision(1) << std::setw(12) << comp;
                }
                std::cout << "\n";
            }
            std::cout << "\n";
        }

        // Display Capability Composition
        if (showCompose && app.capabilities.size() >= 2) {
            std::cout << "----------------------------------------------------------------------\n";
            std::cout << "3. CAPABILITY COMPOSITION ALGEBRA (Section 5)\n";
            std::cout << "   C_12 = C_2 o C_1 => Delta s(C_12) = Delta s(C_1) + Delta s(C_2)\n";
            std::cout << "----------------------------------------------------------------------\n";
            const auto& c1 = app.capabilities[0]; // e.g. add1_x
            const auto& c2 = app.capabilities[1]; // e.g. add2_x
            Capability c12 = embedding.composeCapabilities(c1, c2);

            std::cout << "Component C_1: " << c1.id << " (" << c1.name << ")\n";
            std::cout << "Component C_2: " << c2.id << " (" << c2.name << ")\n";
            std::cout << "Composite C_12: " << c12.id << " [" << c12.name << "]\n";
            std::cout << "  -> Composed Effects:\n";
            for (const auto& e : c12.effects) {
                std::cout << "       " << e.variable << " " << e.op << " " << e.delta.asString() << "\n";
            }
            std::cout << "  -> Composed Time Cost: " << c12.qos.timeCost << " (additive)\n";
            std::cout << "  -> Composed Rel:       " << c12.reliability << " (multiplicative)\n\n";
        }

        // Run Planner to find the optimal order of capabilities
        std::cout << "----------------------------------------------------------------------\n";
        std::cout << "PLANNING: SYNTHESIZING CAPABILITY SEQUENCE TO REACH GOAL STATE\n";
        std::cout << "----------------------------------------------------------------------\n";
        StateTransitionPlanner planner(app, embedding);
        PlanResult plan = planner.findPlan();

        if (!plan.success) {
            std::cerr << "Error: No sequence of capabilities could reach the goal state.\n";
            return 1;
        }

        std::cout << "Plan Status:         SUCCESS [OPTIMAL TRAJECTORY FOUND]\n";
        std::cout << "Planning Latency:    " << plan.planningLatencyMicroseconds << " microseconds\n";
        std::cout << "Nodes Expanded:      " << plan.nodesExpanded << " states evaluated\n";
        std::cout << "Total Steps:         " << plan.steps.size() << " transitions\n";
        std::cout << "Accumulated Cost:    " << plan.totalCost << "\n\n";

        std::cout << "======================================================================\n";
        std::cout << "        ORDERED CAPABILITY SEQUENCE (CAPABILITIES AS TRANSITIONS)     \n";
        std::cout << "======================================================================\n";
        for (size_t i = 0; i < plan.steps.size(); ++i) {
            const auto& step = plan.steps[i];
            std::cout << "Step " << std::setw(2) << (i + 1) << ": "
                      << "State S_" << i << "(x=" << std::setw(2) << static_cast<int>(step.stateBefore.getNum("x"))
                      << ", y=" << std::setw(2) << static_cast<int>(step.stateBefore.getNum("y")) << ")"
                      << "  --[" << std::left << std::setw(14) << step.capability.id << "]--> "
                      << "State S_" << (i + 1) << "(x=" << std::setw(2) << static_cast<int>(step.stateAfter.getNum("x"))
                      << ", y=" << std::setw(2) << static_cast<int>(step.stateAfter.getNum("y")) << ")"
                      << "  (" << step.capability.name << ")\n";
        }
        std::cout << "======================================================================\n";
        std::cout << "FINAL STATE ACHIEVED: (x = " << plan.finalState.getNum("x") 
                  << ", y = " << plan.finalState.getNum("y") << ")  [GOAL SATISFIED!]\n";
        std::cout << "======================================================================\n\n";

        // Export Artifacts
        ArithmeticEngine::exportExecutionText("transition_execution.txt", app, plan);

        std::cout << "Artifacts Successfully Exported:\n";
        std::cout << "  - Execution Log:      transition_execution.txt\n\n";

    } catch (const std::exception& ex) {
        std::cerr << "Fatal Error: " << ex.what() << "\n";
        return 1;
    }

    return 0;
}
