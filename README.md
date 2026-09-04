# EXAMPLE 2: Arithmetic State Transition Planner

**Discrete Variable Transitions, Vector Embeddings, and Ordered Trajectory Planning**  
*Quantitative & Continuous Coordinate State Space*

---

## 1. Overview and Core Intuition

This example illustrates the foundational principles of vector embeddings, capability transitions, and composition in the simplest, most transparent mathematical domain possible: **discrete numerical coordinate states $(x, y)$**.

> **CRITICAL ARCHITECTURAL PRINCIPLE: CAPABILITIES ARE TRANSITIONS (NOT STATES)**
> - A **State** is a point / data snapshot in state space: $S = \{(x, v_x), (y, v_y)\} \in \mathcal{S}$.
> - A **Capability** is a **Transition / Operator / Directed Arrow** ($S \xrightarrow{C_i} S'$) that transforms state variables via effects ($E_i$) when its preconditions ($P_i$) are satisfied.
> - For example, `add_1_x` is **NOT** a state; it is a **transition** that transforms state $(x, y)$ into $(x+1, y)$.

The program reads initial coordinate states and target goal coordinates from a JSON file, computes vector representations, analyzes capability compatibility and composition, and executes goal-directed planning to synthesize the exact ordered sequence of capability transitions required to reach the goal.

---

## 2. Mathematical Formulation & Domain Mapping

### 2.1 Formal Application Model (Section 3)
$$\mathcal{A} = (\mathcal{S}, \mathcal{C}, S_I, G, \mathcal{R}, \mathcal{K})$$
- $\mathcal{S}$: State space $\mathbb{R}^2$ spanned by variables $\{x, y\}$.
- $\mathcal{C}$: Set of available arithmetic capability transitions:
  $$\mathcal{C} = \{ \code{add\_1\_x}, \code{add\_2\_x}, \code{sub\_1\_x}, \code{sub\_2\_x}, \code{add\_1\_y}, \code{add\_2\_y}, \code{sub\_1\_y}, \code{sub\_2\_y}, \code{mul\_2\_x}, \code{div\_2\_y} \}$$
- $S_I$: Initial coordinate state (e.g., $x=0, y=5$).
- $G$: Goal specification (e.g., $x=7, y=3$).
- $\mathcal{R}$: Computational resources (CPU ALU).
- $\mathcal{K}$: Domain bounds and safety barriers (e.g., $x \in [-20, 20]$).

### 2.2 Formal Capability Representation (Section 4)
Each arithmetic operation is modeled as a formal 11-tuple:
$$C_i = (T_i, I_i, O_i, P_i, E_i, K_i, R_i, Q_i, \operatorname{Rel}_i, A_i, M_i)$$
- $T_i = \code{COMPUTATION}$
- $P_i$: Preconditions on coordinates (e.g., $x \le 18$ for $\code{add\_2\_x}$)
- $E_i$: Effects / Transition displacement ($\Delta x = +2$)
- $Q_i$: Operational cost (e.g., latency, energy)
- $\operatorname{Rel}_i = 1.0$: Reliability
- $M_i = \code{CPU\_ALU}$: Execution mechanism

### 2.3 Vector Embedding (Section 6)
- $\phi_S(S) = [x, y, \dots]^T \in \mathbb{R}^{d_s}$
- $\phi_G(G) = [x^*, y^*, \dots]^T \in \mathbb{R}^{d_g}$
- $\phi_C(C_i) = [\Delta x, \Delta y, \text{bounds}, \text{QoS}]^T \in \mathbb{R}^{d_c}$

### 2.4 Capability Composition Algebra (Section 5)
When capability $C_1$ is followed by $C_2$:
$$C_{12} = C_2 \circ C_1$$
In vector arithmetic space:
$$\Delta \mathbf{s}(C_{12}) = \Delta \mathbf{s}(C_1) + \Delta \mathbf{s}(C_2)$$
For example:
$$\code{add\_1\_x} \circ \code{add\_2\_x} \Longrightarrow \Delta x = 1 + 2 = 3 \quad (\code{add\_3\_x})$$
Time costs are additive ($C_{\text{time}} = 1.0 + 1.2 = 2.2$), and reliabilities are multiplicative.

---

## 3. Modular Directory Structure

```
EXAMPLE 2 - Arithmetic State Transitions/
├── include/
│   ├── types.hpp            <- Formal State, Goal, and 11-tuple Capability definitions
│   ├── embedding.hpp        <- Vector embedding engine, spatial displacement, & composition
│   ├── planner.hpp          <- A* Goal-directed transition planner
│   └── engine.hpp           <- JSON problem loader & execution artifact exporter
├── src/
│   └── main.cpp             <- CLI interface, vector display, & plan output
├── third_party/
│   └── nlohmann/json.hpp    <- Vendored single-header JSON library
├── arithmetic_xy_reach_target.json    <- Reach target (x=7, y=3) from (x=0, y=5)
├── arithmetic_grid_navigator.json     <- 2D robot waypoint navigation: (0, 0) -> (15, 8)
├── arithmetic_inventory_balance.json  <- Dual depot stock balance: (24, 6) -> (15, 15)
├── arithmetic_bounded_pipeline.json   <- Bounded multi-step target reachability: (2, 10) -> (12, 2)
├── Makefile                           <- Self-contained C++17 build script
├── README.md                          <- This documentation
├── USAGE.md                           <- Step-by-step usage guide and commands
├── DELIVERABLE_1_FORMAL_EMBEDDING_DESIGN.md
├── DELIVERABLE_2_IMPLEMENTATION.md
├── DELIVERABLE_3_EXPERIMENTAL_DATASET.md
└── DELIVERABLE_4_TECHNICAL_REPORT.md
```

---

## 4. Quick Start

```bash
# Compile
make

# Run default target problem
make run

# Run with custom JSON
./arithmetic_planner arithmetic_bounded_pipeline.json --vectors --compose
```
