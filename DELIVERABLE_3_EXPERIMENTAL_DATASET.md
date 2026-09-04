# Deliverable 3: Experimental Datasets
## EXAMPLE 2: Arithmetic State Transitions

**Implementation Domain**: Quantitative & Continuous Coordinate State Space  
**Directory**: `EXAMPLE 2 - Arithmetic State Transitions/`

---

## 1. Dataset Overview

This implementation includes four quantitative coordinate benchmark datasets in standardized JSON format. Each problem defines numerical state coordinates, transition capabilities with displacement vectors $\Delta \mathbf{s}$, numerical precondition guards, and forbidden hazard zones.

| Dataset File | Problem Domain | Coordinate Dimensions | Initial State | Target Goal | Total Capabilities | Hazard Regions |
|:---|:---|:---:|:---:|:---:|:---:|:---:|
| [`arithmetic_xy_reach_target.json`](arithmetic_xy_reach_target.json) | 2D Point Reachability | $(x, y)$ | $(0, 5)$ | $(7, 3)$ | 8 | $x < 0, y < 0$ |
| [`arithmetic_grid_navigator.json`](arithmetic_grid_navigator.json) | 2D Robot Waypoint Navigation | $(x, y)$ | $(0, 0)$ | $(15, 8)$ | 6 | Obstacle Box |
| [`arithmetic_inventory_balance.json`](arithmetic_inventory_balance.json) | Dual-Depot Stock Balance | $(depot_A, depot_B)$ | $(24, 6)$ | $(15, 15)$ | 5 | Capacity $> 30$ |
| [`arithmetic_bounded_pipeline.json`](arithmetic_bounded_pipeline.json) | Bounded Dynamic Envelope | $(x, y)$ | $(2, 10)$ | $(12, 2)$ | 7 | Critical Zone |

---

## 2. Detailed Dataset Specifications

### 2.1 Benchmark 1: `arithmetic_xy_reach_target.json`
- **Domain**: Canonical 2D Cartesian plane target reachability.
- **Initial State**: $S_I = (x = 0, y = 5)$.
- **Goal State**: $G = (x^* = 7, y^* = 3)$.
- **Capabilities ($\Delta \mathbf{s}$ Transitions)**:
  - `add_2_x`: $\Delta \mathbf{s} = [+2, 0]^T$ ($C_{\text{time}} = 10\,\mu\text{s}, \operatorname{Rel} = 0.99$)
  - `add_1_x`: $\Delta \mathbf{s} = [+1, 0]^T$ ($C_{\text{time}} = 5\,\mu\text{s}, \operatorname{Rel} = 0.999$)
  - `sub_1_x`: $\Delta \mathbf{s} = [-1, 0]^T$ ($C_{\text{time}} = 5\,\mu\text{s}, \operatorname{Rel} = 0.999$)
  - `add_2_y`: $\Delta \mathbf{s} = [0, +2]^T$ ($C_{\text{time}} = 10\,\mu\text{s}, \operatorname{Rel} = 0.99$)
  - `sub_1_y`: $\Delta \mathbf{s} = [0, -1]^T$ ($C_{\text{time}} = 5\,\mu\text{s}, \operatorname{Rel} = 0.999$)
  - `sub_2_y`: $\Delta \mathbf{s} = [0, -2]^T$ ($C_{\text{time}} = 10\,\mu\text{s}, \operatorname{Rel} = 0.99$)
  - `diagonal_step`: $\Delta \mathbf{s} = [+1, -1]^T$ ($C_{\text{time}} = 12\,\mu\text{s}, \operatorname{Rel} = 0.98$)
- **Hazard Guard**: $x \ge 0, y \ge 0$ (non-negative quadrant invariant).

---

### 2.2 Benchmark 2: `arithmetic_grid_navigator.json`
- **Domain**: Robotic waypoint path planning with variable step strides.
- **Initial State**: $S_I = (x = 0, y = 0)$.
- **Goal State**: $G = (x^* = 15, y^* = 8)$.
- **Capabilities ($\Delta \mathbf{s}$ Transitions)**:
  - `stride_5_x`: $\Delta \mathbf{s} = [+5, 0]^T$ (High-speed horizontal transit)
  - `stride_3_x`: $\Delta \mathbf{s} = [+3, 0]^T$ (Medium horizontal transit)
  - `stride_1_x`: $\Delta \mathbf{s} = [+1, 0]^T$ (Fine positioning)
  - `stride_4_y`: $\Delta \mathbf{s} = [0, +4]^T$ (High-speed vertical transit)
  - `stride_2_y`: $\Delta \mathbf{s} = [0, +2]^T$ (Medium vertical transit)
  - `stride_1_y`: $\Delta \mathbf{s} = [0, +1]^T$ (Fine positioning)
- **Objective**: Reach $(15, 8)$ in minimum total steps without overshoot.

---

### 2.3 Benchmark 3: `arithmetic_inventory_balance.json`
- **Domain**: Dual-depot logistics equilibration.
- **Initial State**: $S_I = (\text{stock}_A = 24, \text{stock}_B = 6)$. Total inventory = 30.
- **Goal State**: $G = (\text{stock}_A^* = 15, \text{stock}_B^* = 15)$ (perfectly balanced).
- **Capabilities ($\Delta \mathbf{s}$ Transitions)**:
  - `transfer_5_A_to_B`: $\Delta \mathbf{s} = [-5, +5]^T$ (Bulk transfer; requires $\text{stock}_A \ge 5, \text{stock}_B \le 25$)
  - `transfer_3_A_to_B`: $\Delta \mathbf{s} = [-3, +3]^T$ (Medium transfer; requires $\text{stock}_A \ge 3, \text{stock}_B \le 27$)
  - `transfer_1_A_to_B`: $\Delta \mathbf{s} = [-1, +1]^T$ (Fine balancing; requires $\text{stock}_A \ge 1, \text{stock}_B \le 29$)
  - `transfer_2_B_to_A`: $\Delta \mathbf{s} = [+2, -2]^T$ (Reverse correction)
- **Constraint**: Conservation of total stock ($\text{stock}_A + \text{stock}_B = 30$) strictly maintained across all transitions.

---

### 2.4 Benchmark 4: `arithmetic_bounded_pipeline.json`
- **Domain**: Multi-stage pipeline with upper and lower boundary walls.
- **Initial State**: $S_I = (x = 2, y = 10)$.
- **Goal State**: $G = (x^* = 12, y^* = 2)$.
- **Capabilities ($\Delta \mathbf{s}$ Transitions)**:
  - `step_east_fast`: $\Delta \mathbf{s} = [+3, 0]^T$
  - `step_east_slow`: $\Delta \mathbf{s} = [+1, 0]^T$
  - `step_south_fast`: $\Delta \mathbf{s} = [0, -3]^T$
  - `step_south_slow`: $\Delta \mathbf{s} = [0, -1]^T$
  - `diagonal_advance`: $\Delta \mathbf{s} = [+2, -2]^T$
- **Hazard Guard**: Coordinate region $x \in [5, 7] \land y \in [5, 7]$ is a blocked hazard barrier that the planner must navigate around.

---

## 3. Dataset JSON Schema

```json
{
  "application": "ArithmeticStateTransitions",
  "domain": "Coordinate Vector Space",
  "description": "Problem overview",
  "state_variables": [
    {"name": "x", "type": "numeric", "description": "Horizontal coordinate"},
    {"name": "y", "type": "numeric", "description": "Vertical coordinate"}
  ],
  "initial_state": {
    "x": 0,
    "y": 5
  },
  "goal": {
    "x": 7,
    "y": 3
  },
  "capabilities": [
    {
      "id": "add_2_x",
      "type": "ARITHMETIC_TRANSITION",
      "name": "Add 2 to X",
      "preconditions": {
        "x": {"op": ">=", "val": 0}
      },
      "delta": {
        "x": 2.0,
        "y": 0.0
      },
      "qos": {
        "time": 10.0,
        "resource": 1.0,
        "reliability": 0.99
      }
    }
  ]
}
```
