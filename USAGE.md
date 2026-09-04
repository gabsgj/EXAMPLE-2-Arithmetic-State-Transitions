# Usage Guide: EXAMPLE 2 - Arithmetic State Transitions

This guide describes how to build, run, and experiment with the Arithmetic State Transition Planner.

---

## 1. Building the Program

Compile using the provided `Makefile`:

```bash
cd "EXAMPLE 2 - Arithmetic State Transitions"
make
```

To clean generated binaries and logs:
```bash
make clean
```

---

## 2. Command Line Interface

```text
Usage:
  ./arithmetic_planner <problem_spec.json> [options]

Options:
  --vectors         Display vector embeddings for states, goals, & capabilities
  --compatibility   Display precondition-effect compatibility matrix
  --compose         Demonstrate formal capability composition (C_2 o C_1)
  --help            Show this help message
```

---

## 3. Running Pre-Packaged Problems

### Problem 1: Reaching Target $(x=7, y=3)$ from $(x=0, y=5)$
```bash
./arithmetic_planner arithmetic_xy_reach_target.json --vectors --compatibility --compose
```

**Expected Trajectory Output:**
```text
Step 1: State S_0(x=0, y=5) --[add_2_x]--> State S_1(x=2, y=5)
Step 2: State S_1(x=2, y=5) --[add_2_x]--> State S_2(x=4, y=5)
Step 3: State S_2(x=4, y=5) --[sub_2_y]--> State S_3(x=4, y=3)
Step 4: State S_3(x=4, y=3) --[add_1_x]--> State S_4(x=5, y=3)
Step 5: State S_4(x=5, y=3) --[add_2_x]--> State S_5(x=7, y=3)
FINAL STATE ACHIEVED: (x = 7, y = 3) [GOAL SATISFIED!]
```

### Problem 2: Nonlinear Multiplier Pipeline
```bash
./arithmetic_planner arithmetic_bounded_pipeline.json
```

### Problem 3: 2D Grid Waypoint Navigation (Origin to $(15, 8)$)
```bash
./arithmetic_planner arithmetic_grid_navigator.json --vectors
```

### Problem 4: Multi-Depot Supply Balancing ($x=24, y=6 \to x=15, y=15$)
```bash
./arithmetic_planner arithmetic_inventory_balance.json --compose
```

---

## 4. Output Artifacts

1. `transition_execution.txt`: Step-by-step state evolution log ($S_0 \to S_1 \to \dots \to S_k$).

