# Deliverable 2: Working Implementation
## EXAMPLE 2: Arithmetic State Transitions

**Implementation Domain**: Quantitative & Continuous Coordinate State Space  
**Directory**: `EXAMPLE 2 - Arithmetic State Transitions/`

---

## 1. System Architecture and File Structure

The Arithmetic State Transition Planner is structured into modular, decoupled C++17 components:

```
EXAMPLE 2 - Arithmetic State Transitions/
├── include/
│   ├── types.hpp              # Coordinate State, Target Goal, Preconditions, Delta Effects, Capability
│   ├── embedding.hpp          # 5 Standard vector embedding methods & vector displacement algebra
│   ├── planner.hpp            # A* transition planner with Euclidean/Manhattan heuristic & hazard avoidance
│   ├── engine.hpp             # JSON loading, execution simulation, and transition_execution.txt exporter
│   └── json.hpp               # Embedded single-header JSON parser
├── src/
│   └── main.cpp               # CLI entry point, argument parsing, vector displays, transition execution
├── arithmetic_xy_reach_target.json   # Benchmark 1: Reaches (7, 3) from (0, 5)
├── arithmetic_grid_navigator.json    # Benchmark 2: Waypoint navigation (0, 0) -> (15, 8)
├── arithmetic_inventory_balance.json # Benchmark 3: Stock balance (24, 6) -> (15, 15)
├── arithmetic_bounded_pipeline.json  # Benchmark 4: Bounded target (2, 10) -> (12, 2)
├── transition_execution.txt          # Step-by-step state evolution artifact
├── Makefile                          # Self-contained build script
├── README.md                         # Overview & architecture
├── USAGE.md                          # CLI user guide
├── DELIVERABLE_1_FORMAL_EMBEDDING_DESIGN.md
├── DELIVERABLE_2_IMPLEMENTATION.md
├── DELIVERABLE_3_EXPERIMENTAL_DATASET.md
└── DELIVERABLE_4_TECHNICAL_REPORT.md
```

### Dependencies
- **Language Standard**: C++17 (`-std=c++17`)
- **Compilers**: Clang++ (macOS / Linux) or G++ (Linux / Windows MinGW)
- **Runtime Dependencies**: Zero external dependencies. Uses standard C++ headers.

---

## 2. The 5 Core Standard Vector Embedding Methods

All core mathematical operations are implemented in [`include/embedding.hpp`](include/embedding.hpp):

### Method 1: `encode(const State& s)`
```cpp
std::vector<double> encode(const State& s);
```
- **Description**: Projects the coordinate state into continuous vector space $\phi_S(S) \in \mathbb{R}^{d_s}$.
- **Implementation**:
  ```cpp
  std::vector<double> v(domain_vars.size(), 0.0);
  for (size_t i = 0; i < domain_vars.size(); ++i) {
      v[i] = s.get_numeric(domain_vars[i]);
  }
  return v;
  ```

### Method 2: `encode(const Goal& g)`
```cpp
std::vector<double> encode(const Goal& g);
```
- **Description**: Projects target destination coordinates into continuous vector space $\phi_G(G) \in \mathbb{R}^{d_g}$.

### Method 3: `encode(const Capability& c)`
```cpp
std::vector<double> encode(const Capability& c);
```
- **Description**: Encodes an arithmetic capability as a displacement vector and operational metadata $\phi_C(C) \in \mathbb{R}^{2d_s + 3}$.
- **Subspace Layout**:
  - `[0, ds - 1]`: Displacement vector $\Delta \mathbf{s}(C)$ ($\Delta x, \Delta y, \dots$)
  - `[ds, 2*ds - 1]`: Numerical precondition boundary thresholds $\mathbf{b}_{\text{pre}}$
  - `[2*ds]`: $C_{\text{time}}$ (transition latency)
  - `[2*ds + 1]`: $C_{\text{resource}}$ (step cost)
  - `[2*ds + 2]`: $\operatorname{Rel}_i$ (numerical transition reliability)

### Method 4: `compose(const Capability& c1, const Capability& c2)`
```cpp
Capability compose(const Capability& c1, const Capability& c2);
```
- **Description**: Computes the composite capability $C_{12} = C_2 \circ C_1$.
- **Semantics**:
  - Net displacement is additive: $\Delta \mathbf{s}(C_{12}) = \Delta \mathbf{s}(C_1) + \Delta \mathbf{s}(C_2)$.
  - Preconditions enforce boundary bounds: $P(C_{12})$ requires conditions of $C_1$ and $C_2$ offset by $\Delta \mathbf{s}(C_1)$.
  - Latency and cost are summed: $C_{\text{time}}(C_{12}) = C_{\text{time}}(C_1) + C_{\text{time}}(C_2)$.
  - Reliability is multiplicative: $\operatorname{Rel}(C_{12}) = \operatorname{Rel}(C_1) \cdot \operatorname{Rel}(C_2)$.

### Method 5: `similarity(const std::vector<double>& v1, const std::vector<double>& v2)`
```cpp
double similarity(const std::vector<double>& v1, const std::vector<double>& v2);
```
- **Description**: Computes the cosine similarity between two displacement or coordinate vectors:
  $$\operatorname{Sim}(\mathbf{v}_1, \mathbf{v}_2) = \frac{\langle \mathbf{v}_1, \mathbf{v}_2 \rangle}{\|\mathbf{v}_1\|_2 \cdot \|\mathbf{v}_2\|_2 + \epsilon}$$

---

## 3. Transition Compatibility Operator

In coordinate space, compatibility between capability $C_1$ and $C_2$ evaluates whether applying $C_1$ leaves the state within the valid precondition domain of $C_2$:
```cpp
double compute_compatibility(const Capability& c1, const Capability& c2);
```
Measures the directional alignment and numerical boundary clearance between $\Delta \mathbf{s}(C_1)$ and the precondition gradient of $C_2$.

---

## 4. Arithmetic Transition Planner (`include/planner.hpp`)

The planner uses an $A^*$ algorithm tailored to continuous/discrete coordinate graphs:
- **Heuristic Function**: Euclidean distance to target coordinate:
  $$h(S, G) = \|\phi_G(G) - \phi_S(S)\|_2 = \sqrt{\sum_{k=1}^{d_s} (x^*_k - x_k)^2}$$
- **Admissibility**: Because each capability step has a known maximum coordinate displacement, $h(S, G) / \max_{C} \|\Delta \mathbf{s}(C)\|_2$ provides a strictly admissible lower bound on remaining steps.
- **Hazard Pruning**: States falling into forbidden bounding boxes or exceeding coordinate limits are rejected immediately.

---

## 5. Transition Engine & Artifact Exporter (`include/engine.hpp`)

When the plan $\pi = \langle C_1, C_2, \dots, C_n \rangle$ is executed:
1. Simulates state evolution step-by-step: $S_{t+1} = S_t + \Delta \mathbf{s}(C_{t+1})$.
2. Tracks running Euclidean distance to goal, accumulated latency, and cumulative reliability.
3. Automatically writes a full execution trace to [`transition_execution.txt`](transition_execution.txt).

---

## 6. Compilation & CLI Execution

### Build Instructions
```bash
# Clean and compile
make clean
make
```

### CLI Execution
```bash
# Basic run on 2D coordinate problem
./arithmetic_planner arithmetic_xy_reach_target.json

# Display vector displacements and state embeddings
./arithmetic_planner arithmetic_xy_reach_target.json --vectors

# Display transition composition demo
./arithmetic_planner arithmetic_xy_reach_target.json --compose

# Display compatibility matrix
./arithmetic_planner arithmetic_xy_reach_target.json --compatibility

# Run all 4 benchmark problems
./arithmetic_planner arithmetic_xy_reach_target.json
./arithmetic_planner arithmetic_grid_navigator.json
./arithmetic_planner arithmetic_inventory_balance.json
./arithmetic_planner arithmetic_bounded_pipeline.json
```
