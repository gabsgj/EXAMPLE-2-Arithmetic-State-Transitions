# Deliverable 1: Formal Embedding Design
## EXAMPLE 2: Arithmetic State Transitions

**Implementation Domain**: Quantitative & Continuous Coordinate State Space  
**Directory**: `EXAMPLE 2 - Arithmetic State Transitions/`

---

## 1. Fundamental Principle: Capabilities are Transitions ($\Delta \mathbf{s}$)

In quantitative coordinate spaces, the mathematical distinction between states and capabilities is rigorous:
- **State $\mathbf{s} \in \mathbb{R}^{d_s}$**: A static coordinate position in continuous state space (e.g., coordinates $(x, y, \dots)$).
- **Goal $\mathbf{g} \in \mathbb{R}^{d_g}$**: A target coordinate location $(x^*, y^*, \dots)$.
- **Capability $C_i$**: An **operation / transition arrow / displacement vector** $\Delta \mathbf{s}(C_i) \in \mathbb{R}^{d_s}$.

A capability is **never a state**. A capability is an operator that acts upon a state via vector addition:

$$\phi_S(S') = \phi_S(S) + \Delta \mathbf{s}(C_i)$$

Where $\Delta \mathbf{s}(C_i) = [\Delta x_1, \Delta x_2, \dots, \Delta x_{d_s}]^T$ represents the exact dimensional displacement effected by capability $C_i$.

---

## 2. Mathematical Application Model

The quantitative transition domain is formalized as a 6-tuple:

$$\mathcal{A} = (\mathcal{S}, \mathcal{C}, S_I, G, \mathcal{R}, \mathcal{K})$$

Where:
- $\mathcal{S} \subseteq \mathbb{R}^{d_s}$: Continuous or discrete coordinate space over $d_s$ state dimensions.
- $\mathcal{C}$: Set of atomic displacement capabilities (**transitions, not states**).
- $S_I \in \mathcal{S}$: Initial coordinate vector (e.g., $(x=0, y=5)$).
- $G \subset \mathcal{S}$: Target coordinate goal region (e.g., $(x=7, y=3)$).
- $\mathcal{R}$: Numerical capacity limits and step allowances.
- $\mathcal{K}$: Forbidden coordinate hazard regions (e.g., boundaries, obstacle coordinates, negative overflow zones).

---

## 3. State, Goal, and Capability Embeddings

### 3.1 State Representation $\phi_S(S) \in \mathbb{R}^{d_s}$
A point in the coordinate space:

$$\phi_S(S) = \begin{bmatrix} x \\ y \\ \text{step\_count} \end{bmatrix}$$

### 3.2 Goal Representation $\phi_G(G) \in \mathbb{R}^{d_s}$
The target destination in coordinate space:

$$\phi_G(G) = \begin{bmatrix} x^* \\ y^* \\ \dots \end{bmatrix}$$

### 3.3 11-Tuple Capability Model
Every arithmetic transition capability $C_i \in \mathcal{C}$ is formalized as:

$$C_i = (T_i, I_i, O_i, P_i, E_i, K_i, R_i, Q_i, \operatorname{Rel}_i, A_i, M_i)$$

1. **$T_i$**: $\code{ARITHMETIC\_TRANSITION}$
2. **$I_i$**: Input state coordinates $(x, y)$.
3. **$O_i$**: Output transformed coordinates $(x', y')$.
4. **$P_i$**: Numerical preconditions (e.g., $x \ge 1$, $y \le 20$, $x + y \le 30$).
5. **$E_i$**: Arithmetic displacement deltas $\Delta \mathbf{s} = (\Delta x, \Delta y)$.
6. **$K_i$**: Hazard avoidance guards ($x \notin [x_{\min}, x_{\max}]$).
7. **$R_i$**: Step budget consumption.
8. **$Q_i$**: Latency $C_{\text{time}}$ and computational cost $C_{\text{resource}}$.
9. **$\operatorname{Rel}_i \in [0, 1]$**: Numerical accuracy / transition reliability.
10. **$A_i \in \{0, 1\}$**: Operator availability.
11. **$M_i$**: Arithmetic execution kernel (`add_1_x`, `sub_2_y`, `mul_2_x`, etc.).

### 3.4 Capability Embedding Vector $\phi_C(C) \in \mathbb{R}^{2d_s + 3}$

$$\phi_C(C) = \begin{bmatrix} 
\Delta \mathbf{s}(C) \in \mathbb{R}^{d_s} \\ 
\mathbf{b}_{\text{pre}} \in \mathbb{R}^{d_s} \\ 
C_{\text{time}} \\ 
C_{\text{resource}} \\ 
\operatorname{Rel}_i 
\end{bmatrix}$$

Where $\mathbf{b}_{\text{pre}}$ represents the normalized numerical threshold boundaries required to fire the transition.

---

## 4. Vector Homomorphism of Capability Composition

Sequential capability composition $C_{12} = C_2 \circ C_1$ (applying $C_1$ followed by $C_2$) satisfies an exact **linear vector homomorphism**:

$$\Delta \mathbf{s}(C_2 \circ C_1) = \Delta \mathbf{s}(C_1) + \Delta \mathbf{s}(C_2)$$

### Concrete Example:
Consider two atomic arithmetic capabilities:
- $C_1 = \code{add\_2\_x}$ with displacement $\Delta \mathbf{s}(C_1) = \begin{bmatrix} +2.0 \\ 0.0 \end{bmatrix}$
- $C_2 = \code{sub\_1\_y}$ with displacement $\Delta \mathbf{s}(C_2) = \begin{bmatrix} 0.0 \\ -1.0 \end{bmatrix}$

Their composite capability $C_{12} = C_2 \circ C_1$ has the net displacement:
$$\Delta \mathbf{s}(C_{12}) = \begin{bmatrix} +2.0 \\ 0.0 \end{bmatrix} + \begin{bmatrix} 0.0 \\ -1.0 \end{bmatrix} = \begin{bmatrix} +2.0 \\ -1.0 \end{bmatrix}$$

Applying $C_{12}$ in a single step to initial state $S = (x=0, y=5)$:
$$\phi_S(S') = \begin{bmatrix} 0 \\ 5 \end{bmatrix} + \begin{bmatrix} 2 \\ -1 \end{bmatrix} = \begin{bmatrix} 2 \\ 4 \end{bmatrix}$$
This produces the exact same state as applying $C_1$ then $C_2$ sequentially:
$$S \xrightarrow{C_1} (2, 5) \xrightarrow{C_2} (2, 4)$$

Quality attributes aggregate homomorphically:
- $C_{\text{time}}(C_{12}) = C_{\text{time}}(C_1) + C_{\text{time}}(C_2)$
- $\operatorname{Rel}(C_{12}) = \operatorname{Rel}(C_1) \cdot \operatorname{Rel}(C_2)$

---

## 5. Goal-Relevance Directional Cosine Heuristic

During automated planning, the remaining distance from current coordinate $\phi_S(S)$ to goal coordinate $\phi_G(G)$ is represented by the error displacement vector:

$$\mathbf{e}(S, G) = \phi_G(G) - \phi_S(S)$$

Candidate capabilities are evaluated using the directional cosine alignment between the candidate's transition displacement $\Delta \mathbf{s}(C)$ and the target direction $\mathbf{e}(S, G)$:

$$\operatorname{Relevance}(C, S, G) = \cos \theta \left( \Delta \mathbf{s}(C), \; \mathbf{e}(S, G) \right) = \frac{\langle \Delta \mathbf{s}(C), \; \mathbf{e}(S, G) \rangle}{\|\Delta \mathbf{s}(C)\|_2 \cdot \|\mathbf{e}(S, G)\|_2 + \epsilon}$$

### Heuristic Properties:
- **$\cos \theta \approx +1.0$**: Capability points directly toward the target goal.
- **$\cos \theta \approx 0.0$**: Capability is orthogonal (advances one dimension while leaving target dimension unchanged).
- **$\cos \theta \approx -1.0$**: Capability moves away from the target goal (pruned or heavily penalized).

This vector formulation turns combinatorial discrete branch expansion into directed continuous gradient tracking.
