# COMP0249 — Question 1a: Factor Graph for GPS-Enabled Localization

A factor graph is an undirected bipartite graph which represents the factorisation of a joint probability distribution over a set of unknown variables. It represents a model of unknown variables and their dependencies with the graphs vertices being divided into 2 sets:
- Variable Nodes (Drawn as circles): Represent the unknowns that need to be estimated by the graph
- Factor Nodes (Drawn as squares): represent the information required to solve for the unknowns; measurements or prior information.

Edges in the graph connect variable nodes to the factor nodes which they depend on. Where no edge may connect 2 variable nodes or 2 factor nodes directly; they can only connect vertices from one set to the other set.

## Variable Nodes (Vertices)

The only variable node in this problem is the vehicle pose vertex; which is given by equation 1 of appendix A.2

$$x_k = \begin{pmatrix}x_k \\ y_k \\ \psi_k \end{pmatrix}\qquad \qquad (1)$$

In this equation $x_k$ and $y_k$ are the position coordinates in the world-fixed frame and $\psi_k$ is the bearing of the vehicle.

## Addition Operator

Addition operators define how variable nodes are updated during optimisation - taking a perturbation: $\delta = [\delta_x, \delta_y, \delta_{\psi}]^T$ to update the state estimate during optimisation:

$$x_k \oplus \delta = \begin{pmatrix} x_k + \delta_x \\ y_k + \delta_y \\ (\psi_k + \delta_\psi) \bmod 2\pi \end{pmatrix}$$

Position components $x_k$ and $y_k$ use standard addition operation and live in $\mathbb{R}^2$. The bearing lives in $S^1$ due to its different addition operator. The state space is therefore $\mathbb{R}^2 \times S^1$. Noting that the bearing addition operator is different because bearing is a value between 0 and $2\pi$ whereby after exceeding $2\pi$ it returns to $0$ rather than being continuous like the $x_k$ and $y_k$ operators.

## Factor Nodes (Edges)

### Prior Factor (Unary)

The prior factor is an anchor for the initial pose to prevent the graph being underconstrained.

| Property        | Description                                                                         |
| --------------- | ----------------------------------------------------------------------------------- |
| **Type**        | Unary                                                                               |
| **Connects to** | $\mathbf{x}_0$                                                                      |
| **Equation**    | $\mathbf{x}_0 \sim \mathcal{N}(\boldsymbol{\mu}_0, \boldsymbol{\Sigma}_0)$          |
| **Measurement** | Initial state estimate $\boldsymbol{\mu}_0$ with covariance $\boldsymbol{\Sigma}_0$ |
| **Residual**    | $\mathbf{e}_{\text{prior}} = \mathbf{x}_0 \ominus \boldsymbol{\mu}_0$               |

### Process Factor (Binary)

| Property        | Description                                                                                                    |
| --------------- | -------------------------------------------------------------------------------------------------------------- |
| **Type**        | Binary                                                                                                         |
| **Connects to** | $\mathbf{x}_k$ and $\mathbf{x}_{k+1}$                                                                          |
| **Equation**    | Eq. 3 & 4 (A.2): $\mathbf{x}_{k+1} = \mathbf{x}_k + \mathbf{M}(\psi_k)\,(\mathbf{u}_{k+1} + \mathbf{v}_{k+1})$ |
| **Measurement** | Odometry input $\mathbf{u}_{k+1} = [s_k,\; 0,\; \dot{\psi}_k]^\top$                                            |
| **Residual**    | $\mathbf{e}_{\text{proc}} = \mathbf{x}_{k+1} \ominus [\mathbf{x}_k + \mathbf{M}(\psi_k)\,\mathbf{u}_{k+1}]$    |
| **Noise**       | $\mathbf{v}_{k+1} \sim \mathcal{N}(\mathbf{0}, \mathbf{Q}_k)$, diagonal                                        |

This factor is non-linear due to M depending on $\psi_k$ (Eq. 4), requiring iterative linearisation.

### GPS Measurement Factor (Unary)

| Property        | Description                                                                         |
| --------------- | ----------------------------------------------------------------------------------- |
| **Type**        | Unary                                                                               |
| **Connects to** | $\mathbf{x}_{k+1}$                                                                  |
| **Equation**    | A.3: $\mathbf{z}_{k+1}^G = [x_{k+1},\; y_{k+1}]^\top + \mathbf{w}_{k+1}^G$        |
| **Measurement** | GPS position fix $\mathbf{z}_{k+1}^G \in \mathbb{R}^2$                              |
| **Residual**    | $\mathbf{e}_{\text{GPS}} = \mathbf{z}_{k+1}^G - [x_{k+1},\; y_{k+1}]^\top$         |
| **Noise**       | $\mathbf{w}_{k+1}^G \sim \mathcal{N}(\mathbf{0}, \mathbf{R}^G)$, diagonal, constant |

This factor is linear — it directly observes position, not heading.

## Factor Graph Diagram

![Factor Graph Diagram](factor_graph_q1a.png)

**Legend:**
- **(X_k)** — Variable nodes (circles): vehicle pose at timestep $k$
- **[P]** — Prior factor (square): unary prior on initial pose $\mathbf{x}_0$
- **[F_k]** — Process factors (squares): binary odometry factors encoding Eq. 3 & 4 (A.2)
- **[G_k]** — GPS factors (squares): unary GPS measurement factors (A.3)

The process factors form a Markov chain. The GPS factors provide position corrections. Together they are complementary: without GPS the trajectory drifts; without process factors the heading is unobservable.
