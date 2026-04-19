# Optimal Solver for Makespan-Optimal Coordinated Motion Planning for Unlabeled Robots in Graphs

This package provides a solver for makespan-optimal coordinated motion planning given unlabeled robots in arbitrary graphs.
The decision problem of whether a schedule with makespan M exists is polynomial-time solvable by applying a reduction to maximum s-t-flow with unit capacities, introduced by Yu and LaValle:
[Multi-agent Path Planning and Network Flow](https://doi.org/10.1007/978-3-642-36279-8_10), Algorithmic Foundations of Robotics, 2013.
A binary search over the possible makespan values will then yield the optimal solution.

## Problem Description
Consider a simple and connected graph $G = (V,E)$ with a set of start positions $S \subseteq V$, and target positions $T \subseteq V$.
We assume $|S| \leq |T|$.
The goal is to move all robots from their start positions to distinct target positions in as few timesteps as possible.
Robots are unlabeled, meaning any robot may occupy any target position.
At each timestep, every robot may either move to an adjacent vertex or stay in place.
No two robots may occupy the same vertex at the same time, and no two robots may swap positions in a single timestep.

## Installation
It should be easy to install the solver using the following command:

```bash
pip install --verbose git+https://github.com/KaiKobbe/unlabeled_cmp_solver
```

## Max-Flow Reduction by Yu and LaValle
Consider an instance of unlabeled coordinated motion planning on a graph $G = (V,E)$ with a set of start positions $S \subseteq V$, and target positions $T \subseteq V$.
The reduction solves the decision problem of whether a feasible schedule of makespan $M$ exists.

We construct a time-expanded network $G'$ as follows.
For each vertex $v \in V$, the vertex set of $G'$ contains $2M + 1$ copies of it, namely
$$v_0 = v_0', v_1, v_1', v_2, v_2', \dots, v_M, v_M'.$$
In addition, $G'$ contains a source vertex $s$ and a sink vertex $t$, and we subsequently introduce auxiliary vertices.

The arc set is defined as follows.
For each $v \in V$, add arcs between consecutive copies, namely
$$(v_0',v_1), (v_1,v_1'), (v_1',v_2), \dots, (v_M,v_M').$$
For each edge $\{v,w\} \in E$ and each $0 \leq i < M$, add two auxiliary vertices $q$ and $r$ (separately for each case), and the following arcs:
$$(v_i',q), (w_i',q), (q,r), (r,v_{i+1}), (r,w_{i+1}).$$
This construction ensures that no two robots traverse the same edge in opposite directions at the same time (edge conflict).
Finally, add arcs $(s,v_0)$ for all $v \in S$ and arcs $(v_M',t)$ for all $v \in T$.
All arcs have unit capacity.
The resulting network $G'$ then has a maximum $(s,t)$-flow with value $|R|$ if and only if the corresponding instance of unlabeled coordinated motion planning in $G$ has a feasible schedule with makespan $M$.
