# Optimal Solver for Makespan-Optimal Coordinated Motion Planning for Unlabeled Robots in Graphs



## Installation
It should be easy to install the solver using the following command:

```bash
pip install --verbose git+https://github.com/KaiKobbe/unlabeled_cmp_solver
```

## Max-Flow Reduction by Yu and LaValle
Consider an instance of unlabeled coordinated motion planning on a graph $G = (V,E)$ with a set of robots $R$, start positions $S \subseteq V$, and target positions $T \subseteq V$.
We assume $|R| = |S| \leq |T|$.
The reduction solves the decision problem of whether a feasible schedule of makespan $M$ exists.

We construct a directed graph $G'$ as follows.
For each vertex $v \in V$, the vertex set of $G'$ contains $2M + 1$ copies of it, namely
$$
    v_0 = v_0', v_1, v_1', v_2, v_2', \dots, v_M, v_M'.
$$
In addition, $G'$ contains a source vertex $s$ and a sink vertex $t$, and we subsequently introduce auxiliary vertices.

The arc set is defined as follows.
For each $v \in V$, add arcs between consecutive copies, namely
$$
    (v_0',v_1), (v_1,v_1'), (v_1',v_2), \dots, (v_M,v_M').
$$
For each edge $\{v,w\} \in E$ and each $0 \leq i < M$, add two auxiliary vertices $q$ and $r$ (separately for each case), and the following arcs:
$$
    (v_i',q), (w_i',q), (q,r), (r,v_{i+1}), (r,w_{i+1}).
$$
This construction ensures that no two robots traverse the same edge in opposite directions at the same time (edge conflict).
Finally, add arcs $(s,v_0)$ for all $v \in S$ and arcs $(v_M',t)$ for all $v \in T$.
All arcs have unit capacity.
The resulting network $G'$ then has a maximum $(s,t)$-flow with value $|R|$ if and only if the corresponding instance of unlabeled coordinated motion planning in $G$ has a feasible schedule with makespan $M$.
