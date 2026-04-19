from typing import Any

import networkx as nx
from networkx.algorithms.bipartite import maximum_matching

Node = Any

class Instance:
    """
    Represents an instance of unlabeled coordinated motion planning in graphs.

    An instance consists of:
    - a simple, connected, undirected, and unweighted graph,
    - a subset of vertices indicating start positions, one per robot, and
    - a subset of vertices indicating target positions.
    """

    def __init__(self, graph: nx.Graph, start_positions: list[Node], target_positions: list[Node]) -> None:
        assert nx.is_connected(graph), "Graph must be connected"
        assert not graph.is_directed(), "Graph must be undirected"
        assert nx.number_of_selfloops(graph) == 0, "Graph must have no self-loops"
        assert not isinstance(graph, (nx.MultiGraph, nx.MultiDiGraph)), "Graph must have no parallel edges"
        assert all(w == {} for _, _, w in graph.edges(data=True)), "Graph must be unweighted"

        all_nodes = set(graph.nodes)
        assert all(v in all_nodes for v in start_positions), "All start positions must be nodes in the graph"
        assert all(v in all_nodes for v in target_positions), "All target positions must be nodes in the graph"

        assert len(start_positions) == len(set(start_positions)), "Start positions must be unique"
        assert len(target_positions) == len(set(target_positions)), "Target positions must be unique"
        assert len(start_positions) > 0, "There must be at least one start position"
        assert len(start_positions) <= len(target_positions), "Number of start positions must not exceed target positions"

        self._graph = graph
        self._start_positions = start_positions
        self._target_positions = target_positions
        self._num_robots = len(self._start_positions)
        self._num_nodes = self._graph.number_of_nodes()
        self._diameter = self._compute_diameter()

    @property
    def graph(self) -> nx.Graph:
        return self._graph.copy()
    
    @property
    def start_positions(self) -> list[Node]:
        return list(self._start_positions)
    
    @property
    def target_positions(self) -> list[Node]:
        return list(self._target_positions)
    
    @property
    def num_nodes(self) -> int:
        return self._num_nodes
    
    @property
    def num_robots(self) -> int:
        return self._num_robots
    
    @property
    def diameter(self) -> int:
        return self._diameter

    def _compute_diameter(self) -> int:
        """
        Computes the diameter of the instance which is defined as the smallest possible maximum travel distance when optimally assigning robots to targets.
        This clearly is a lower bound on the makespan for every feasible schedule.
        """

        # Compute shortest path distances between all start and target nodes
        distances: dict[tuple[Node, Node], int] = {}
        for s in self._start_positions:
            lengths = nx.single_source_shortest_path_length(self._graph, s)
            for t in self._target_positions:
                distances[(s, t)] = 0 if s == t else lengths[t]

        # Build bipartite graph
        B = nx.Graph()
        top_nodes = [("s", s) for s in self._start_positions]
        B.add_nodes_from(top_nodes, bipartite=0)
        bottom_nodes = [("t", t) for t in self._target_positions]
        B.add_nodes_from(bottom_nodes, bipartite=1)
        for (s, t), d in distances.items():
            B.add_edge(("s", s), ("t", t), weight=d)

        # MinBottleneck matching via binary search over sorted edge weights
        sorted_weights = sorted(set(distances.values()))

        lb, ub = 0, len(sorted_weights) - 1
        result = sorted_weights[-1]

        while lb < ub:
            mid = (lb + ub) // 2
            threshold = sorted_weights[mid]

            subgraph = nx.Graph()
            subgraph.add_nodes_from(B.nodes(data=True))
            subgraph.add_edges_from(
                (u, v) for u, v, w in B.edges(data="weight") if w <= threshold
            )

            matching = maximum_matching(subgraph, top_nodes=top_nodes)
            if len(matching) // 2 >= self._num_robots:
                result = threshold
                ub = mid
            else:
                lb = mid + 1

        return result
    
    def __repr__(self) -> str:
        return (
            f"Instance(num_nodes={self._num_nodes}, "
            f"num_robots={self._num_robots}, "
            f"num_targets={len(self._target_positions)}, "
            f"diameter={self._diameter})"
        )
