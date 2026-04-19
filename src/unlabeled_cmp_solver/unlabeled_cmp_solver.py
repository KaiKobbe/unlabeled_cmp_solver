import logging
from dataclasses import dataclass
from typing import Any

from ortools.graph.python import max_flow

from .instance import Instance
from .schedule import Schedule
from ._utils.timer import Timer

Node = Any

@dataclass
class _Network:
    start_nodes: list[int]
    end_nodes: list[int]
    capacities: list[int]
    source: int
    sink: int

class UnlabeledCMPSolver:
    """
    Solver for makespan-optimal unlabeled coordinated motion planning on graphs.

    Given an Instance, computes a schedule minimizing the makespan via binary search,
    where each step decides feasibility using a reduction to maximum s-t-flow on a time-expanded network, as described by Yu and LaValle.
    """

    def __init__(self, instance: Instance, logger: logging.Logger | None=None) -> None:
        assert isinstance(instance, Instance), "instance must be an Instance object"
        # Instance data
        self._instance = instance
        self._graph = instance.graph
        self._start_positions = instance.start_positions
        self._target_positions = instance.target_positions
        self._num_nodes = instance.num_nodes
        self._num_robots = instance.num_robots
        self._instance_diameter = instance.diameter

        # Node index mappings
        nodes = list(self._graph.nodes)
        self._node_idx = {v: i for i, v in enumerate(nodes)}
        self._idx_node = {i: v for i, v in enumerate(nodes)}

        # Solver state
        self._stats = {}
        self._timer = Timer()
        self._logger = logger if logger is not None else logging.getLogger("UnlabeledCMPSolver")

    def optimize_makespan(self) -> Schedule:
        """
        Computes a schedule with minimum makespan using binary search on the makespan.
        The search range is initialized with the instance diameter as a lower bound and 2n - 1 as an upper bound, where n is the number of vertices (feasible by Yu and LaValle, Theorem 17).
        At each step, feasibility of the decision problem is decided via a max-flow computation on a time-expanded network; see method decide_makespan.
        """
        self._timer.reset()
        binary_search_iterations = 0

        lb = self._instance_diameter
        ub = 2 * self._num_nodes - 1
        best_schedule = self.decide_makespan(ub)
        self._logger.info(f"Starting binary search with lb={lb}, ub={ub}.")

        while lb < ub:
            binary_search_iterations += 1
            mid = (lb + ub) // 2
            if schedule := self.decide_makespan(mid):
                best_schedule = schedule
                ub = mid
            else:
                lb = mid + 1

        self._logger.info(f"Finished binary search. Optimal makespan: {ub}.")
        self._stats["total_time"] = self._timer.time()
        self._stats["binary_search_iterations"] = binary_search_iterations
        self._stats["optimal_makespan"] = ub
        return best_schedule

    
    def decide_makespan(self, m: int) -> Schedule | None:
        """
        Decides if a feasible schedule of makespan m exists.
        Returns the schedule if feasible, otherwise None.
        """
        solver = max_flow.SimpleMaxFlow()
    
        t0 = self._timer.time()
        network = self._construct_time_expanded_network(m)
        t1 = self._timer.time()
    
        num_arcs = len(network.start_nodes)
        num_nodes = network.sink + 1

        all_arcs = solver.add_arcs_with_capacity(network.start_nodes, network.end_nodes, network.capacities)

        t2 = self._timer.time()
        status = solver.solve(network.source, network.sink)
        t3 = self._timer.time()

        assert status == solver.OPTIMAL, "max-flow solver did not finish with optimality"

        flow_value = solver.optimal_flow()
        probe_key = f"probe(m={m})"
        self._stats[probe_key] = {
            "network_nodes": num_nodes,
            "network_arcs": num_arcs,
            "flow_value": flow_value,
            "feasible": flow_value == self._num_robots,
            "network_construction_time": t1-t0,
            "max_flow_time": t3-t2,
        }

        if flow_value != self._num_robots:
            return None

        solution_flows = solver.flows(all_arcs)
        flow_graph: dict[int, list[int]] = {}
        for i, arc in enumerate(all_arcs):
            if solution_flows[i] > 0:
                u = solver.tail(arc)
                v = solver.head(arc)
                flow_graph.setdefault(u, []).append(v)

        robot_paths = self._decompose_flow(flow_graph, network.source, network.sink, m)
        return Schedule(robot_paths)


    def decide_stretch(self, s: float) -> Schedule | None:
        """
        Decides if a feasible schedule with stretch factor at most s exists.
        The stretch factor is the ratio of makespan to the instance diameter.
        Returns the schedule if feasible, otherwise None.
        """
        return self.decide_makespan(int(self._instance_diameter * s))
    
    def get_stats(self) -> dict:
        """Returns statistics collected during the last call to optimize_makespan."""
        return dict(self._stats)
    
    def _construct_time_expanded_network(self, m: int) -> _Network:
        """
        Constructs the time-expanded network for makespan m as described by Yu and LaValle.
        Returns a _Network containing the arc lists and source/sink IDs for the max-flow solver.
        """
        self._logger.info(f"Starting to construct the time expanded network for makespan {m}.")

        nodes = list(self._graph.nodes)
        edges = list(self._graph.edges)

        # Node ID scheme per vertex v (index idx):
        # v_i (i >= 1)  : idx * (2m+1) + 2i - 1
        # v_i' (i >= 0) : idx * (2m+1) + 2i

        def v_prime(v, i):  # v_i'
            idx = self._node_idx[v]
            return idx * (2 * m + 1) + 2 * i

        def v_plain(v, i):  # v_i, i >= 1
            idx = self._node_idx[v]
            return idx * (2 * m + 1) + (2 * i - 1)

        # Auxiliary node ID: 2 per (edge, timestep) pair
        aux_start = self._num_nodes * (2 * m + 1)

        def q_node(e_idx, i):
            return aux_start + (e_idx * m + i) * 2

        def r_node(e_idx, i):
            return aux_start + (e_idx * m + i) * 2 + 1

        source = aux_start + 2 * len(edges) * m
        sink = source + 1

        starts, ends, capacities = [], [], []

        def add_arc(u, v):
            starts.append(u)
            ends.append(v)
            capacities.append(1)

        # Vertex arcs: for each vertex v add v_0' -> v_1 -> v_1' -> v_2 -> ... -> v_m -> v_m'
        for v in nodes:
            add_arc(v_prime(v, 0), v_plain(v, 1))
            for i in range(1, m):
                add_arc(v_plain(v, i), v_prime(v, i))
                add_arc(v_prime(v, i), v_plain(v, i + 1))
            add_arc(v_plain(v, m), v_prime(v, m))

        # Edge arcs: for each {v,w} and each time step 0 <= i < m, introduce auxiliary vertices q and r, then add
        # v_i' -> q and w_i' -> q,
        # q -> r, and
        # r -> v_{i+1} and r -> w_{i+1}.
        for e_idx, (v, w) in enumerate(edges):
            for i in range(m):
                q = q_node(e_idx, i)
                r = r_node(e_idx, i)
                add_arc(v_prime(v, i), q)
                add_arc(v_prime(w, i), q)
                add_arc(q, r)
                add_arc(r, v_plain(v, i + 1))
                add_arc(r, v_plain(w, i + 1))

        # Source -> v_0' for each start position
        for s in self._start_positions:
            add_arc(source, v_prime(s, 0))

        # v_m' -> sink for each target position
        for t in self._target_positions:
            add_arc(v_prime(t, m), sink)

        self._logger.info(f"Finished constructing the time expanded network for makespan {m}.")
        return _Network(starts, ends, capacities, source, sink)
    
    def _decode_node(self, node_id: int, m: int) -> tuple[Node, int] | None:
        """Returns (vertex, timestep) if node_id encodes a v_i node, else None."""
        aux_start = self._num_nodes * (2 * m + 1)
        if node_id >= aux_start:
            return None
        idx = node_id // (2 * m + 1)
        offset = node_id % (2 * m + 1)
        # by convention, v_0 == v_0'
        if offset == 0:
            return self._idx_node[idx], 0
        elif offset % 2 == 1:
            return self._idx_node[idx], (offset + 1) // 2
        else:
            return None

    def _decompose_flow(self, flow_graph: dict[int, list[int]], source: int, sink: int, m: int) -> list[list[Node]]:
        """
        Decomposes a unit flow into individual robot trajectories.
        Each trajectory is a list of m + 1 vertices indicating the robot's position at each timestep.
        """
        robot_paths = []
        while True:
            path = self._find_path(flow_graph, source, sink)
            if path is None:
                break
            for u, v in zip(path, path[1:]):
                flow_graph[u].remove(v)
            trajectory: list[Node | None] = [None] * (m + 1)
            for node_id in path:
                decoded = self._decode_node(node_id, m)
                if decoded is not None:
                    vertex, timestep = decoded
                    trajectory[timestep] = vertex
            assert all(v is not None for v in trajectory), "Flow decomposition produced incomplete trajectory"
            robot_paths.append(trajectory)
        return robot_paths
    
    def _find_path(self, flow_graph: dict[int, list[int]], source: int, sink: int) -> list[int] | None:
        """Returns a path from source to sink in the flow graph using DFS, or None if no path exists."""
        stack = [(source, [source], {source})]
        while stack:
            node, path, visited = stack.pop()
            for neighbor in flow_graph.get(node, []):
                if neighbor == sink:
                    return path + [sink]
                if neighbor not in visited:
                    stack.append((neighbor, path + [neighbor], visited | {neighbor}))
        return None