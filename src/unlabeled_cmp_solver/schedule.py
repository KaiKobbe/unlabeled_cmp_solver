from typing import Any

from .instance import Instance

Node = Any

class Schedule:
    """
    Represents a schedule for an instance of unlabeled coordinated motion planning in graphs.
    A schedule of makespan m consists of one path per robot, where each path is a list of m + 1 nodes indicating the robot's position at each timestep 0, 1, ..., m.
    """
    
    def __init__(self, robot_paths: list[list[Node]]) -> None:
        assert len(robot_paths) > 0, "Schedule must have at least one robot"
        self._robot_paths = robot_paths
        assert self._is_valid()

    @property
    def robot_paths(self) -> list[list[Node]]:
        return [list(path) for path in self._robot_paths]
    
    @property
    def num_robots(self) -> int:
        return len(self._robot_paths)

    @property
    def makespan(self) -> int:
        return len(self._robot_paths[0]) - 1
    
    def fits_instance(self, instance: Instance) -> bool:
        """Checks if the schedule is valid for the given instance."""
        graph = instance.graph
        start_positions = instance.start_positions
        target_positions = instance.target_positions
        m = self.makespan

        # Correct number of robots
        if len(self._robot_paths) != instance.num_robots:
            return False

        for path in self._robot_paths:
            # Each robot starts at a start position
            if path[0] not in start_positions:
                return False
            # Each robot ends at a target position
            if path[-1] not in target_positions:
                return False

            for t in range(m):
                u, v = path[t], path[t + 1]
                # Each robot only moves along edges
                if u != v and not graph.has_edge(u, v):
                    return False

        return True
    
    def _is_valid(self) -> bool:
        """Checks if the schedule is in correct shape."""
        m = self.makespan

        # All paths have correct length
        if any(len(path) != m + 1 for path in self._robot_paths):
            return False
        
        # No vertex collision should be possible
        for t in range(m + 1):
            positions_at_t = [path[t] for path in self._robot_paths]
            if len(positions_at_t) != len(set(positions_at_t)):
                return False
        
        # No edge collision should be possible
        for t in range(m):
            swaps = [(path[t], path[t+1]) for path in self._robot_paths]
            for a, b in swaps:
                if (b, a) in swaps and a != b:
                    return False
        
        return True
    
    def __repr__(self) -> str:
        return f"Schedule(num_robots={self.num_robots}, makespan={self.makespan})"
