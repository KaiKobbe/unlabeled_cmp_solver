"""
This package provides a solver for makespan-optimal coordinated motion planning given unlabeled robots in arbitrary graphs.
The problem is polynomial-time solvable by applying a reduction to maximum s-t-flow with unit capacities, introduced by Yu and LaValle:
Multi-agent Path Planning and Network Flow, Algorithmic Foundations of Robotics, 2013, doi: 10.1007/978-3-642-36279-8_10

The goal is to move all robots from their start positions to distinct target positions simultaneously, minimizing the number of timesteps.
Robots are unlabeled, meaning any robot may occupy any target position.
At each timestep, every robot may either move to an adjacent vertex or stay in place.
No two robots may occupy the same vertex at the same time, and no two robots may swap positions in a single timestep.
"""
from logging import Logger

from .instance import Instance
from .schedule import Schedule
from .unlabeled_cmp_solver import UnlabeledCMPSolver
from .visualization import visualize_schedule

def optimize_makespan(instance: Instance, logger: Logger | None=None) -> tuple[Schedule, dict]:
    """Computes schedule with minimum makespan."""
    solver = UnlabeledCMPSolver(instance, logger=logger)
    schedule = solver.optimize_makespan()
    return schedule, solver.get_stats()

def decide_makespan(instance: Instance, m: int, logger: Logger | None=None) -> tuple[Schedule | None, dict]:
    """Decides if a schedule with makespan m exists and returns it if possible."""
    solver = UnlabeledCMPSolver(instance, logger=logger)
    schedule = solver.decide_makespan(m)
    return schedule, solver.get_stats()

def decide_stretch(instance: Instance, s: float, logger: Logger | None=None) -> tuple[Schedule | None, dict]:
    """
    Decides if a schedule with stretch factor s exists and returns it if possible.
    The stretch factor of a schedule is defined as the makespan of the schedule divided by the diameter of the instance.
    """
    solver = UnlabeledCMPSolver(instance, logger=logger)
    schedule = solver.decide_stretch(s)
    return schedule, solver.get_stats()

__all__ = ["Instance", "Schedule", "visualize_schedule", "optimize_makespan", "decide_makespan", "decide_stretch"]
