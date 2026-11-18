"""Replanning on a fixed roadmap as the environment changes.

The roadmap is built once, in free space. Every replan rechecks all of its vertices and edges against
the current environment, then searches the valid part for a path.
"""

import time
from dataclasses import dataclass

import numpy as np
import vamp

from coven import _core

robot = vamp.panda
planner = _core.panda


def roadmap_settings(nodes):
    settings = vamp.PRMSettings(vamp.PRMNeighborParams(robot.dimension(), robot.space_measure()))
    settings.max_samples = nodes
    settings.max_iterations = 1000 * nodes
    return settings


def build_roadmap(nodes, start, goal, environment=None, parallel=True, threads=None, batch_size=1024):
    """A PRM roadmap with `nodes` vertices; the same roadmap whether built by VAMP or in parallel."""
    environment = environment if environment is not None else vamp.Environment()
    settings = roadmap_settings(nodes)
    if not parallel:
        return robot.roadmap(start, goal, environment, settings, robot.halton())
    _core.set_num_threads(threads or _core.max_threads())
    return planner.build_roadmap(start, goal, environment, settings, robot.halton(), batch_size)


@dataclass
class Step:
    path: np.ndarray  # (waypoints, dimension); empty when no path was found
    validity: _core.RoadmapValidity
    seconds: float  # revalidation and search together
    revalidate_seconds: float
    edges: int  # edges rechecked


class Replanner:
    """Holds one roadmap and replans on it with a fixed number of threads."""

    def __init__(self, roadmap, threads=1):
        self.roadmap = roadmap
        self.graph = planner.graph(roadmap)
        self.threads = threads

    def replan(self, start, goal, environment):
        # The thread count is per calling thread, so each Replanner can use its own
        _core.set_num_threads(self.threads)
        begin = time.perf_counter()
        validity = planner.revalidate(self.roadmap, self.graph, environment)
        middle = time.perf_counter()
        path = planner.query(self.roadmap, self.graph, validity, start, goal, environment)
        end = time.perf_counter()
        waypoints = path.numpy() if len(path) else np.empty((0, robot.dimension()), dtype=np.float32)
        return Step(waypoints, validity, end - begin, middle - begin, len(self.graph))
