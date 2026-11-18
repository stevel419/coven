"""Checks of the Python bindings against VAMP's own planner and validators.

Run after installing the package: pytest tests
"""

import numpy as np
import pytest
import vamp

import coven
from coven import Replanner, build_roadmap

robot = vamp.panda
START = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785], dtype=np.float32)
GOAL = np.array([2.35, 1.0, 0.0, -0.8, 0, 2.5, 0.785], dtype=np.float32)
CAGE = [
    [0.55, 0, 0.25], [0.35, 0.35, 0.25], [0, 0.55, 0.25], [-0.55, 0, 0.25], [-0.35, -0.35, 0.25],
    [0, -0.55, 0.25], [0.35, -0.35, 0.25], [0.35, 0.35, 0.8], [0, 0.55, 0.8], [-0.35, 0.35, 0.8],
    [-0.55, 0, 0.8], [-0.35, -0.35, 0.8], [0, -0.55, 0.8], [0.35, -0.35, 0.8],
]


def cage(shift=0.0):
    environment = vamp.Environment()
    for x, y, z in CAGE:
        environment.add_sphere(vamp.Sphere([x + shift, y, z], 0.2))
    return environment


def motion_valid(a, b, environment):
    # Path.validate() checks each segment at the robot's resolution, as the planner does
    path = robot.Path()
    path.append(a)
    path.append(b)
    return path.validate(environment)


@pytest.fixture(scope="module")
def roadmap():
    return build_roadmap(800, START, GOAL, cage(), parallel=False)


def test_parallel_roadmap_equals_serial(roadmap):
    for threads in (1, coven.max_threads()):
        parallel = build_roadmap(800, START, GOAL, cage(), parallel=True, threads=threads)
        assert np.array_equal(coven.panda.vertices(parallel), coven.panda.vertices(roadmap))
        assert [sorted(e) for e in parallel.edges] == [sorted(e) for e in roadmap.edges]


def test_revalidation_equals_serial_recheck(roadmap):
    moved = cage(0.05)
    vertices = coven.panda.vertices(roadmap)
    edges = coven.panda.graph(roadmap).edges
    expected_vertices = np.array([robot.validate(v, moved) for v in vertices])
    expected_edges = np.array([
        expected_vertices[i] and expected_vertices[j] and motion_valid(vertices[i], vertices[j], moved)
        for i, j in edges
    ])
    assert 0 < expected_edges.sum() < len(edges)

    for threads in (1, 2, coven.max_threads()):
        step = Replanner(roadmap, threads).replan(START, GOAL, moved)
        assert np.array_equal(step.validity.vertices, expected_vertices)
        assert np.array_equal(step.validity.edges, expected_edges)


@pytest.mark.parametrize("shift", [0.0, 0.02, 0.05])
def test_paths_are_valid(shift):
    environment = cage(shift)
    replanner = Replanner(build_roadmap(3000, START, GOAL), coven.max_threads())
    step = replanner.replan(START, GOAL, environment)
    assert len(step.path) >= 2
    assert np.allclose(step.path[0], START) and np.allclose(step.path[-1], GOAL)
    for a, b in zip(step.path[:-1], step.path[1:]):
        assert motion_valid(a, b, environment)


def test_no_path_when_everything_is_blocked(roadmap):
    blocked = vamp.Environment()
    blocked.add_sphere(vamp.Sphere([0, 0, 0.5], 2.0))
    step = Replanner(roadmap).replan(START, GOAL, blocked)
    assert len(step.path) == 0
    assert not step.validity.edges.any()
