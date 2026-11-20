"""MotionBenchMaker problems for the Panda, as planning service scenes.

VAMP ships the MotionBenchMaker (MBM) problems in resources/panda/problems.tar.bz2: seven scenarios
(table_pick, table_under_pick, box, bookshelf_small, bookshelf_tall, bookshelf_thin, cage) of 100
problems each, as MoveIt scene and request files. This module downloads that archive from the VAMP
commit coven builds against, and converts each problem the way VAMP's own problem_dict_to_vamp()
does: cylinders become capsules, except in "box", where VAMP uses a box around each cylinder.

    scenarios()          -> ["bookshelf_small", ...]
    problems("box")      -> [{"scene": {...}, "start": [7 floats], "goal": [7 floats]}, ...]

A scene is the body coven-server takes for PUT /scenes/{id}, with a "name" on every obstacle.
"""

import json
import re
import tarfile
import urllib.request
from functools import lru_cache

import numpy as np
import yaml
from vamp.transformations import (
    euler_from_matrix,
    identity_matrix,
    quaternion_matrix,
    translation_from_matrix,
    translation_matrix,
)

from coven.panda_model import VAMP_COMMIT, cache_dir

JOINTS = [f"panda_joint{i}" for i in range(1, 8)]
FILE = re.compile(r"^problems/(\w+)_panda/(scene|request)(\d+)\.yaml$")


def archive():
    """Path to the problem archive, downloading it (about 340 kB) on first use."""
    path = cache_dir() / "problems.tar.bz2"
    if not path.exists():
        url = f"https://raw.githubusercontent.com/KavrakiLab/vamp/{VAMP_COMMIT}/resources/panda/problems.tar.bz2"
        path.parent.mkdir(parents=True, exist_ok=True)
        urllib.request.urlretrieve(url, path.with_suffix(".part"))
        path.with_suffix(".part").replace(path)
    return path


def pose(tf):
    return np.dot(translation_matrix(tf["position"]), quaternion_matrix(tf["orientation"]))


def scene(data, scenario):
    """A service scene from a MoveIt planning scene."""
    result = {"spheres": [], "cuboids": [], "capsules": []}
    for co in data["world"]["collision_objects"]:
        base = identity_matrix() if "pose" not in co else pose(co["pose"])
        primitive = co["primitives"][0]
        transform = np.dot(base, pose(co["primitive_poses"][0]))
        center = translation_from_matrix(transform).tolist()
        euler = [float(a) for a in euler_from_matrix(transform)]
        dimensions = primitive["dimensions"]
        kind = primitive["type"]
        if kind == "sphere":
            result["spheres"].append({"name": co["id"], "center": center, "radius": dimensions[0]})
        elif kind == "box":
            result["cuboids"].append({"name": co["id"], "center": center, "euler": euler,
                                      "half_extents": [d / 2 for d in dimensions]})
        elif kind == "cylinder" and scenario == "box":
            # As VAMP: a top-down grasp of the cylinder needs the box, not a capsule's rounded ends
            length, radius = dimensions
            result["cuboids"].append({"name": co["id"], "center": center, "euler": euler,
                                      "half_extents": [radius, radius, length / 2]})
        elif kind == "cylinder":
            length, radius = dimensions
            result["capsules"].append({"name": co["id"], "center": center, "euler": euler, "radius": radius,
                                       "length": length})
        else:
            raise ValueError(f"unknown primitive {kind}")
    return result


def request(data):
    """Start and goal from a MoveIt motion plan request."""
    state = data["start_state"]["joint_state"]
    start = [state["position"][state["name"].index(j)] for j in JOINTS]
    constraints = {c["joint_name"]: c["position"] for c in data["goal_constraints"][0]["joint_constraints"]}
    return start, [constraints[j] for j in JOINTS]


@lru_cache(maxsize=None)
def _all():
    cached = cache_dir() / "problems.json"
    if cached.exists():
        return json.loads(cached.read_text())

    files = {}
    with tarfile.open(archive(), "r:bz2") as tar:
        for member in tar.getmembers():
            match = FILE.match(member.name)
            if match:
                scenario, kind, number = match.groups()
                files[(scenario, int(number), kind)] = yaml.safe_load(tar.extractfile(member))

    result = {}
    for scenario, number in sorted({(s, n) for s, n, _ in files}):
        start, goal = request(files[(scenario, number, "request")])
        result.setdefault(scenario, []).append(
            {"scene": scene(files[(scenario, number, "scene")], scenario), "start": start, "goal": goal})
    cached.write_text(json.dumps(result))
    return result


def scenarios():
    return sorted(_all())


def problems(scenario):
    """The scenario's 100 problems, in file order; KeyError for an unknown scenario."""
    return _all()[scenario]
