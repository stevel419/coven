"""The HTTP planning service, end to end on a local port.

Run after installing the package: pytest tests
"""

import http.client
import json
import math
import re
import threading
import time

import numpy as np
import pytest
import vamp

import coven
from coven import build_roadmap, problems
from coven.server import make_server

from test_replanner import CAGE, GOAL, START, cage, motion_valid


@pytest.fixture(scope="module")
def server():
    service = coven.panda.Service(build_roadmap(3000, START, GOAL), threads=coven.max_threads())
    server = make_server(service, port=0)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    yield server
    server.shutdown()
    server.server_close()


def request(server, method, path, body=None):
    connection = http.client.HTTPConnection(*server.server_address, timeout=30)
    connection.request(method, path, body=None if body is None else json.dumps(body),
                       headers={"Content-Type": "application/json"})
    response = connection.getresponse()
    data = response.read()
    connection.close()
    return response.status, json.loads(data) if data else None


def cage_body(shift=0.0):
    return {"spheres": [{"center": [x + shift, y, z], "radius": 0.2} for x, y, z in CAGE]}


def test_plan_in_a_scene(server):
    status, body = request(server, "PUT", "/scenes/cage", cage_body())
    assert status == 200 and body["version"] == 1

    for shift in (0.02, 0.05):
        status, body = request(server, "PUT", "/scenes/cage", cage_body(shift))
        assert status == 200
        version = body["version"]

        status, body = request(server, "POST", "/scenes/cage/plan", {"start": START.tolist(), "goal": GOAL.tolist()})
        assert status == 200
        assert body["version"] == version and body["rounds"] >= 1
        path = np.array(body["path"], dtype=np.float32)
        assert len(path) > 2
        assert np.allclose(path[0], START) and np.allclose(path[-1], GOAL)
        for a, b in zip(path[:-1], path[1:]):
            assert motion_valid(a, b, cage(shift))


def test_cuboids_and_trivial_path(server):
    far = {"cuboids": [{"center": [3, 3, 3], "euler": [0, 0, 0.3], "half_extents": [0.1, 0.1, 0.1]}]}
    assert request(server, "PUT", "/scenes/far", far)[0] == 200
    status, body = request(server, "POST", "/scenes/far/plan", {"start": START.tolist(), "goal": GOAL.tolist()})
    assert status == 200 and len(body["path"]) == 2


def test_concurrent_clients(server):
    assert request(server, "PUT", "/scenes/shared", cage_body(0.02))[0] == 200
    results = []

    def client():
        results.append(request(server, "POST", "/scenes/shared/plan",
                               {"start": START.tolist(), "goal": GOAL.tolist()}))

    threads = [threading.Thread(target=client) for _ in range(16)]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()
    assert len(results) == 16
    assert all(status == 200 and len(body["path"]) > 2 for status, body in results)


def get(server, path):
    connection = http.client.HTTPConnection(*server.server_address, timeout=30)
    connection.request("GET", path)
    response = connection.getresponse()
    data = response.read()
    connection.close()
    return response.status, response.getheader("Content-Type"), data


def test_web_page(server):
    status, content_type, page = get(server, "/")
    assert status == 200 and content_type.startswith("text/html")
    script = re.search(r'src="\./(assets/[^"]+\.js)"', page.decode()).group(1)
    status, content_type, _ = get(server, f"/{script}")
    assert status == 200 and "javascript" in content_type
    for path in ("/panda/../server.py", "/../server.py", "/assets/../../server.py", "/%2e%2e/server.py"):
        assert get(server, path)[0] == 404, path


def test_problem_conversion():
    """A MoveIt scene and request become a service scene, start and goal, as in VAMP's converter."""
    quarter = [0, 0, math.sin(math.pi / 8), math.cos(math.pi / 8)]  # 45 degrees about z, (x, y, z, w)
    data = {"world": {"collision_objects": [
        {"id": "ball", "primitives": [{"type": "sphere", "dimensions": [0.1]}],
         "primitive_poses": [{"position": [1, 0, 0], "orientation": [0, 0, 0, 1]}]},
        {"id": "can", "primitives": [{"type": "cylinder", "dimensions": [0.2, 0.05]}],
         "primitive_poses": [{"position": [0, 1, 0], "orientation": quarter}]},
        {"id": "crate", "pose": {"position": [0, 0, 1], "orientation": [0, 0, 0, 1]},
         "primitives": [{"type": "box", "dimensions": [0.2, 0.4, 0.6]}],
         "primitive_poses": [{"position": [0.5, 0, 0], "orientation": quarter}]},
    ]}}
    scene = problems.scene(data, "table_pick")
    assert scene["spheres"] == [{"name": "ball", "center": [1.0, 0.0, 0.0], "radius": 0.1}]
    can, = scene["capsules"]
    assert can["radius"] == 0.05 and can["length"] == 0.2 and can["euler"] == pytest.approx([0, 0, math.pi / 4])
    crate, = scene["cuboids"]
    assert crate["center"] == pytest.approx([0.5, 0, 1]) and crate["half_extents"] == pytest.approx([0.1, 0.2, 0.3])

    # In "box", VAMP replaces cylinders by boxes around them
    boxed = problems.scene(data, "box")
    assert not boxed["capsules"] and boxed["cuboids"][0]["half_extents"] == [0.05, 0.05, 0.1]

    names = [f"panda_joint{i}" for i in range(1, 8)]
    start, goal = problems.request({
        "start_state": {"joint_state": {"name": names[::-1], "position": list(range(7))[::-1]}},
        "goal_constraints": [{"joint_constraints": [{"joint_name": n, "position": 10 + i} for i, n in enumerate(names)]}],
    })
    assert start == list(range(7)) and goal == [10 + i for i in range(7)]


def test_limits():
    service = coven.panda.Service(build_roadmap(500, START, GOAL), threads=1)
    server = make_server(service, port=0, max_scenes=2, scene_ttl=0.5)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    try:
        plan = {"start": START.tolist(), "goal": GOAL.tolist()}

        # A third scene drops the least recently used one
        for name in ("a", "b"):
            assert request(server, "PUT", f"/scenes/{name}", {})[0] == 200
        assert request(server, "POST", "/scenes/a/plan", plan)[0] == 200  # a is now the newer one
        assert request(server, "PUT", "/scenes/c", {})[0] == 200
        assert request(server, "POST", "/scenes/b/plan", plan)[0] == 404
        assert request(server, "POST", "/scenes/a/plan", plan)[0] == 200

        # Unused scenes expire
        time.sleep(0.7)
        assert request(server, "PUT", "/scenes/d", {})[0] == 200
        assert request(server, "POST", "/scenes/a/plan", plan)[0] == 404
        assert request(server, "POST", "/scenes/d/plan", plan)[0] == 200

        # Too many obstacles, too large a body
        many = {"spheres": [{"center": [3, 3, 3], "radius": 0.1}] * 501}
        assert request(server, "PUT", "/scenes/e", many)[0] == 400
        connection = http.client.HTTPConnection(*server.server_address, timeout=30)
        connection.putrequest("PUT", "/scenes/e")
        connection.putheader("Content-Length", str(2 << 20))
        connection.endheaders()
        assert connection.getresponse().status == 413
        connection.close()
    finally:
        server.shutdown()
        server.server_close()


def test_capsules(server):
    body = {"capsules": [{"center": [3, 3, 3], "euler": [0, 0, 0], "radius": 0.1, "length": 0.3}]}
    assert request(server, "PUT", "/scenes/capsule", body)[0] == 200
    bad = {"capsules": [{"center": [3, 3, 3], "radius": 0, "length": 0.3}]}
    assert request(server, "PUT", "/scenes/capsule", bad)[0] == 400


def test_errors(server):
    plan = {"start": START.tolist(), "goal": GOAL.tolist()}
    assert request(server, "POST", "/scenes/nowhere/plan", plan)[0] == 404
    assert request(server, "PUT", "/scenes/errors", {"spheres": []})[0] == 200
    assert request(server, "POST", "/scenes/errors/plan", {"start": [0, 1], "goal": plan["goal"]})[0] == 400
    assert request(server, "POST", "/scenes/errors/plan", {"start": START.tolist()})[0] == 400
    assert request(server, "PUT", "/scenes/errors", {"spheres": [{"center": [0, 0], "radius": 1}]})[0] == 400
    assert request(server, "PUT", "/scenes/errors", {"spheres": [{"center": [0, 0, 0], "radius": -1}]})[0] == 400
    assert request(server, "GET", "/nowhere")[0] == 404
    assert request(server, "DELETE", "/scenes/errors")[0] == 204
    assert request(server, "DELETE", "/scenes/errors")[0] == 404

    status, stats = request(server, "GET", "/stats")
    assert status == 200 and stats["requests"] >= 1 and stats["vertices"] == 3000


def test_service_matches_replanner():
    roadmap = build_roadmap(3000, START, GOAL)
    service = coven.panda.Service(roadmap, threads=2)
    environment = cage(0.05)
    service.set_scene("cage", environment)
    reply = service.plan("cage", START, GOAL)
    step = coven.Replanner(roadmap, 2).replan(START, GOAL, environment)
    assert reply.status == "ok" and len(reply.path) > 2

    def cost(path):
        return float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum())

    assert cost(reply.path.numpy()) == pytest.approx(cost(step.path), rel=1e-4)
    assert vamp.panda.Path is type(reply.path)
