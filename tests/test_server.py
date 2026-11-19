"""The HTTP planning service, end to end on a local port.

Run after installing the package: pytest tests
"""

import http.client
import json
import threading

import numpy as np
import pytest
import vamp

import coven
from coven import build_roadmap
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
