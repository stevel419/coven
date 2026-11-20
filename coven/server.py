"""HTTP planning service for the Panda.

One roadmap, built at startup in free space, and any number of named scenes. Plan requests from all
clients are answered in rounds whose collision checks run in one parallel loop (coven.panda.Service).

    coven-server --nodes 10000 --threads 16 --port 8000

    PUT    /scenes/{id}       {"spheres": [{"center": [x, y, z], "radius": r}, ...],
                               "cuboids": [{"center": [...], "euler": [...], "half_extents": [...]}, ...],
                               "capsules": [{"center": [...], "euler": [...], "radius": r, "length": l}, ...]}
                              -> {"version": n}
                              Euler angles are VAMP's (static x, then y, then z); capsules lie along
                              their local z axis, with `length` between the centers of their ends.
    DELETE /scenes/{id}       -> 204, or 404
    POST   /scenes/{id}/plan  {"start": [7 floats], "goal": [7 floats]}
                              -> {"path": [[7 floats], ...] or [], "version", "rounds", "checks",
                                  "searches", "fallback", "queued_ms", "total_ms"}
                              The path is collision-free in the scene's "version" current when the
                              request started; later updates do not restart it.
    GET    /stats             -> totals and options
    GET    /problems          -> the MotionBenchMaker scenarios (coven.problems)
    GET    /problems/{name}   -> that scenario's 100 problems: [{"scene", "start", "goal"}, ...]
    GET    /                  -> a web page to edit scenes and plan in them (built from web/)

Errors: 400 malformed body, 404 unknown scene or path, 413 body over 1 MB, 503 overloaded.

Limits: 500 obstacles per scene; at most --max-scenes scenes (the least recently used is dropped), and
scenes unused for --scene-ttl seconds are dropped. A client whose scene was dropped gets 404 and sends it
again.
"""

import argparse
import json
import math
import mimetypes
import re
import threading
import time
from collections import OrderedDict
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import numpy as np
import vamp

from coven import _core, problems
from coven.panda_model import PANDA_FILES, panda_urdf
from coven.replanner import build_roadmap

WEB = Path(__file__).parent / "web"

robot = vamp.panda
# Start and goal of VAMP's sphere cage example; they only seed the roadmap
START = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785], dtype=np.float32)
GOAL = np.array([2.35, 1.0, 0.0, -0.8, 0.0, 2.5, 0.785], dtype=np.float32)
ROUTE = re.compile(r"^/scenes/([A-Za-z0-9_.-]{1,128})(/plan)?$")
STATUS = {"ok": 200, "unknown_scene": 404, "overloaded": 503, "shut_down": 503}
# Limits that keep one client from using up the server's memory
MAX_BODY = 1 << 20  # bytes per request
MAX_OBSTACLES = 500  # per scene; MotionBenchMaker scenes have at most 21


class Scenes:
    """Scenes made over HTTP, by last use. Beyond `limit` scenes the least recently used one is erased, and
    scenes unused for `ttl` seconds are erased; clients can always send a scene again."""

    def __init__(self, service, limit, ttl):
        self.service, self.limit, self.ttl = service, limit, ttl
        self.used = OrderedDict()  # scene -> time of last use, oldest first
        self.lock = threading.Lock()

    def touch(self, scene, create=False):
        now = time.monotonic()
        with self.lock:
            while self.used and now - next(iter(self.used.values())) > self.ttl:
                self.service.erase_scene(self.used.popitem(last=False)[0])
            if scene in self.used or create:
                self.used[scene] = now
                self.used.move_to_end(scene)
            while len(self.used) > self.limit:
                self.service.erase_scene(self.used.popitem(last=False)[0])

    def forget(self, scene):
        with self.lock:
            self.used.pop(scene, None)


class BadRequest(Exception):
    pass


def vector(values, length):
    a = np.asarray(values, dtype=np.float32)
    if a.shape != (length,) or not np.all(np.isfinite(a)):
        raise BadRequest(f"expected {length} finite numbers")
    return a


class TooLarge(BadRequest):
    code = 413


def environment(body):
    """A vamp.Environment from a scene body."""
    lists = [body.get(kind, []) for kind in ("spheres", "cuboids", "capsules")]
    if not all(isinstance(obstacles, list) for obstacles in lists):
        raise BadRequest("spheres, cuboids and capsules must be lists")
    if sum(map(len, lists)) > MAX_OBSTACLES:
        raise BadRequest(f"at most {MAX_OBSTACLES} obstacles per scene")
    env = vamp.Environment()
    try:
        for s in body.get("spheres", []):
            radius = float(s["radius"])
            if not (math.isfinite(radius) and radius > 0):
                raise BadRequest("sphere radius must be positive")
            env.add_sphere(vamp.Sphere(vector(s["center"], 3).tolist(), radius))
        for c in body.get("cuboids", []):
            env.add_cuboid(vamp.Cuboid(vector(c["center"], 3).tolist(),
                                       vector(c.get("euler", [0, 0, 0]), 3).tolist(),
                                       vector(c["half_extents"], 3).tolist()))
        for c in body.get("capsules", []):
            radius, length = float(c["radius"]), float(c["length"])
            if not (math.isfinite(radius) and radius > 0 and math.isfinite(length) and length >= 0):
                raise BadRequest("capsule radius must be positive and length not negative")
            env.add_capsule(vamp.Cylinder(vector(c["center"], 3).tolist(),
                                          vector(c.get("euler", [0, 0, 0]), 3).tolist(), radius, length))
    except (KeyError, TypeError, AttributeError) as e:
        raise BadRequest(f"malformed obstacle: {e}") from e
    return env


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"  # keep-alive, so load tests do not open a connection per request
    # Small responses must not wait for acknowledgments (Nagle's algorithm): that added 40-90 ms to each
    # request on a kept-alive connection
    disable_nagle_algorithm = True
    service = None
    verbose = False

    def do_GET(self):
        path = self.path.split("?", 1)[0]
        if path == "/stats":
            return self.send(200, self.service.stats())
        if path == "/problems":
            return self.send(200, problems.scenarios())  # downloads the problems on first use
        if path.startswith("/problems/"):
            try:
                return self.send(200, problems.problems(path[len("/problems/"):]))
            except KeyError:
                return self.send(404, {"error": "unknown scenario"})
        if path.startswith("/panda/") and path[len("/panda/"):] in PANDA_FILES:
            urdf = panda_urdf()  # downloads the model on first use
            name = path[len("/panda/"):]
            return self.send_file(urdf.parent / name, "application/xml" if name.endswith(".urdf") else "text/plain")

        # The web page (built from web/ into coven/web/)
        file = (WEB / (path.lstrip("/") or "index.html")).resolve()
        if file.is_file() and file.is_relative_to(WEB.resolve()):
            return self.send_file(file, mimetypes.guess_type(file.name)[0] or "application/octet-stream")
        self.send(404, {"error": "not found"})

    def send_file(self, path, content_type):
        self.respond(200, path.read_bytes(), content_type)

    def respond(self, code, data, content_type=None):
        self.send_response(code)
        if content_type:
            self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        # Headers and body in one write: a second small write can wait for the client's delayed
        # acknowledgment, in a proxy between the two ends even when this socket has TCP_NODELAY
        self._headers_buffer.append(b"\r\n" + data)
        self.flush_headers()

    def do_PUT(self):
        def put(scene, body):
            env = environment(body)
            self.server.scenes.touch(scene, create=True)
            self.send(200, {"version": self.service.set_scene(scene, env)})

        self.handle_scene(put, plan=False)

    def do_DELETE(self):
        def delete(scene, _):
            self.server.scenes.forget(scene)
            if self.service.erase_scene(scene):
                self.send(204)
            else:
                self.send(404, {"error": "unknown scene"})

        self.handle_scene(delete, plan=False, body=False)

    def do_POST(self):
        def plan(scene, body):
            try:
                start, goal = body["start"], body["goal"]
            except (KeyError, TypeError) as e:
                raise BadRequest("expected start and goal") from e
            dimension = robot.dimension()
            self.server.scenes.touch(scene)
            reply = self.service.plan(scene, vector(start, dimension), vector(goal, dimension))
            if reply.status != "ok":
                return self.send(STATUS[reply.status], {"error": reply.status})
            self.send(200, {
                "path": reply.path.numpy().tolist() if len(reply.path) else [],
                "version": reply.version,
                "rounds": reply.rounds,
                "checks": reply.checks,
                "searches": reply.searches,
                "fallback": reply.fallback,
                "queued_ms": 1e3 * reply.queued_seconds,
                "total_ms": 1e3 * reply.total_seconds,
            })

        self.handle_scene(plan, plan=True)

    def handle_scene(self, action, plan, body=True):
        match = ROUTE.match(self.path)
        if not match or bool(match.group(2)) != plan:
            self.drain()
            return self.send(404, {"error": "not found"})
        try:
            action(match.group(1), self.read_json() if body else None)
        except BadRequest as e:
            self.send(getattr(e, "code", 400), {"error": str(e)})

    def read_json(self):
        try:
            length = int(self.headers.get("Content-Length", 0))
        except ValueError as e:
            raise BadRequest("bad Content-Length") from e
        if length > MAX_BODY:
            self.close_connection = True  # the body is not read
            raise TooLarge(f"at most {MAX_BODY} bytes per request")
        try:
            body = json.loads(self.rfile.read(length))
        except (ValueError, json.JSONDecodeError) as e:
            raise BadRequest("body must be JSON") from e
        if not isinstance(body, dict):
            raise BadRequest("body must be a JSON object")
        return body

    def drain(self):
        try:
            length = int(self.headers.get("Content-Length", 0) or 0)
        except ValueError:
            length = MAX_BODY + 1
        if length > MAX_BODY:
            self.close_connection = True
        elif length:
            self.rfile.read(length)

    def send(self, code, body=None):
        if body is None:
            return self.respond(code, b"")
        self.respond(code, json.dumps(body).encode(), "application/json")

    def log_message(self, format, *args):
        if self.verbose:
            super().log_message(format, *args)


class Server(ThreadingHTTPServer):
    daemon_threads = True
    request_queue_size = 1024  # pending connections; load tests open many at once


def make_server(service, host="127.0.0.1", port=8000, verbose=False, max_scenes=1000, scene_ttl=3600.0):
    """An HTTP server for a coven.panda.Service; port 0 picks a free one (server.server_address)."""
    handler = type("BoundHandler", (Handler,), {"service": service, "verbose": verbose})
    server = Server((host, port), handler)
    server.scenes = Scenes(service, max_scenes, scene_ttl)
    return server


def main():
    parser = argparse.ArgumentParser(description="Coven planning service")
    parser.add_argument("--nodes", type=int, default=10000, help="roadmap size")
    parser.add_argument("--threads", type=int, default=0, help="0: all cores")
    parser.add_argument("--window-us", type=int, default=0, help="batching window of an idle service")
    parser.add_argument("--eager", action="store_true", help="recheck every edge when a scene changes")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8000)
    parser.add_argument("--verbose", action="store_true", help="log every request")
    parser.add_argument("--max-scenes", type=int, default=1000, help="scenes kept; the least recently used goes")
    parser.add_argument("--scene-ttl", type=float, default=3600, help="seconds an unused scene is kept")
    args = parser.parse_args()

    roadmap = build_roadmap(args.nodes, START, GOAL, threads=args.threads or None)
    service = _core.panda.Service(roadmap, threads=args.threads, window_us=args.window_us, eager=args.eager)
    server = make_server(service, args.host, args.port, args.verbose, args.max_scenes, args.scene_ttl)
    stats = service.stats()
    print(f"serving {stats['vertices']} vertices with {stats['threads']} threads; open "
          f"http://{args.host}:{server.server_address[1]}", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
