"""HTTP planning service for the Panda.

One roadmap, built at startup in free space, and any number of named scenes. Plan requests from all
clients are answered in rounds whose collision checks run in one parallel loop (coven.panda.Service).

    coven-server --nodes 10000 --threads 16 --port 8000

    PUT    /scenes/{id}       {"spheres": [{"center": [x, y, z], "radius": r}, ...],
                               "cuboids": [{"center": [...], "euler": [...], "half_extents": [...]}, ...]}
                              -> {"version": n}
    DELETE /scenes/{id}       -> 204, or 404
    POST   /scenes/{id}/plan  {"start": [7 floats], "goal": [7 floats]}
                              -> {"path": [[7 floats], ...] or [], "version", "rounds", "checks",
                                  "searches", "fallback", "queued_ms", "total_ms"}
                              The path is collision-free in the scene's "version" current when the
                              request started; later updates do not restart it.
    GET    /stats             -> totals and options

Errors: 400 malformed body, 404 unknown scene or path, 503 overloaded.
"""

import argparse
import json
import math
import re
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np
import vamp

from coven import _core
from coven.replanner import build_roadmap

robot = vamp.panda
# Start and goal of VAMP's sphere cage example; they only seed the roadmap
START = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785], dtype=np.float32)
GOAL = np.array([2.35, 1.0, 0.0, -0.8, 0.0, 2.5, 0.785], dtype=np.float32)
ROUTE = re.compile(r"^/scenes/([A-Za-z0-9_.-]{1,128})(/plan)?$")
STATUS = {"ok": 200, "unknown_scene": 404, "overloaded": 503, "shut_down": 503}


class BadRequest(Exception):
    pass


def vector(values, length):
    a = np.asarray(values, dtype=np.float32)
    if a.shape != (length,) or not np.all(np.isfinite(a)):
        raise BadRequest(f"expected {length} finite numbers")
    return a


def environment(body):
    """A vamp.Environment from a scene body."""
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
    except (KeyError, TypeError, AttributeError) as e:
        raise BadRequest(f"malformed obstacle: {e}") from e
    return env


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"  # keep-alive, so load tests do not open a connection per request
    service = None
    verbose = False

    def do_GET(self):
        if self.path == "/stats":
            return self.send(200, self.service.stats())
        self.send(404, {"error": "not found"})

    def do_PUT(self):
        def put(scene, body):
            self.send(200, {"version": self.service.set_scene(scene, environment(body))})

        self.handle_scene(put, plan=False)

    def do_DELETE(self):
        def delete(scene, _):
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
            self.send(400, {"error": str(e)})

    def read_json(self):
        try:
            data = self.rfile.read(int(self.headers.get("Content-Length", 0)))
            body = json.loads(data)
        except (ValueError, json.JSONDecodeError) as e:
            raise BadRequest("body must be JSON") from e
        if not isinstance(body, dict):
            raise BadRequest("body must be a JSON object")
        return body

    def drain(self):
        length = int(self.headers.get("Content-Length", 0) or 0)
        if length:
            self.rfile.read(length)

    def send(self, code, body=None):
        data = json.dumps(body).encode() if body is not None else b""
        self.send_response(code)
        if body is not None:
            self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def log_message(self, format, *args):
        if self.verbose:
            super().log_message(format, *args)


class Server(ThreadingHTTPServer):
    daemon_threads = True
    request_queue_size = 1024  # pending connections; load tests open many at once


def make_server(service, host="127.0.0.1", port=8000, verbose=False):
    """An HTTP server for a coven.panda.Service; port 0 picks a free one (server.server_address)."""
    handler = type("BoundHandler", (Handler,), {"service": service, "verbose": verbose})
    return Server((host, port), handler)


def main():
    parser = argparse.ArgumentParser(description="Coven planning service")
    parser.add_argument("--nodes", type=int, default=10000, help="roadmap size")
    parser.add_argument("--threads", type=int, default=0, help="0: all cores")
    parser.add_argument("--window-us", type=int, default=0, help="batching window of an idle service")
    parser.add_argument("--eager", action="store_true", help="recheck every edge when a scene changes")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8000)
    parser.add_argument("--verbose", action="store_true", help="log every request")
    args = parser.parse_args()

    roadmap = build_roadmap(args.nodes, START, GOAL, threads=args.threads or None)
    service = _core.panda.Service(roadmap, threads=args.threads, window_us=args.window_us, eager=args.eager)
    server = make_server(service, args.host, args.port, args.verbose)
    stats = service.stats()
    print(f"serving {stats['vertices']} vertices with {stats['threads']} threads on "
          f"http://{args.host}:{server.server_address[1]}", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
