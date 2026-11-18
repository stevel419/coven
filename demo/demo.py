"""Live replanning demo: a Panda arm moves between two poses while obstacles move around it.

    python demo/demo.py [--nodes 5000] [--obstacles 10] [--port 8080]

then open http://localhost:8080. Each lane keeps a roadmap built once in free space; as fast as it
can, it rechecks every roadmap edge against the current obstacles and searches the valid part for a
path from where the arm is now. The arm follows the most recent path at a fixed joint speed.

Side by side, the left lane replans with one thread and its roadmap comes from VAMP's unmodified
serial PRM; the right lane uses the threads set in the panel and the parallel planner. Both
roadmaps are the same. A lane whose replans take longer follows older paths; the panel shows how
often the arm is moving along a path segment that the obstacles have already blocked.
"""

import argparse
import logging
import threading
import time
import urllib.request
from collections import deque
from pathlib import Path

import numpy as np
import viser
import vamp
import yourdfpy
from viser.extras import ViserUrdf

import coven
from coven import Replanner, build_roadmap

robot = vamp.panda
# yourdfpy warns on every pose update that the Panda's second finger mimics an unactuated joint
logging.getLogger("yourdfpy").setLevel(logging.ERROR)
POSES = [
    np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785], dtype=np.float32),
    np.array([2.35, 1.0, 0.0, -0.8, 0, 2.5, 0.785], dtype=np.float32),
]
# The Panda model comes from the VAMP commit that coven builds against (see CMakeLists.txt)
VAMP_COMMIT = "27cb9b66ebd8d101ce88d330f3b028398afc1cfa"
PANDA_FILES = ["panda_spherized.urdf", "meshes/collision/link0.obj", "meshes/collision/link7.obj"] + [
    f"meshes/visual/{name}.obj" for name in ("finger", "hand", "link1", "link2", "link3", "link4", "link5", "link6")]
JOINT_SPEED = 0.8  # radians per second along the path
MAX_DRAWN_EDGES = 600
LANE_OFFSET = 1.3
VALID, INVALID, PATH = (190, 200, 215), (227, 73, 72), (27, 175, 122)


def panda_urdf():
    """Path to VAMP's Panda URDF, downloading it and its meshes (about 9 MB) on first use."""
    cache = Path.home() / ".cache" / "coven" / VAMP_COMMIT[:12] / "panda"
    for name in PANDA_FILES:
        path = cache / name
        if not path.exists():
            path.parent.mkdir(parents=True, exist_ok=True)
            url = f"https://raw.githubusercontent.com/KavrakiLab/vamp/{VAMP_COMMIT}/resources/panda/{name}"
            urllib.request.urlretrieve(url, path.with_suffix(".part"))
            path.with_suffix(".part").replace(path)
    return cache / "panda_spherized.urdf"


class Obstacles:
    """Spheres on scripted orbits around the arm, or wherever they were dragged."""

    def __init__(self, count):
        rng = np.random.default_rng(759)
        self.orbit = rng.uniform(0.45, 0.75, 32)
        self.height = rng.uniform(0.15, 0.95, 32)
        self.speed = rng.uniform(0.25, 0.6, 32) * rng.choice([-1, 1], 32)
        self.phase = rng.uniform(0, 2 * np.pi, 32)
        self.radius = rng.uniform(0.07, 0.11, 32)
        self.count = count
        self.dragged = None  # positions while dragging, else None

    def positions(self, t):
        if self.dragged is not None:
            return self.dragged
        n = self.count
        angle = self.phase[:n] + self.speed[:n] * t
        z = self.height[:n] + 0.1 * np.sin(1.7 * self.speed[:n] * t)
        return np.stack([self.orbit[:n] * np.cos(angle), self.orbit[:n] * np.sin(angle), z], axis=1)

    def environment(self, positions):
        environment = vamp.Environment()
        for p, r in zip(positions, self.radius):
            environment.add_sphere(vamp.Sphere(list(map(float, p)), float(r)))
        return environment


class Lane:
    """One arm with its own replanning thread."""

    def __init__(self, app, name, label, offset):
        self.app = app
        self.name = name
        self.replanner = None
        self.threads = 1
        self.enabled = True
        self.q = POSES[0].copy()
        self.goal = 1
        self.path = np.empty((0, 7), dtype=np.float32)
        self.path_version = 0
        self.validity = None
        self.history = deque()  # (finish time, seconds, revalidate seconds, edges)
        self.blocked = deque(maxlen=300)  # per frame while moving: was the next path segment blocked?
        self.lock = threading.Lock()

        server = app.server
        self.frame = server.scene.add_frame(f"/{name}", show_axes=False, position=(0, offset, 0))
        server.scene.add_label(f"/{name}/label", label, position=(0, 0, 1.35), anchor="bottom-center")
        self.urdf = ViserUrdf(server, yourdfpy.URDF.load(str(panda_urdf())), root_node_name=f"/{name}/robot")
        self.urdf.update_cfg(self.q)
        self.obstacle_handles = []
        threading.Thread(target=self.loop, daemon=True).start()

    def loop(self):
        while True:
            replanner = self.replanner
            if replanner is None or not self.enabled:
                time.sleep(0.05)
                continue
            replanner.threads = self.threads
            with self.lock:
                start, goal = self.q.copy(), POSES[self.goal]
            positions = self.app.snapshot()
            step = replanner.replan(start, goal, self.app.obstacles.environment(positions))
            now = time.perf_counter()
            with self.lock:
                if replanner is not self.replanner:
                    continue  # the roadmap was replaced while this replan ran
                # Follow the new path from wherever the arm got to meanwhile
                self.path = np.vstack([self.q, step.path[1:]]) if len(step.path) else step.path
                self.path_version += 1
                self.validity = step.validity
                self.history.append((now, step.seconds, step.revalidate_seconds, step.edges))
                while self.history and now - self.history[0][0] > 2.0:
                    self.history.popleft()

    def advance(self, dt, environment):
        with self.lock:
            budget = JOINT_SPEED * dt
            while budget > 0 and len(self.path) > 1:
                to_next = self.path[1] - self.q
                distance = float(np.linalg.norm(to_next))
                if distance <= budget:
                    self.q = self.path[1].copy()
                    self.path = self.path[1:]
                    budget -= distance
                else:
                    self.q = self.q + to_next * (budget / distance)
                    budget = 0
            if np.linalg.norm(self.q - POSES[self.goal]) < 1e-3:
                self.goal = 1 - self.goal
                self.path = np.empty((0, 7), dtype=np.float32)
            q = self.q.copy()
            next_waypoint = self.path[1].copy() if len(self.path) > 1 else None

        # Is the stretch the arm is about to move along still free in the scene as it is now?
        if next_waypoint is not None:
            segment = robot.Path()
            segment.append(q)
            segment.append(next_waypoint)
            self.blocked.append(not segment.validate(environment))
        self.urdf.update_cfg(q)

    def stats(self):
        with self.lock:
            history = list(self.history)
        if not history:
            return "waiting for the first replan"
        rate = len(history) / 2.0
        seconds = np.mean([h[1] for h in history])
        revalidate = np.mean([h[2] for h in history])
        edges = history[-1][3]
        return (f"**{self.threads} thread{'s' if self.threads > 1 else ''}**  \n"
                f"replans/s: **{rate:.1f}** ({1e3 * seconds:.1f} ms each)  \n"
                f"edges checked/s: **{edges / revalidate / 1e6:.2f} M** ({edges:,} per replan)  \n"
                f"moving on a blocked path: **{100 * np.mean(self.blocked or [0]):.0f}%** of the time")


class App:
    def __init__(self, args):
        self.server = viser.ViserServer(port=args.port)
        self.server.scene.set_up_direction("+z")
        self.server.initial_camera.position = (3.4, 0.0, 1.8)
        self.server.initial_camera.look_at = (0.0, 0.0, 0.4)
        self.ee = np.empty((0, 3))  # end-effector position of every roadmap vertex
        self.obstacles = Obstacles(args.obstacles)
        self.positions = self.obstacles.positions(0.0)
        self.positions_lock = threading.Lock()
        self.start_time = time.perf_counter()
        self.drag_handles = []
        self.drawn = {}  # lane name -> (path version, edges drawn, end-effector positions)

        gui = self.server.gui
        self.side_by_side = gui.add_checkbox("Side by side with serial", True)
        self.show_roadmap = gui.add_checkbox("Show roadmap", True)
        self.threads = gui.add_slider("Threads (parallel lane)", 1, coven.max_threads(), 1,
                                      max(coven.max_threads() - 1, 1))
        self.count = gui.add_slider("Obstacles", 1, 32, 1, args.obstacles)
        self.drag = gui.add_checkbox("Drag obstacles by hand", False)
        self.nodes = gui.add_dropdown("Roadmap nodes", ["1000", "5000", "20000", "50000", "100000"],
                                      initial_value=str(args.nodes))
        self.rebuild = gui.add_button("Rebuild roadmaps")
        self.build_info = gui.add_markdown("")
        gui.add_markdown("**Serial lane** (left)")
        self.serial_stats = gui.add_markdown("")
        gui.add_markdown("**Parallel lane** (right)")
        self.parallel_stats = gui.add_markdown("")

        self.serial = Lane(self, "serial", "serial PRM, 1 thread", -LANE_OFFSET)
        self.parallel = Lane(self, "parallel", "parallel PRM", LANE_OFFSET)
        self.rebuild.on_click(lambda _: threading.Thread(target=self.build, daemon=True).start())
        self.build()

    def snapshot(self):
        with self.positions_lock:
            return self.positions.copy()

    def build(self):
        """Build the serial lane's roadmap with VAMP's PRM and the parallel lane's with PPRM."""
        self.rebuild.disabled = True
        nodes = int(self.nodes.value)
        self.build_info.content = f"building {nodes:,}-node roadmaps..."
        # Pause both lanes so the two builds are timed on an idle machine
        for lane in (self.serial, self.parallel):
            with lane.lock:
                lane.replanner = None
        t = time.perf_counter()
        serial = build_roadmap(nodes, POSES[0], POSES[1], parallel=False)
        serial_seconds = time.perf_counter() - t
        t = time.perf_counter()
        parallel = build_roadmap(nodes, POSES[0], POSES[1], parallel=True, threads=self.threads.value)
        parallel_seconds = time.perf_counter() - t
        same = np.array_equal(coven.panda.vertices(serial), coven.panda.vertices(parallel)) and \
            [sorted(e) for e in serial.edges] == [sorted(e) for e in parallel.edges]

        ee = np.array([robot.eefk(v)[:3, 3] for v in coven.panda.vertices(parallel)])
        self.ee = ee
        for lane, roadmap in ((self.serial, serial), (self.parallel, parallel)):
            replanner = Replanner(roadmap)
            edges = replanner.graph.edges
            drawn = np.linspace(0, len(edges) - 1, min(MAX_DRAWN_EDGES, len(edges))).astype(int)
            self.drawn[lane.name] = (-1, drawn, ee[edges[drawn]])
            with lane.lock:
                lane.replanner = replanner
                lane.history.clear()
                lane.path = np.empty((0, 7), dtype=np.float32)
        self.build_info.content = (
            f"{nodes:,} nodes: serial build {serial_seconds:.2f} s, parallel build {parallel_seconds:.2f} s "
            f"with {self.threads.value} threads; roadmaps identical: **{'yes' if same else 'NO'}**")
        self.rebuild.disabled = False

    def set_drag(self, on):
        """Put a drag handle on each obstacle (in the parallel lane), starting where it is now."""
        for handle in self.drag_handles:
            handle.remove()
        self.drag_handles = []
        self.obstacles.dragged = None
        if on:
            positions = self.obstacles.positions(time.perf_counter() - self.start_time)
            self.obstacles.dragged = positions
            for i, p in enumerate(positions):
                self.drag_handles.append(self.server.scene.add_transform_controls(
                    f"/parallel/drag/{i}", scale=0.2, disable_rotations=True, position=tuple(p)))

    def draw(self, lane, positions):
        handles = lane.obstacle_handles
        while len(handles) > len(positions):
            handles.pop().remove()
        while len(handles) < len(positions):
            i = len(handles)
            handles.append(self.server.scene.add_icosphere(
                f"/{lane.name}/obstacles/{i}", float(self.obstacles.radius[i]), (235, 104, 52)))
        for handle, p in zip(handles, positions):
            handle.position = tuple(p)

        version, drawn, segments = self.drawn.get(lane.name, (0, None, None))
        if drawn is None or lane.path_version == version:
            return
        with lane.lock:
            validity, path, lane_version = lane.validity, lane.path.copy(), lane.path_version
        if validity is None:
            return
        self.drawn[lane.name] = (lane_version, drawn, segments)
        # Blocked vertices as red points at their end-effector positions, and a sample of valid edges
        show = self.show_roadmap.value
        blocked = self.ee[~validity.vertices] if len(validity.vertices) == len(self.ee) else self.ee[:0]
        self.server.scene.add_point_cloud(f"/{lane.name}/blocked", blocked.astype(np.float32) if len(blocked)
                                          else np.zeros((1, 3), np.float32), INVALID, point_size=0.006,
                                          visible=show and len(blocked) > 0)
        valid = segments[validity.edges[drawn]]
        if len(valid):
            self.server.scene.add_line_segments(f"/{lane.name}/roadmap", valid, VALID, thickness=1,
                                                thickness_units="screen", visible=show)
        if len(path) > 1:
            ee = np.array([robot.eefk(q)[:3, 3] for q in path])
            self.server.scene.add_line_segments(f"/{lane.name}/path", np.stack([ee[:-1], ee[1:]], axis=1),
                                                PATH, thickness=4, thickness_units="screen")

    def run(self):
        last = time.perf_counter()
        while True:
            now = time.perf_counter()
            dt, last = now - last, now
            self.obstacles.count = self.count.value
            if self.drag.value != bool(self.drag_handles) or \
                    (self.drag.value and len(self.drag_handles) != self.count.value):
                self.set_drag(self.drag.value)
            if self.drag.value:
                self.obstacles.dragged = np.array([h.position for h in self.drag_handles])
            positions = self.obstacles.positions(now - self.start_time)
            with self.positions_lock:
                self.positions = positions
            environment = self.obstacles.environment(positions)

            self.parallel.threads = self.threads.value
            self.serial.enabled = self.side_by_side.value
            self.serial.frame.visible = self.side_by_side.value
            for lane in (self.serial, self.parallel):
                if lane.enabled:
                    lane.advance(dt, environment)
                    self.draw(lane, positions)
            self.serial_stats.content = self.serial.stats() if self.serial.enabled else "off"
            self.parallel_stats.content = self.parallel.stats()
            time.sleep(max(0.0, 1 / 30 - (time.perf_counter() - now)))


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--nodes", type=int, default=5000, choices=[1000, 5000, 20000, 50000, 100000])
    parser.add_argument("--obstacles", type=int, default=10)
    parser.add_argument("--port", type=int, default=8080)
    App(parser.parse_args()).run()


if __name__ == "__main__":
    main()
