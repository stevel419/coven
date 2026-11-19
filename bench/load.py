"""Load test of the HTTP planning service: coven_load's open-loop test, over HTTP.

    coven-server --nodes 10000 --threads 16 --port 8000 &
    python bench/load.py --url http://127.0.0.1:8000 --rate 500 [--scenes 1] [--update-ms 100]
                         [--duration 3] [--warmup 1] [--connections 64] [--commit HASH] [--header]

Prints one CSV row with coven_load's columns (transport http). The scenes are the sphere cage, moved
as in coven_load; roadmap size, threads, window and mode are read from the server. Each connection
sends one request at a time, so --connections must exceed rate x latency for the load to stay open
loop; latency still runs from each request's scheduled arrival.
"""

import argparse
import http.client
import itertools
import json
import math
import platform
import random
import threading
import time
from urllib.parse import urlparse

import numpy as np
import vamp

COLUMNS = ("transport,mode,env,nodes,threads,window_us,rate,scenes,update_ms,duration_s,requests,ok,"
           "overloaded,no_path,throughput,p50_ms,p90_ms,p99_ms,p999_ms,mean_ms,rounds,checks,searches,"
           "fallbacks,checks_per_round,update_busy,search_busy,check_busy,commit,compiler,cpu")
CAGE = [
    [0.55, 0, 0.25], [0.35, 0.35, 0.25], [0, 0.55, 0.25], [-0.55, 0, 0.25], [-0.35, -0.35, 0.25],
    [0, -0.55, 0.25], [0.35, -0.35, 0.25], [0.35, 0.35, 0.8], [0, 0.55, 0.8], [-0.35, 0.35, 0.8],
    [-0.55, 0, 0.8], [-0.35, -0.35, 0.8], [0, -0.55, 0.8], [0.35, -0.35, 0.8],
]


def cage_body(shift):
    return {"spheres": [{"center": [x + shift, y, z], "radius": 0.2} for x, y, z in CAGE]}


def scene_at(s, scenes, t):
    return cage_body(0.1 * math.sin(2 * math.pi * (t / 2 + s / scenes)))


def configurations(n):
    """Collision-free configurations in the unmoved cage, as in coven_load."""
    environment = vamp.Environment()
    for center in CAGE:
        environment.add_sphere(vamp.Sphere(center, 0.2))
    rng = vamp.panda.halton()
    rng.skip(777)
    result = []
    while len(result) < n:
        c = rng.next()
        if vamp.panda.validate(c, environment):
            result.append(c.tolist())
    return result


def cpu_model():
    try:
        with open("/proc/cpuinfo") as f:
            for line in f:
                if line.startswith("model name"):
                    return line.split(":", 1)[1].strip().replace(",", " ")
    except OSError:
        pass
    return platform.processor() or "unknown"


def percentile(values, p):
    if not values:
        return 0.0
    values = sorted(values)
    return values[min(len(values) - 1, int(p * len(values)))]


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--url", default="http://127.0.0.1:8000")
    parser.add_argument("--rate", type=float, default=500)
    parser.add_argument("--scenes", type=int, default=1)
    parser.add_argument("--update-ms", type=int, default=100)
    parser.add_argument("--duration", type=float, default=3)
    parser.add_argument("--warmup", type=float, default=1)
    parser.add_argument("--connections", type=int, default=64)
    parser.add_argument("--commit", default="unknown")
    parser.add_argument("--header", action="store_true")
    args = parser.parse_args()
    if args.header:
        print(COLUMNS)
        return

    url = urlparse(args.url)

    def connect():
        return http.client.HTTPConnection(url.hostname, url.port, timeout=60)

    def call(connection, method, path, body=None):
        connection.request(method, path, body=None if body is None else json.dumps(body),
                           headers={"Content-Type": "application/json"})
        response = connection.getresponse()
        data = response.read()
        return response.status, json.loads(data) if data else None

    control = connect()
    names = [f"s{s}" for s in range(args.scenes)]
    for s, name in enumerate(names):
        call(control, "PUT", f"/scenes/{name}", scene_at(s, args.scenes, 0))
    pool = configurations(512)
    _, before = call(control, "GET", "/stats")

    # Open-loop arrivals, fixed seed
    random_ = random.Random(759)
    arrivals, t = [], random_.expovariate(args.rate)
    while t < args.warmup + args.duration:
        arrivals.append((t, names[random_.randrange(args.scenes)], random_.choice(pool), random_.choice(pool)))
        t += random_.expovariate(args.rate)

    t0 = time.perf_counter()
    stop = threading.Event()
    results = [None] * len(arrivals)
    next_index = itertools.count()

    def worker():
        connection = connect()
        for i in next_index:
            if i >= len(arrivals):
                return
            scheduled, scene, start, goal = arrivals[i]
            delay = t0 + scheduled - time.perf_counter()
            if delay > 0:
                time.sleep(delay)
            status, body = call(connection, "POST", f"/scenes/{scene}/plan", {"start": start, "goal": goal})
            results[i] = (scheduled, time.perf_counter() - t0, status, body)

    def updater():
        updates = connect()
        tick = t0
        while not stop.is_set():
            tick += args.update_ms / 1e3
            time.sleep(max(0.0, tick - time.perf_counter()))
            for s, name in enumerate(names):
                call(updates, "PUT", f"/scenes/{name}", scene_at(s, args.scenes, time.perf_counter() - t0))

    workers = [threading.Thread(target=worker) for _ in range(args.connections)]
    update_thread = threading.Thread(target=updater, daemon=True) if args.update_ms > 0 else None
    for thread in workers + ([update_thread] if update_thread else []):
        thread.start()
    for thread in workers:
        thread.join()
    stop.set()
    if update_thread:
        update_thread.join()
    elapsed = time.perf_counter() - t0
    _, after = call(control, "GET", "/stats")

    measured = [r for r in results if r[0] >= args.warmup]
    good = [r for r in measured if r[2] == 200]
    latencies = [1e3 * (done - scheduled) for scheduled, done, _, _ in good]
    per = lambda key: float(np.mean([r[3][key] for r in good])) if good else 0.0  # noqa: E731
    busy = lambda key: (after[key] - before[key]) / elapsed  # noqa: E731
    rounds = after["rounds"] - before["rounds"]
    row = [
        "http", "eager" if after["eager"] else "lazy", "cage", after["vertices"], after["threads"],
        after["window_us"], args.rate, args.scenes, args.update_ms, args.duration, len(measured), len(good),
        sum(r[2] == 503 for r in measured), sum(not r[3]["path"] for r in good),
        len(good) / (max([args.warmup + args.duration] + [r[1] for r in good]) - args.warmup),
        percentile(latencies, 0.5), percentile(latencies, 0.9), percentile(latencies, 0.99),
        percentile(latencies, 0.999), float(np.mean(latencies)) if latencies else 0.0, per("rounds"),
        per("checks"), per("searches"), sum(r[3]["fallback"] for r in good), (after["checks"] - before["checks"]) / rounds if rounds else 0.0,
        busy("update_seconds"), busy("search_seconds"), busy("check_seconds"), args.commit,
        f"python {platform.python_version()}", cpu_model(),
    ]
    print(",".join(str(v) for v in row))


if __name__ == "__main__":
    main()
