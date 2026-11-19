# Coven

Multi-core PRM roadmap construction and replanning for [VAMP](https://github.com/KavrakiLab/vamp).

![Serial and parallel replanning side by side](demo/recording.gif)

*Two Panda arms replan among moving obstacles on the same 5,000-node roadmap. Left: one thread.
Right: 11 threads. The faster arm stops following blocked paths sooner.*

VAMP makes collision checking fast by checking eight poses along one edge at once with SIMD, on a
single core. Its PRM still checks roadmap edges one after another, and edge checking is most of the
time a PRM spends. Coven checks many edges at once across CPU cores with OpenMP. The roadmap it builds
is identical to VAMP's serial PRM: the same vertices, edges and sampler state, for any thread count.

Once built, a roadmap can be reused as obstacles move. Coven rechecks every vertex and edge against
the new environment in parallel, then searches the part that is still valid. That makes the replan
rate depend directly on the number of cores, which the demo above shows.

## How it works

`coven::PPRM::build_roadmap()` (in [`include/coven/pprm.hh`](include/coven/pprm.hh)) works through
the valid samples in batches. For each batch it:

1. draws samples in generator order, checks them in parallel, and keeps the first `B` valid ones;
2. finds each new node's neighbors among the existing roadmap nodes and the earlier nodes of the
   same batch, with `k` and `r` computed for the roadmap size the serial planner would see at that
   node;
3. checks every candidate edge of the batch in one `#pragma omp parallel for`;
4. inserts the nodes and their valid edges serially, in the serial planner's order.

Only step 3 and the sample checks run in parallel, and each edge is checked exactly as the serial
planner checks it, so the result does not depend on the batch size, thread count or OpenMP schedule.

- `revalidate()` rechecks every vertex and edge of a roadmap in parallel. A blocked vertex
  invalidates its edges without checking them.
- `query()` connects the start and goal to their nearest valid vertices and runs A* over the valid
  edges.

Two properties of VAMP make this safe:

- VAMP's generated `fkcc` collision functions only read the environment, so threads can share one
  environment. `fkcc_attach`, used when an object is attached to the gripper, writes to the
  environment, so the loops run on one thread when the environment has an attachment.
- `nigh`'s `nearest()` is `const` and keeps its search state local, so threads can query the
  neighbor tree at once as long as nobody inserts. Step 2 relies on this; step 4 is the only writer.

The collision-checking loops use the OpenMP schedules that were fastest in the benchmarks: `guided`
for construction and `dynamic` for revalidation. Define `COVEN_SCHEDULE=runtime` when compiling to
choose the schedule with `OMP_SCHEDULE` instead.

## Install

Linux or WSL2 with an AVX2 CPU, GCC 11+ or Clang, CMake 3.16+ and Python 3.10+. VAMP itself is not
modified: CMake downloads it at a pinned commit, together with its dependencies.

C++ (header-only library, tests and benchmark):

```bash
git clone https://github.com/stevel419/coven.git && cd coven
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
ctest --test-dir build --output-on-failure
```

Python (installs VAMP's Python package from the same commit, then Coven's bindings):

```bash
pip install ".[demo,bench,test]"
pytest
```

Run `pytest` rather than `python -m pytest`: the latter puts the source folder first on the import
path, and that folder does not contain the compiled `coven._core`.

If linking fails with undefined `Catch::` or `GOMP_` symbols under a conda compiler, pass
`-DCMAKE_AR=$(which $GCC_AR) -DCMAKE_RANLIB=$(which $GCC_RANLIB)`. VAMP builds with LTO, and LTO
archives need the compiler's own `ar`.

## Usage

```python
import numpy as np
import vamp
import coven

start = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785], dtype=np.float32)
goal = np.array([2.35, 1.0, 0.0, -0.8, 0.0, 2.5, 0.785], dtype=np.float32)

settings = vamp.PRMSettings(vamp.PRMNeighborParams(vamp.panda.dimension(), vamp.panda.space_measure()))
settings.max_samples = 10_000
settings.max_iterations = 10_000_000

# Build once, in free space
coven.set_num_threads(8)
roadmap = coven.panda.build_roadmap(start, goal, vamp.Environment(), settings, vamp.panda.halton())
# The same roadmap as vamp.panda.roadmap(start, goal, vamp.Environment(), settings, vamp.panda.halton())

# Replan on it whenever the obstacles change
environment = vamp.Environment()
environment.add_sphere(vamp.Sphere([0.4, 0.4, 0.6], 0.2))
graph = coven.panda.graph(roadmap)
validity = coven.panda.revalidate(roadmap, graph, environment)
path = coven.panda.query(roadmap, graph, validity, start, goal, environment)  # empty if there is none
```

Configurations are `float32` numpy arrays. The roadmap and path are VAMP's own Python types.
`coven.Replanner` wraps the last three calls for the Panda. Bindings exist for all of VAMP's robots: `coven.panda`, `ur5`, `fetch`, `baxter` and
`sphere`.

## Demo

```bash
python demo/demo.py --nodes 5000 --obstacles 10
```

Open http://localhost:8080. On first run the demo downloads VAMP's Panda model (about 9 MB).

Two Panda arms move between two poses while spheres orbit them. Each arm keeps a roadmap built once
in free space. As fast as it can, it rechecks the roadmap against the current obstacles and searches
for a path from where the arm is now. The arm follows its latest path (green) at a fixed joint
speed. Gray lines are a sample of the valid roadmap edges and red points are blocked vertices, both
drawn at the end-effector position.

- **Left arm:** replans with one thread, on a roadmap from VAMP's serial PRM.
- **Right arm:** uses Coven with the number of threads set in the panel.

Both roadmaps are identical, and the panel checks this after every build. The panel also shows:

- replans per second;
- edges checked per second;
- how often the arm is moving along a path segment that the obstacles have already blocked.

A lane that replans slowly follows older paths. You can also drag the obstacles by hand.

The arms do not dodge: an obstacle can still sweep into an arm, whatever the replan rate. The replan
rate only changes how soon an arm stops following a path that is no longer free.

## Benchmarks

`bench/bench.cc` builds `coven_bench`, which prints one CSV row per timed run: the configuration,
wall time, time per phase (sampling, neighbor search, edge validation, insertion), edges checked,
roadmap size, commit, compiler and CPU.

- **Baseline:** VAMP's unmodified `PRM::build_roadmap()`, compiled with the same compiler and flags.
- **Problem:** the Panda with the start and goal of VAMP's sphere cage example, in three
  environments: `empty`; `cage`, VAMP's 14 spheres of radius 0.2; `spheres64`, 64 spheres of
  radius 0.06 at fixed random positions.
- **Timing:** one warm-up and 20 timed runs per configuration (5 at 100,000 nodes), with the same
  sampler seed every run. Figures and tables use the median.
- **Threads:** one per physical core (`OMP_PROC_BIND=close`, `OMP_PLACES=cores`), in powers of two
  up to the number of cores available.

| Sweep | What varies | Figures |
|---|---|---|
| `scaling` | threads × environment, and threads × schedule. 10,000 nodes, batch 1024 | speedup, phase breakdown, schedule |
| `sizes` | batch size (1 to 16,384) × roadmap size (100 to 100,000), all cores | batch size |
| `revalidate` | threads × roadmap size (5,000 to 100,000), and threads × schedule at 20,000 nodes | revalidation, schedule |

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j --target coven_bench
build/coven_bench --check --mode parallel --nodes 5000 --threads 8 --repeats 0 --warmup 0
for sweep in scaling sizes revalidate; do bash bench/sweep.sh $sweep build/coven_bench bench/results/$sweep.csv; done
python bench/plot.py bench/results/*.csv --out bench/results/figures
```

`--check` fails unless the parallel roadmap equals the serial one. `REPEATS=5` in front of
`sweep.sh` shortens a sweep. `plot.py` writes the figures and `summary.txt`, the numbers behind them.

On a Slurm cluster, `bench/slurm.sh` builds on the compute node, so `-march=native` matches the CPU,
runs the check, then runs one sweep:

```bash
mkdir -p bench/results
MODULES="gcc cmake" sbatch -p <partition> --dependency=singleton bench/slurm.sh scaling
```

## Results

Pending.

## Layout

| Path | Contents |
|---|---|
| `include/coven/pprm.hh` | The parallel planner: header-only C++17 with OpenMP |
| `src/python.cc` | Python bindings (`coven._core`) |
| `coven/` | Python package, including `Replanner` for replanning on a fixed roadmap |
| `tests/` | C++ (Catch2) and Python tests |
| `bench/` | Benchmark driver, sweeps, plots and results |
| `demo/` | `viser` demo |

## Acknowledgements

Coven is built on [VAMP](https://github.com/KavrakiLab/vamp) (Apache-2.0) by Zachary Kingston, Wil
Thomason, Clayton Ramsey and the Kavraki Lab:

> W. Thomason, Z. Kingston, and L. E. Kavraki. "Motions in Microseconds via Vectorized
> Sampling-Based Planning." *IEEE International Conference on Robotics and Automation (ICRA)*, 2024.
