#!/usr/bin/env bash
#SBATCH -t 0-00:30:00
#SBATCH -J coven
#SBATCH -c 32
#SBATCH -o bench/results/slurm-%j.out -e bench/results/slurm-%j.err
#
# Build Coven and run one benchmark sweep on a Slurm compute node. Submit from the repository root,
# one sweep per job, with the partition and the compiler modules of your cluster:
#
#   mkdir -p bench/results
#   MODULES="gcc cmake" sbatch -p <partition> bench/slurm.sh <scaling|sizes|revalidate|service|mbm|quick>
#
# service-http also needs the Python package (pip install .) in the job's environment.
#
# -c counts hardware threads. The sweeps run one thread per physical core, so on nodes with two
# hardware threads per core the default -c 32 gives 16 threads. The job does not ask for an
# exclusive node; add --dependency=singleton to run your jobs one at a time.
set -euo pipefail

sweep=${1:?usage: sbatch bench/slurm.sh <scaling|sizes|revalidate|service|service-http|mbm|quick>}

# Toolchain: GCC 11 or newer and CMake 3.16 or newer
if [ -n "${MODULES:-}" ]; then
    module load $MODULES
fi
g++ --version | head -n 1
cmake --version | head -n 1
if [ "$(g++ -dumpversion | cut -d. -f1)" -lt 11 ]; then
    echo "g++ is too old; set MODULES to load GCC 11 or newer" >&2
    exit 1
fi

# Built on the compute node so -march=native matches the CPU the benchmark runs on. CMake downloads
# VAMP and its dependencies on the first configure. If compute nodes have no internet access, run
#   CPM_SOURCE_CACHE=$HOME/.cache/CPM cmake -S . -B build/slurm
# once on the login node; the job then reuses the downloaded sources.
export CPM_SOURCE_CACHE=$HOME/.cache/CPM
cmake -S . -B build/slurm -DCMAKE_BUILD_TYPE=Release
cmake --build build/slurm --target coven_bench coven_load coven_mbm -j "$SLURM_CPUS_PER_TASK"

lscpu | grep -E "Model name|Socket|Core|Thread|NUMA node\(s\)"

# The parallel planner must reproduce the serial roadmap on this machine before anything is timed
OMP_NUM_THREADS=$SLURM_CPUS_PER_TASK build/slurm/coven_bench --check --mode parallel --nodes 5000 \
    --threads "$SLURM_CPUS_PER_TASK" --repeats 0 --warmup 0

bash bench/sweep.sh "$sweep" build/slurm/coven_bench "bench/results/$sweep-$SLURM_JOB_ID.csv"
