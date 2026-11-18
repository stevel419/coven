#!/usr/bin/env bash
#SBATCH -p instruction
#SBATCH -t 0-00:30:00
#SBATCH -J coven
#SBATCH -c 16
#SBATCH -o bench/results/slurm-%j.out -e bench/results/slurm-%j.err
#
# Build Coven and run one benchmark sweep on an Euler compute node.
# Submit from the repository root, one sweep per job:
#
#   sbatch bench/euler.sbatch scaling
#   sbatch bench/euler.sbatch sizes
#   sbatch bench/euler.sbatch revalidate
#
# Every requested core is used: the sweeps run up to $SLURM_CPUS_PER_TASK threads. To use a
# different core count, pass it at submission: sbatch -c 32 bench/euler.sbatch scaling
#
# The job does not ask for an exclusive node, so other jobs can share it. Each configuration is
# repeated and the plots use the median; rerun a sweep if its spread is large.
set -euo pipefail

sweep=${1:?usage: sbatch bench/euler.sbatch <scaling|sizes|revalidate|quick>}

# Toolchain: GCC 11 or newer and CMake 3.16 or newer. Change the module names to match the output
# of `module avail gcc cmake` on Euler.
module load gcc/13.2.0 cmake || true
g++ --version | head -n 1
cmake --version | head -n 1
if [ "$(g++ -dumpversion | cut -d. -f1)" -lt 11 ]; then
    echo "g++ is too old; load a GCC 11+ module in bench/euler.sbatch" >&2
    exit 1
fi

# Built on the compute node so -march=native matches the CPU the benchmark runs on. CMake downloads
# VAMP's dependencies on the first configure. If compute nodes have no internet access, run
#   CPM_SOURCE_CACHE=$HOME/.cache/CPM cmake -S . -B build/euler
# once on the login node; the job then reuses the downloaded sources.
export CPM_SOURCE_CACHE=$HOME/.cache/CPM
cmake -S . -B build/euler -DCMAKE_BUILD_TYPE=Release
cmake --build build/euler --target coven_bench -j "$SLURM_CPUS_PER_TASK"

lscpu | grep -E "Model name|Socket|Core|Thread|NUMA node\(s\)"

# The parallel planner must reproduce the serial roadmap on this machine before anything is timed
OMP_NUM_THREADS=$SLURM_CPUS_PER_TASK build/euler/coven_bench --check --mode parallel --nodes 5000 \
    --threads "$SLURM_CPUS_PER_TASK" --repeats 0 --warmup 0

bash bench/sweep.sh "$sweep" build/euler/coven_bench "bench/results/euler-$sweep-$SLURM_JOB_ID.csv"
