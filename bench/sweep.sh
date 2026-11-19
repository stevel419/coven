#!/usr/bin/env bash
# Run one benchmark sweep and append its rows to a CSV file.
#
#   bench/sweep.sh <scaling|sizes|revalidate|quick> <path to coven_bench> <out.csv>
#
# scaling     threads x environment, and threads x schedule       (figures 1, 2, 3)
# sizes       batch size x roadmap size at the full thread count  (figure 4)
# revalidate  full recheck of a roadmap, threads x roadmap size, and threads x schedule (figure 5)
# quick       a few small runs of every kind, to test the pipeline
#
# The thread counts are 1, 2, 4, ... up to the cores of the Slurm allocation (or of this machine).
# REPEATS sets the timed runs per configuration (default 20); each configuration adds one warm-up.
set -euo pipefail

sweep=$1
bench=$2
out=$3
repeats=${REPEATS:-20}

# One thread per core, neighbours first
export OMP_PROC_BIND=close OMP_PLACES=cores

# Threads go up to the physical cores this process may use: OpenMP makes one place per core of the
# allocation, so two hyperthreads of one core count once.
places=$(OMP_DISPLAY_ENV=true "$bench" --mode serial --nodes 10 --repeats 0 --warmup 0 2>&1 >/dev/null |
    grep -m 1 "OMP_PLACES" | grep -o "{" | wc -l)
cores=${SLURM_CPUS_PER_TASK:-$(nproc)}
if [ "$places" -gt 0 ] && [ "$places" -lt "$cores" ]; then
    echo "using $places threads at most: one per physical core of the $cores CPUs allocated" >&2
    cores=$places
fi
threads=()
for ((t = 1; t < cores; t *= 2)); do threads+=("$t"); done
threads+=("$cores")

here=$(cd "$(dirname "$0")" && pwd)
commit=$(git -C "$here" rev-parse --short HEAD 2>/dev/null || echo unknown)

mkdir -p "$(dirname "$out")"
[ -s "$out" ] || "$bench" --header --mode serial --nodes 10 --repeats 0 --warmup 0 > "$out"

run() {
    echo "coven_bench $*" >&2
    "$bench" --commit "$commit" "$@" >> "$out"
}

case $sweep in
scaling)
    for env in empty cage spheres64; do
        run --mode serial --env "$env" --nodes 10000 --repeats "$repeats"
        for t in "${threads[@]}"; do
            run --mode parallel --env "$env" --nodes 10000 --threads "$t" --repeats "$repeats"
        done
    done
    for t in "${threads[@]}"; do
        for schedule in "static 0" "dynamic 0" "dynamic 16"; do
            set -- $schedule
            run --mode parallel --env cage --nodes 10000 --threads "$t" --schedule "$1" --chunk "$2" \
                --repeats "$repeats"
        done
    done
    ;;
sizes)
    for nodes in 100 300 1000 5000 20000 100000; do
        r=$repeats
        [ "$nodes" -ge 100000 ] && r=$(((repeats + 3) / 4))
        run --mode serial --nodes "$nodes" --repeats "$r"
        run --mode parallel --nodes "$nodes" --threads 1 --repeats "$r"
        for batch in 1 16 64 256 1024 4096 16384; do
            run --mode parallel --nodes "$nodes" --threads "$cores" --batch "$batch" --repeats "$r"
        done
    done
    ;;
revalidate)
    for nodes in 5000 20000 100000; do
        r=$repeats
        [ "$nodes" -ge 100000 ] && r=$(((repeats + 3) / 4))
        for t in "${threads[@]}"; do
            run --mode revalidate --nodes "$nodes" --threads "$t" --repeats "$r"
        done
    done
    for t in "${threads[@]}"; do
        for schedule in "static 0" "guided 0" "dynamic 16"; do
            set -- $schedule
            run --mode revalidate --nodes 20000 --threads "$t" --schedule "$1" --chunk "$2" --repeats "$repeats"
        done
    done
    ;;
quick)
    run --mode serial --nodes 2000 --repeats 3
    for t in "${threads[@]}"; do
        run --mode parallel --nodes 2000 --threads "$t" --repeats 3
        run --mode parallel --nodes 2000 --threads "$t" --schedule static --repeats 3
        run --mode revalidate --nodes 2000 --threads "$t" --repeats 3
    done
    for nodes in 100 2000; do
        [ "$nodes" = 100 ] && run --mode serial --nodes 100 --repeats 3
        for batch in 1 64 1024; do
            run --mode parallel --nodes "$nodes" --threads "$cores" --batch "$batch" --repeats 3
        done
    done
    ;;
*)
    echo "unknown sweep: $sweep" >&2
    exit 2
    ;;
esac
