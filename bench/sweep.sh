#!/usr/bin/env bash
# Run one benchmark sweep and append its rows to a CSV file.
#
#   bench/sweep.sh <sweep> <path to coven_bench> <out.csv>
#
# scaling       threads x environment, and threads x schedule       (figures 1, 2, 3)
# sizes         batch size x roadmap size at the full thread count  (figure 4)
# revalidate    full recheck of a roadmap, threads x roadmap size, and threads x schedule (figure 5)
# service       planning service load tests with coven_load (next to coven_bench)  (figures 6, 7, 8)
# service-http  the same service over HTTP with bench/load.py, and in process     (figure 9)
# mbm           MotionBenchMaker: construction and revalidation per scenario  (figure 10)
# mbm-problems  every MotionBenchMaker problem with VAMP's RRT-Connect and with the service, using
#               coven_mbm next to coven_bench                                  (figures 11, 12)
#               (needs the Python package installed: pip install .)
# quick         a few small runs of every kind, to test the pipeline
#
# The thread counts are 1, 2, 4, ... up to the cores of the Slurm allocation (or of this machine).
# REPEATS sets the timed runs per configuration (default 20); each configuration adds one warm-up.
# DURATION sets the measured seconds of each load test (default 3), after one second of warm-up.
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
load=$(dirname "$bench")/coven_load
mbm=$(dirname "$bench")/coven_mbm
case $sweep in
service*) [ -s "$out" ] || "$load" --header > "$out" ;;
mbm-problems) [ -s "$out" ] || "$mbm" --header > "$out" ;;
*) [ -s "$out" ] || "$bench" --header --mode serial --nodes 10 --repeats 0 --warmup 0 > "$out" ;;
esac

run() {
    echo "coven_bench $*" >&2
    "$bench" --commit "$commit" "$@" >> "$out"
}

# The service's threads are not pinned: the load generator and the scene updater share the cores
duration=${DURATION:-3}
load_run() {
    echo "coven_load $*" >&2
    env -u OMP_PROC_BIND -u OMP_PLACES "$load" --commit "$commit" --duration "$duration" --warmup 1 "$@" >> "$out"
}

rates="100 300 1000 3000 10000 30000"

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
service)
    # Latency against throughput, by batching window (lazy, one scene moving every second)
    for window in 0 250 1000 4000; do
        for rate in $rates; do
            load_run --threads "$cores" --window-us "$window" --update-ms 1000 --rate "$rate"
        done
    done
    # One shared scene against 16 separate ones, by thread count, lazy and eager
    for mode in lazy eager; do
        for t in $(printf '%s\n' 1 4 "$cores" | sort -nu); do
            for scenes in 1 16; do
                [ "$t" = "$cores" ] && [ "$scenes" = 1 ] && continue  # run below
                for rate in $rates; do
                    load_run --mode "$mode" --threads "$t" --scenes "$scenes" --update-ms 1000 --rate "$rate"
                done
            done
        done
    done
    # Lazy against eager checking, by how often the obstacles move (0: never)
    for mode in lazy eager; do
        for update in 10 100 1000 0; do
            [ "$mode" = lazy ] && [ "$update" = 1000 ] && continue
            for rate in $rates; do
                load_run --threads "$cores" --mode "$mode" --update-ms "$update" --rate "$rate"
            done
        done
    done
    ;;
mbm)
    # MotionBenchMaker: construction and revalidation in each scenario's first scene, as above
    r=$(((repeats + 1) / 2))
    for scenario in bookshelf_small bookshelf_tall bookshelf_thin box cage table_pick table_under_pick; do
        run --mode serial --env "mbm:$scenario" --nodes 10000 --repeats "$r"
        for t in "${threads[@]}"; do
            run --mode parallel --env "mbm:$scenario" --nodes 10000 --threads "$t" --repeats "$r"
            run --mode revalidate --env "mbm:$scenario" --nodes 10000 --threads "$t" --repeats "$r"
        done
    done
    ;;
mbm-problems)
    # Every MotionBenchMaker problem with VAMP's RRT-Connect and with the planning service (coven_mbm)
    mbm_run() {
        echo "coven_mbm $*" >&2
        "$mbm" --commit "$commit" "$@" >> "$out"
    }
    mbm_run --method rrtc
    for nodes in 5000 10000 20000 50000; do
        mbm_run --method lazy --nodes "$nodes" --threads "$cores"
    done
    mbm_run --method lazy --nodes 10000 --connections 8 --threads "$cores"
    mbm_run --method lazy --nodes 10000 --threads 1
    mbm_run --method eager --nodes 10000 --threads "$cores"
    ;;
service-http)
    port=${PORT:-$((20000 + RANDOM % 20000))}
    env -u OMP_PROC_BIND -u OMP_PLACES coven-server --nodes 10000 --threads "$cores" --port "$port" >&2 &
    server=$!
    trap 'kill $server 2>/dev/null' EXIT
    ready=0
    for _ in $(seq 120); do
        if python -c "import urllib.request; urllib.request.urlopen('http://127.0.0.1:$port/stats')" 2>/dev/null; then
            ready=1
            break
        fi
        sleep 1
    done
    if [ "$ready" = 0 ]; then
        echo "coven-server did not start" >&2
        exit 1
    fi
    for rate in 100 300 1000 3000; do
        echo "load.py --rate $rate" >&2
        python "$here/load.py" --url "http://127.0.0.1:$port" --rate "$rate" --update-ms 1000 \
            --duration "$duration" --commit "$commit" >> "$out"
        load_run --threads "$cores" --update-ms 1000 --rate "$rate"
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
    # One short load test of the service, printed rather than written (its columns differ)
    env -u OMP_PROC_BIND -u OMP_PLACES "$load" --threads "$cores" --nodes 2000 --rate 300 --duration 1 \
        --warmup 0.5 >&2
    ;;
*)
    echo "unknown sweep: $sweep" >&2
    exit 2
    ;;
esac
