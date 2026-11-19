"""Draw the benchmark figures from the CSV files that bench/sweep.sh writes.

    python bench/plot.py bench/results/*.csv --out bench/results/figures

Writes five PNG files and summary.txt (the numbers behind the figures) into the output directory.
Runs of the same configuration from several files are pooled. A figure is skipped, with a message,
when the CSV files do not contain the sweep it needs.
"""

import argparse
import csv
import statistics
import sys
from collections import defaultdict
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.colors import LinearSegmentedColormap, TwoSlopeNorm  # noqa: E402

SURFACE, TEXT, MUTED, GRID = "#fcfcfb", "#0b0b0b", "#52514e", "#e6e5e0"
# Categorical colors in fixed order, and one-hue ramps for ordered series (checked for
# color-vision-deficiency separation; do not reorder)
SERIES = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4"]
RAMP = ["#86b6ef", "#2a78d6", "#104281"]
DIVERGING = LinearSegmentedColormap.from_list("slower_faster", ["#d03b3b", "#f0efec", "#2a78d6"])
# Schedule of every run except the schedule sweep (coven_bench --schedule default)
DEFAULT_SCHEDULE = "guided"
PHASES = [("validate_ns", "edge validation"), ("neighbor_ns", "neighbor search"), ("sample_ns", "sampling"),
          ("insert_ns", "insertion"), ("other_ns", "other")]

plt.rcParams.update({
    "figure.facecolor": SURFACE, "axes.facecolor": SURFACE, "savefig.facecolor": SURFACE,
    "text.color": TEXT, "axes.labelcolor": MUTED, "axes.edgecolor": GRID, "axes.titlecolor": TEXT,
    "xtick.color": MUTED, "ytick.color": MUTED, "axes.grid": True, "grid.color": GRID,
    "grid.linewidth": 0.8, "axes.spines.top": False, "axes.spines.right": False, "axes.axisbelow": True,
    "lines.linewidth": 2, "lines.markersize": 7, "lines.markeredgecolor": SURFACE,
    "lines.markeredgewidth": 1.5, "font.size": 10, "axes.titlesize": 11, "axes.titleweight": "bold",
    "axes.titlelocation": "left", "legend.frameon": False, "figure.dpi": 150,
})


def load(paths):
    """Timed runs grouped by configuration: key -> list of rows with numeric fields."""
    runs = defaultdict(list)
    for path in paths:
        with open(path, newline="") as f:
            for row in csv.DictReader(f):
                for name in ("nodes", "batch", "threads", "chunk", "total_ns", "sample_ns", "neighbor_ns",
                             "validate_ns", "insert_ns", "edges_checked", "vertices", "edges"):
                    row[name] = int(row[name])
                schedule = row["schedule"] + (f",{row['chunk']}" if row["chunk"] else "")
                key = (row["mode"], row["env"], row["nodes"], row["batch"], row["threads"], schedule)
                if row["mode"] == "serial":
                    key = ("serial", row["env"], row["nodes"], 0, 1, "")
                runs[key].append(row)
    return runs


def med(rows, name="total_ns"):
    return statistics.median(r[name] for r in rows)


def quartiles(rows, name="total_ns"):
    values = sorted(r[name] for r in rows)
    if len(values) < 4:
        return values[0], values[-1]
    q = statistics.quantiles(values, n=4)
    return q[0], q[2]


def select(runs, **want):
    """Configurations whose key matches every given field, as {key: rows}."""
    names = ("mode", "env", "nodes", "batch", "threads", "schedule")
    return {k: v for k, v in runs.items() if all(k[names.index(n)] == x for n, x in want.items())}


def most_threads(runs, **want):
    """The (env, nodes, batch) groups among matching parallel runs, ordered by thread counts covered."""
    groups = defaultdict(set)
    for key in select(runs, **want):
        groups[key[1:4]].add(key[4])
    return sorted(groups, key=lambda g: (-len(groups[g]), g))


def save(fig, out, name, lines):
    fig.savefig(out / name, bbox_inches="tight")
    plt.close(fig)
    lines.append(f"wrote {name}")


def thread_axis(ax, threads):
    ax.set_xscale("log", base=2)
    ax.set_xticks(threads, [str(t) for t in threads])
    ax.minorticks_off()
    ax.set_xlabel("threads")


def figure_speedup(runs, out, lines):
    groups = [g for g in most_threads(runs, mode="parallel", schedule=DEFAULT_SCHEDULE)
              if ("serial", g[0], g[1], 0, 1, "") in runs]
    if not groups:
        return lines.append("figure 1 skipped: no scaling sweep with a serial baseline")
    nodes, batch = groups[0][1], groups[0][2]
    envs = sorted({g[0] for g in groups if g[1:] == (nodes, batch)})
    fig, axes = plt.subplots(1, len(envs), figsize=(4.2 * len(envs), 3.8), sharey=True, squeeze=False)
    lines.append(f"\nFigure 1: speedup over unmodified serial PRM, {nodes} nodes, batch {batch}")
    for ax, env in zip(axes[0], envs):
        serial = med(runs[("serial", env, nodes, 0, 1, "")])
        by_threads = {k[4]: v for k, v in select(
            runs, mode="parallel", env=env, nodes=nodes, batch=batch, schedule=DEFAULT_SCHEDULE).items()}
        threads = sorted(by_threads)
        speedup = [serial / med(by_threads[t]) for t in threads]
        low = [serial / quartiles(by_threads[t])[1] for t in threads]
        high = [serial / quartiles(by_threads[t])[0] for t in threads]

        # Amdahl's law from the one-thread breakdown: only validation and neighbor search scale
        one = by_threads[threads[0]]
        parallel_share = (med(one, "validate_ns") + med(one, "neighbor_ns")) / med(one)
        amdahl = [speedup[0] / ((1 - parallel_share) + parallel_share * threads[0] / t) for t in threads]

        ax.plot(threads, threads, color=GRID, linewidth=1.2, label="linear")
        ax.plot(threads, amdahl, color=MUTED, linewidth=1.2, label="Amdahl limit")
        ax.fill_between(threads, low, high, color=SERIES[0], alpha=0.15, linewidth=0)
        ax.plot(threads, speedup, color=SERIES[0], marker="o", label="measured")
        ax.annotate(f"{speedup[-1]:.1f}x", (threads[-1], speedup[-1]), xytext=(-4, 8),
                    textcoords="offset points", ha="right", color=TEXT)
        ax.set_title(f"{env}: serial share {100 * (1 - parallel_share):.1f}%")
        thread_axis(ax, threads)
        ax.set_yscale("log", base=2)
        ax.set_yticks(threads, [str(t) for t in threads])
        lines.append(f"  {env}: serial {serial / 1e6:.1f} ms, serial share {100 * (1 - parallel_share):.2f}%")
        for t, s, a in zip(threads, speedup, amdahl):
            q1, q3 = quartiles(by_threads[t])
            lines.append(f"    {t:3d} threads: {serial / s / 1e6:9.1f} ms (IQR {q1 / 1e6:.1f}-{q3 / 1e6:.1f})  "
                         f"speedup {s:5.2f}x  vs one thread "
                         f"{speedup[0] and s / speedup[0]:5.2f}x  efficiency {100 * s / t:5.1f}%  "
                         f"Amdahl {a:5.2f}x")
    axes[0][0].set_ylabel("speedup over serial PRM")
    axes[0][0].legend(loc="upper left")
    fig.suptitle(f"Roadmap construction, {nodes} nodes, batch {batch}", x=0.02, y=1.04, ha="left",
                 fontweight="bold")
    save(fig, out, "1_speedup.png", lines)


def figure_breakdown(runs, out, lines):
    groups = most_threads(runs, mode="parallel", schedule=DEFAULT_SCHEDULE)
    if not groups:
        return lines.append("figure 2 skipped: no parallel runs")
    env, nodes, batch = next((g for g in groups if g[0] == "cage"), groups[0])
    by_threads = {k[4]: v for k, v in select(
        runs, mode="parallel", env=env, nodes=nodes, batch=batch, schedule=DEFAULT_SCHEDULE).items()}
    threads = sorted(by_threads)
    parts = {}
    for t in threads:
        parts[t] = {name: med(by_threads[t], name) / 1e6 for name, _ in PHASES[:-1]}
        parts[t]["other_ns"] = max(med(by_threads[t]) / 1e6 - sum(parts[t].values()), 0)

    fig, (absolute, share) = plt.subplots(1, 2, figsize=(10, 3.8))
    lines.append(f"\nFigure 2: time breakdown in ms, {env}, {nodes} nodes, batch {batch}")
    lines.append("  threads " + " ".join(f"{label:>16}" for _, label in PHASES))
    x = range(len(threads))
    for ax, relative in ((absolute, False), (share, True)):
        bottom = [0.0] * len(threads)
        for (name, label), color in zip(PHASES, SERIES):
            height = [parts[t][name] * (100 / sum(parts[t].values()) if relative else 1) for t in threads]
            ax.bar(x, height, 0.5, bottom=bottom, color=color, edgecolor=SURFACE, linewidth=1.5, label=label)
            bottom = [b + h for b, h in zip(bottom, height)]
        ax.set_xticks(x, [str(t) for t in threads])
        ax.set_xlabel("threads")
        ax.grid(axis="x", visible=False)
    for t in threads:
        lines.append(f"  {t:7d} " + " ".join(f"{parts[t][name]:16.1f}" for name, _ in PHASES))
    absolute.set_title("Time per phase (ms)")
    share.set_title("Share of total time (%)")
    share.legend(loc="center left", bbox_to_anchor=(1.0, 0.5))
    fig.suptitle(f"Where the time goes: {env}, {nodes} nodes, batch {batch}", x=0.02, y=1.04, ha="left",
                 fontweight="bold")
    save(fig, out, "2_breakdown.png", lines)


def figure_schedule(runs, out, lines):
    schedules = defaultdict(dict)
    for key, rows in select(runs, mode="parallel").items():
        schedules[(key[1], key[2], key[3], key[5])][key[4]] = rows
    groups = defaultdict(list)
    for env, nodes, batch, schedule in schedules:
        groups[(env, nodes, batch)].append(schedule)
    groups = {g: s for g, s in groups.items() if len(s) > 1 and ("serial", g[0], g[1], 0, 1, "") in runs}
    if not groups:
        return lines.append("figure 3 skipped: no schedule sweep")
    env, nodes, batch = max(groups, key=lambda g: (len(groups[g]), g[0] == "cage"))
    serial = med(runs[("serial", env, nodes, 0, 1, "")])
    fig, ax = plt.subplots(figsize=(5.6, 3.8))
    lines.append(f"\nFigure 3: speedup by OpenMP schedule, {env}, {nodes} nodes, batch {batch}")
    for schedule, color in zip(sorted(groups[(env, nodes, batch)]), SERIES):
        by_threads = schedules[(env, nodes, batch, schedule)]
        threads = sorted(by_threads)
        speedup = [serial / med(by_threads[t]) for t in threads]
        ax.plot(threads, speedup, color=color, marker="o", label=schedule.replace(",", ", chunk "))
        lines.append(f"  {schedule:12s} " + "  ".join(f"{t}t {s:.2f}x" for t, s in zip(threads, speedup)))
    thread_axis(ax, threads)
    ax.set_ylabel("speedup over serial PRM")
    ax.set_title(f"Schedule of the edge validation loop: {env}, {nodes} nodes")
    ax.legend(loc="upper left")
    save(fig, out, "3_schedule.png", lines)


def figure_sizes(runs, out, lines):
    cells = {}
    for key, rows in select(runs, mode="parallel", schedule=DEFAULT_SCHEDULE).items():
        _, env, nodes, batch, threads, _ = key
        serial = runs.get(("serial", env, nodes, 0, 1, ""))
        if serial and threads > 1:
            cells.setdefault((env, threads), {})[(nodes, batch)] = med(serial) / med(rows)
    cells = {g: c for g, c in cells.items() if len({b for _, b in c}) > 1 and len({n for n, _ in c}) > 1}
    if not cells:
        return lines.append("figure 4 skipped: no batch size x roadmap size sweep")
    (env, threads), grid = max(cells.items(), key=lambda item: len(item[1]))
    # Roadmap sizes from other sweeps (a single batch size) would leave a row of empty cells
    sizes = sorted({n for n, _ in grid if sum(m == n for m, _ in grid) > 1})
    grid = {(n, b): v for (n, b), v in grid.items() if n in sizes}
    batches = sorted({b for _, b in grid})
    fig, ax = plt.subplots(figsize=(1.0 * len(batches) + 2.2, 0.55 * len(sizes) + 1.6))
    values = [[grid.get((n, b), float("nan")) for b in batches] for n in sizes]
    top = max(max(v for v in grid.values()), 2.0)
    ax.imshow(values, cmap=DIVERGING, norm=TwoSlopeNorm(1.0, min(min(grid.values()), 0.5), top),
              aspect="auto")
    lines.append(f"\nFigure 4: speedup over serial PRM at {threads} threads, {env} (rows nodes, columns batch)")
    lines.append("  nodes    " + " ".join(f"{b:>7}" for b in batches))
    for i, n in enumerate(sizes):
        for j, b in enumerate(batches):
            if (n, b) in grid:
                ax.text(j, i, f"{grid[(n, b)]:.2f}x", ha="center", va="center",
                        color="white" if grid[(n, b)] > 0.6 * top or grid[(n, b)] < 0.6 else TEXT)
        lines.append(f"  {n:7d}  " + " ".join(f"{grid.get((n, b), float('nan')):6.2f}x" for b in batches))
    wins = [n for n in sizes if max(grid.get((n, b), 0) for b in batches) > 1]
    crossover = (f"Parallel wins from {wins[0]} nodes" if wins else "Parallel never wins") + \
        (f"; loses at {max(n for n in sizes if n not in wins)} and below" if len(wins) < len(sizes) else "")
    lines.append(f"  crossover: {crossover}")
    ax.set_xticks(range(len(batches)), [str(b) for b in batches])
    ax.set_yticks(range(len(sizes)), [str(n) for n in sizes])
    ax.set_xlabel("batch size")
    ax.set_ylabel("roadmap nodes")
    ax.grid(visible=False)
    ax.set_title(f"Speedup over serial PRM at {threads} threads, {env}\nRed is slower than serial. "
                 f"{crossover}.", fontsize=10)
    save(fig, out, "4_sizes.png", lines)


def figure_revalidate(runs, out, lines):
    by_size = defaultdict(dict)
    for key, rows in select(runs, mode="revalidate").items():
        by_size[(key[1], key[2])][key[4]] = rows
    if not by_size:
        return lines.append("figure 5 skipped: no revalidation sweep")
    fig, ax = plt.subplots(figsize=(5.6, 3.8))
    lines.append("\nFigure 5: full revalidation of a roadmap")
    shown = sorted(by_size)[-len(RAMP):]
    for (env, nodes), color in zip(shown, RAMP[-len(shown):]):
        by_threads = by_size[(env, nodes)]
        threads = sorted(by_threads)
        edges = by_threads[threads[0]][0]["edges_checked"]
        rate = [edges / (med(by_threads[t]) / 1e9) / 1e6 for t in threads]
        ax.plot(threads, rate, color=color, marker="o", label=f"{nodes} nodes, {edges} edges")
        lines.append(f"  {env} {nodes} nodes, {edges} edges")
        for t, r in zip(threads, rate):
            lines.append(f"    {t:3d} threads: {med(by_threads[t]) / 1e6:9.2f} ms per recheck, "
                         f"{r:6.2f} M edges/s, {1e9 / med(by_threads[t]):7.1f} rechecks/s")
    thread_axis(ax, threads)
    ax.set_ylim(bottom=0)
    ax.set_ylabel("million edges rechecked per second")
    ax.set_title("Revalidation rate after the obstacles move")
    ax.legend(loc="upper left")
    save(fig, out, "5_revalidate.png", lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("csv", nargs="+", type=Path)
    parser.add_argument("--out", type=Path, default=Path("bench/results/figures"))
    args = parser.parse_args()
    runs = load(args.csv)
    if not runs:
        sys.exit("no rows in the given CSV files")
    args.out.mkdir(parents=True, exist_ok=True)

    first = next(iter(runs.values()))[0]
    lines = [f"cpu: {first['cpu']}", f"compiler: {first['compiler']}", f"commit: {first['commit']}"]
    for figure in (figure_speedup, figure_breakdown, figure_schedule, figure_sizes, figure_revalidate):
        figure(runs, args.out, lines)
    (args.out / "summary.txt").write_text("\n".join(lines) + "\n")
    print("\n".join(lines))


if __name__ == "__main__":
    main()
