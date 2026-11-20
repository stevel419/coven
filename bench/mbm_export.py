"""Write the MotionBenchMaker problems for the Panda as plain text for coven_bench and coven_mbm.

    python bench/mbm_export.py bench/mbm_panda.txt

Reads them through coven.problems (VAMP's resources/panda/problems.tar.bz2 at the commit coven builds
against). One problem per block:

    problem <scenario> <number> <valid>     valid: 1 when start and goal are collision-free
    start <7 joint values>
    goal <7 joint values>
    sphere <x y z> <radius>
    cuboid <x y z> <euler x y z> <half extents x y z>
    capsule <x y z> <euler x y z> <radius> <length>
"""

import sys

import numpy as np
import vamp

from coven import problems
from coven.server import environment


def numbers(values):
    return " ".join(f"{v:.6g}" for v in values)


def main(out):
    lines = [
        "# MotionBenchMaker problems for the Panda, from VAMP's resources/panda/problems.tar.bz2 (VAMP and",
        "# MotionBenchMaker, Kavraki Lab). Written by bench/mbm_export.py; see that file for the format.",
        "# Derived from VAMP and licensed under its Apache License 2.0: see bench/mbm_panda.LICENSE.",
    ]
    for scenario in problems.scenarios():
        for number, p in enumerate(problems.problems(scenario), 1):
            env = environment(p["scene"])
            valid = all(vamp.panda.validate(np.array(q, dtype=np.float32), env) for q in (p["start"], p["goal"]))
            lines.append(f"problem {scenario} {number} {int(valid)}")
            lines.append(f"start {numbers(p['start'])}")
            lines.append(f"goal {numbers(p['goal'])}")
            for s in p["scene"]["spheres"]:
                lines.append(f"sphere {numbers(s['center'])} {s['radius']:.6g}")
            for c in p["scene"]["cuboids"]:
                lines.append(f"cuboid {numbers(c['center'])} {numbers(c['euler'])} {numbers(c['half_extents'])}")
            for c in p["scene"]["capsules"]:
                lines.append(f"capsule {numbers(c['center'])} {numbers(c['euler'])} {c['radius']:.6g} {c['length']:.6g}")
    with open(out, "w") as f:
        f.write("\n".join(lines) + "\n")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "bench/mbm_panda.txt")
