"""VAMP's Panda model (URDF and meshes), downloaded on first use and cached."""

import urllib.request
from pathlib import Path

# The VAMP commit that coven builds against (see CMakeLists.txt)
VAMP_COMMIT = "27cb9b66ebd8d101ce88d330f3b028398afc1cfa"
PANDA_FILES = ["panda_spherized.urdf", "meshes/collision/link0.obj", "meshes/collision/link7.obj"] + [
    f"meshes/visual/{name}.obj" for name in ("finger", "hand", "link1", "link2", "link3", "link4", "link5", "link6")]


def cache_dir():
    """Where downloaded VAMP resources for the Panda are kept."""
    return Path.home() / ".cache" / "coven" / VAMP_COMMIT[:12] / "panda"


def panda_urdf():
    """Path to VAMP's Panda URDF, downloading it and its meshes (about 9 MB) on first use."""
    cache = cache_dir()
    for name in PANDA_FILES:
        path = cache / name
        if not path.exists():
            path.parent.mkdir(parents=True, exist_ok=True)
            url = f"https://raw.githubusercontent.com/KavrakiLab/vamp/{VAMP_COMMIT}/resources/panda/{name}"
            urllib.request.urlretrieve(url, path.with_suffix(".part"))
            path.with_suffix(".part").replace(path)
    return cache / "panda_spherized.urdf"
