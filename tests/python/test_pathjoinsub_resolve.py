"""Cross-package PathJoinSubstitution target resolution.

For every ``*.launch.py`` file, every
``PathJoinSubstitution([FindPackageShare('<pkg>'), 'seg1', 'seg2', ...])``
where ``<pkg>`` is a managed package must resolve to a file that exists in
the target package's source tree.

Catches launch files whose config / rviz / sub-launch path references went
stale during the ROS1 -> ROS2 port (e.g., a config yaml renamed on master
but still referenced under the old name in a launch file).

Carve-outs:
- External packages (FindPackageShare('X') where X is not managed) are
  skipped — we cannot introspect their share/ trees.
- Directory-only targets (no file extension on the last segment) are
  skipped — those resolve at runtime by convention.
- ``msckf_calib.yaml`` is skipped: the upstream workflow generates this
  file at first run via ``msckf_calib_gen``, so an apparent "missing"
  reference is intentional.

Mirrors Section R of ``tests/static/check_ros2_port.sh``.
"""
from __future__ import annotations

import re
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict, List, Tuple

import pytest


REPO_ROOT = Path(__file__).resolve().parents[2]

_MANAGED_SEARCH_ROOTS = ("backend", "frontend")

# Files intentionally referenced despite being generated at first run or
# living in a different package than FindPackageShare() implies.
_CARVE_BASENAMES = {"msckf_calib.yaml"}


_PJ_SPAN_RE = re.compile(
    r"PathJoinSubstitution\s*\(\s*\[(?P<body>[^\]]+)\]",
    re.DOTALL,
)
_FPS_RE = re.compile(
    r"FindPackageShare\s*\(\s*['\"](?P<pkg>[^'\"]+)['\"]\s*\)",
)
_STRING_RE = re.compile(r"['\"]([^'\"]+)['\"]")


def _build_pkgmap() -> Dict[str, Path]:
    out: Dict[str, Path] = {}
    for root in _MANAGED_SEARCH_ROOTS:
        for xml in (REPO_ROOT / root).rglob("package.xml"):
            if "clipper_semantic_object" in xml.parts:
                continue
            try:
                tree = ET.parse(xml)
            except (ET.ParseError, OSError):
                continue
            name_el = tree.getroot().find("name")
            if name_el is None or not name_el.text:
                continue
            out[name_el.text.strip()] = xml.parent
    return out


_PKG_MAP = _build_pkgmap()


def _launch_files() -> List[Path]:
    out: List[Path] = []
    for root in _MANAGED_SEARCH_ROOTS:
        out.extend(sorted((REPO_ROOT / root).rglob("*.launch.py")))
    return [p for p in out if "clipper_semantic_object" not in p.parts]


def _extract_refs(text: str) -> List[Tuple[str, List[str]]]:
    """Return a list of (pkg_name, [segments...]) for each managed ref."""
    flat = text.replace("\n", " ")
    refs: List[Tuple[str, List[str]]] = []
    for m in _PJ_SPAN_RE.finditer(flat):
        body = m.group("body")
        fps = _FPS_RE.search(body)
        if not fps:
            continue
        pkg = fps.group("pkg")
        after = body[fps.end():]
        segs = _STRING_RE.findall(after)
        refs.append((pkg, segs))
    return refs


_LAUNCH_FILES = _launch_files()
_LAUNCH_IDS = [
    str(p.relative_to(REPO_ROOT)).replace("\\", "/") for p in _LAUNCH_FILES
]


@pytest.mark.parametrize("launch_file", _LAUNCH_FILES, ids=_LAUNCH_IDS)
def test_pathjoinsub_targets_resolve(launch_file: Path) -> None:
    """All cross-package PathJoinSubstitution targets exist on disk."""

    try:
        text = launch_file.read_text(errors="replace")
    except OSError:
        pytest.skip(f"could not read {launch_file}")
        return

    missing: List[str] = []
    for pkg, segs in _extract_refs(text):
        if pkg not in _PKG_MAP:
            continue  # external package
        if not segs:
            continue
        path = "/".join(segs)
        base = segs[-1]
        if "." not in base:
            continue  # directory target, not a file
        if base in _CARVE_BASENAMES:
            continue
        target = _PKG_MAP[pkg] / path
        if not target.exists():
            missing.append(
                f"FindPackageShare({pkg!r}) -> {path} (target not at "
                f"{target.relative_to(REPO_ROOT)})"
            )

    if missing:
        pytest.fail(
            f"{launch_file.relative_to(REPO_ROOT)}: "
            f"{len(missing)} unresolved PathJoinSubstitution target(s)\n  "
            + "\n  ".join(missing)
        )
