"""Launch dict-parameter keys vs ``declare_parameter()`` cross-reference.

For every ``Node(package=P, executable=E, parameters=[{...}])`` in a launch
file, verify every literal dict key is declared in package ``P``'s source
tree via either ``declare_parameter("key", ...)``, the sloam wrapper family
``declare_or_get<T>(node, "key", default)`` (plain or prefixed with any
identifier chars), the ``get_param_or(node, "key", ...)`` helper, or a
``declare_parameter_if_not_declared(node, "key", ...)`` form. A missing
declaration is a silent-no-op bug at runtime: ROS2 ignores un-declared
parameters passed through launch, so the target node sees only its C++
defaults.

External packages (``package=`` not in the managed list) and non-dict
parameter entries (``parameters=[yaml_path]``) are skipped.

Special case: ``scan2shape_launch`` installs scripts from the sibling
``frontend/scan2shape/script/`` directory via ``install(PROGRAMS
../script/*.py)``, so we walk that directory too when collecting declared
keys for that package.

Mirrors Section P of ``tests/static/check_ros2_port.sh``.
"""
from __future__ import annotations

import re
from pathlib import Path
from typing import Dict, List, Set

import pytest


REPO_ROOT = Path(__file__).resolve().parents[2]


# Map managed package name -> list of search roots to walk when collecting
# declared parameter keys. Most packages are a single directory; some
# (scan2shape_launch) install scripts from a sibling directory.
PKG_SOURCE_ROOTS: Dict[str, List[Path]] = {
    "sloam_msgs": [REPO_ROOT / "backend" / "sloam_msgs"],
    "sloam": [REPO_ROOT / "backend" / "sloam"],
    "multi_robot_utils_launch": [REPO_ROOT / "backend" / "multi_robot_utils_launch"],
    "object_modeller": [REPO_ROOT / "frontend" / "object_modeller"],
    "scan2shape_launch": [
        REPO_ROOT / "frontend" / "scan2shape" / "scan2shape_launch",
        REPO_ROOT / "frontend" / "scan2shape" / "script",
    ],
}

MANAGED_PKG_NAMES: Set[str] = set(PKG_SOURCE_ROOTS.keys())


_CPP_SUFFIXES = {".cpp", ".cc", ".h", ".hpp"}
_PY_SUFFIXES = {".py"}
_CPP_LINE_COMMENT_RE = re.compile(r"//.*")

# In the flattened launch source, find every ``Node(...)`` block whose ``(``
# is not preceded by a word character (excludes ComposableNode /
# LifecycleNode). We stop at the first ``)``, which is good enough for the
# repo's launch style.
_NODE_BLOCK_RE = re.compile(r"(?:^|[^A-Za-z0-9_])Node\s*\(([^)]{0,2000})\)")

_DICT_KEY_RE = re.compile(r"['\"]([A-Za-z_][A-Za-z0-9_.]*)['\"]\s*:")
_PACKAGE_RE = re.compile(r"package\s*=\s*['\"]([^'\"]+)['\"]")
_EXECUTABLE_RE = re.compile(r"executable\s*=\s*['\"]([^'\"]+)['\"]")
_PARAMETERS_RE = re.compile(r"parameters\s*=\s*\[([^\]]*)\]")

_DECLARE_CPP_RE = re.compile(
    r'declare_parameter\s*(?:<[^>]*>)?\s*\(\s*"([^"]+)"',
)
# sloam wrappers: declare_or_get<T>(node, "key", default) OR any prefix
# form like _declare_or_get<T>(...). No leading-underscore requirement.
_DECLARE_OR_GET_RE = re.compile(
    r'[A-Za-z_]*declare_or_get\s*<[^>]*>\s*\(\s*[A-Za-z_][A-Za-z0-9_>.]*\s*,\s*"([^"]+)"',
)
_GET_PARAM_OR_RE = re.compile(
    r'\bget_param_or\s*\(\s*[A-Za-z_][A-Za-z0-9_>.]*\s*,\s*"([^"]+)"',
)
_DECL_IF_NOT_RE = re.compile(
    r'\bdeclare_parameter_if_not_declared\s*\(\s*[A-Za-z_][A-Za-z0-9_>.]*\s*,\s*"([^"]+)"',
)
_DECLARE_PY_RE = re.compile(
    r'declare_parameter\s*\(\s*[\'"]([^\'"]+)[\'"]',
)


def _strip_cpp_comments(text: str) -> str:
    return "\n".join(_CPP_LINE_COMMENT_RE.sub("", line) for line in text.splitlines())


def _strip_py_comments(text: str) -> str:
    # Not string-literal-aware; that's fine for our patterns (no
    # declare_parameter("#foo", ...) shenanigans in this repo).
    lines = []
    for line in text.splitlines():
        in_s = False
        in_d = False
        out = []
        for ch in line:
            if ch == "#" and not in_s and not in_d:
                break
            if ch == "'" and not in_d:
                in_s = not in_s
            elif ch == '"' and not in_s:
                in_d = not in_d
            out.append(ch)
        lines.append("".join(out))
    return "\n".join(lines)


def _collect_declared_keys_from_dirs(pkg_dirs: List[Path]) -> Set[str]:
    keys: Set[str] = set()
    for pkg_dir in pkg_dirs:
        if not pkg_dir.is_dir():
            continue
        for src in pkg_dir.rglob("*"):
            if not src.is_file():
                continue
            if "clipper_semantic_object" in src.parts:
                continue
            suffix = src.suffix.lower()
            try:
                text = src.read_text(errors="replace")
            except OSError:
                continue
            if suffix in _CPP_SUFFIXES:
                stripped = _strip_cpp_comments(text)
                keys.update(_DECLARE_CPP_RE.findall(stripped))
                keys.update(_DECLARE_OR_GET_RE.findall(stripped))
                keys.update(_GET_PARAM_OR_RE.findall(stripped))
                keys.update(_DECL_IF_NOT_RE.findall(stripped))
            elif suffix in _PY_SUFFIXES:
                stripped = _strip_py_comments(text)
                keys.update(_DECLARE_PY_RE.findall(stripped))
    return keys


def _launch_files() -> List[Path]:
    out: List[Path] = []
    for sub in ("backend", "frontend"):
        base = REPO_ROOT / sub
        if not base.is_dir():
            continue
        out.extend(sorted(base.rglob("*.launch.py")))
    return out


_LAUNCH_FILES = _launch_files()
_LAUNCH_IDS = [
    str(p.relative_to(REPO_ROOT)).replace("\\", "/") for p in _LAUNCH_FILES
]


_DECLARED_CACHE: Dict[str, Set[str]] = {}


def _declared_keys_for(pkg_name: str) -> Set[str]:
    if pkg_name in _DECLARED_CACHE:
        return _DECLARED_CACHE[pkg_name]
    roots = PKG_SOURCE_ROOTS.get(pkg_name, [])
    if not roots:
        _DECLARED_CACHE[pkg_name] = set()
    else:
        _DECLARED_CACHE[pkg_name] = _collect_declared_keys_from_dirs(roots)
    return _DECLARED_CACHE[pkg_name]


@pytest.mark.parametrize("launch_file", _LAUNCH_FILES, ids=_LAUNCH_IDS)
def test_launch_param_keys(launch_file: Path) -> None:
    """Every dict key in ``parameters=[{...}]`` is declared in the target pkg."""

    try:
        raw = launch_file.read_text(errors="replace")
    except OSError:
        pytest.skip(f"could not read {launch_file}")
        return

    flat = raw.replace("\n", " ")
    problems: List[str] = []
    nodes_seen = 0

    for match in _NODE_BLOCK_RE.finditer(flat):
        block = match.group(1)
        pkg_m = _PACKAGE_RE.search(block)
        if not pkg_m:
            continue
        pkg = pkg_m.group(1)
        if pkg not in MANAGED_PKG_NAMES:
            continue
        nodes_seen += 1
        exe_m = _EXECUTABLE_RE.search(block)
        exe = exe_m.group(1) if exe_m else "<unknown>"
        plist_m = _PARAMETERS_RE.search(block)
        if not plist_m:
            continue
        plist = plist_m.group(1)
        keys = set(_DICT_KEY_RE.findall(plist))
        if not keys:
            continue
        declared = _declared_keys_for(pkg)
        for key in sorted(keys):
            if key.startswith("/") or key.endswith((".yaml", ".yml")):
                continue
            if key not in declared:
                problems.append(
                    f"Node(package={pkg!r}, executable={exe!r}) "
                    f"parameters=[{{'{key}': ...}}] "
                    f"but no declare_parameter(\"{key}\", ...) found in {pkg} source"
                )

    if problems:
        pytest.fail(
            f"{launch_file.relative_to(REPO_ROOT)}: "
            f"{len(problems)} undeclared launch parameter key(s) "
            f"({nodes_seen} Node calls checked)\n  "
            + "\n  ".join(problems)
        )
