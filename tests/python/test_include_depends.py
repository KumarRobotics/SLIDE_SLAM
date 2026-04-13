"""Cross-package ``#include`` -> ``<depend>`` consistency check.

For every C/C++ source file in every managed package, extract every
``#include <pkg/...>`` or ``#include "pkg/..."`` top-level component, find
the owning ``package.xml``, and verify every non-exempt ``pkg`` is listed in
the package.xml's ``<depend>`` / ``<build_depend>`` / ``<exec_depend>`` /
``<test_depend>`` / ``<buildtool_depend>`` tags.

Exempt names are things found via ``find_package()`` + ``target_link_libraries``
rather than via ament dependency resolution - the C++ standard library,
Eigen, Boost, PCL, OpenCV, GTSAM, Sophus, etc.  Self-includes (``pkg``
equals the owning package's own name) are also skipped.

The ``backend/sloam/clipper_semantic_object/`` tree is exempt entirely: it's
a vendored third-party library added via ``add_subdirectory()``, not a ROS
package, so its ``#include``s don't need to appear in sloam's package.xml.

Mirrors Section O of ``tests/static/check_ros2_port.sh``.
"""
from __future__ import annotations

import re
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict, List, Set

import pytest


REPO_ROOT = Path(__file__).resolve().parents[2]

# (relpath_from_repo_root, package_name) for each of the 5 managed packages.
MANAGED_PACKAGE_DIRS = (
    ("backend/sloam_msgs", "sloam_msgs"),
    ("backend/sloam", "sloam"),
    ("backend/multi_robot_utils_launch", "multi_robot_utils_launch"),
    ("frontend/object_modeller", "object_modeller"),
    ("frontend/scan2shape/scan2shape_launch", "scan2shape_launch"),
)

_CPP_SUFFIXES = {".cpp", ".cc", ".h", ".hpp"}

# Top-level pkg extraction: captures whatever appears after the opening
# ``<`` / ``"`` up to the first ``/``. A slash is required so that C++
# stdlib-style angle-bracket includes like ``<string>`` / ``<vector>`` are
# skipped automatically.
_INCLUDE_RE = re.compile(
    r'^\s*#include\s*[<"]([A-Za-z_][A-Za-z0-9_]*)/',
    re.MULTILINE,
)

_CPP_LINE_COMMENT_RE = re.compile(r"//.*")


# Names that are pulled via ``find_package(...)`` + ``target_link_libraries``
# (or the C++ standard library).  They must NOT appear in ``<depend>``.
EXEMPT_INCLUDE_PREFIXES: Set[str] = {
    # System C++ / C libraries
    "Eigen", "eigen3", "unsupported", "boost", "gtsam", "pcl", "opencv2",
    "OpenCV", "fmt", "yaml-cpp", "glog", "tbb", "gtest", "benchmark", "sys",
    "arpa", "netinet", "linux", "fcntl", "unistd", "pthread", "signal",
    "errno", "locale", "stddef", "stdio", "stdlib", "string", "time",
    "wchar", "wctype",
    # C++ stdlib headers that happen to have a slash inside them
    "math", "chrono", "memory", "vector", "map", "set", "iostream",
    "fstream", "sstream", "cstdlib", "cstring", "algorithm", "functional",
    "utility", "tuple", "optional", "thread", "mutex", "atomic",
    "condition_variable", "future", "filesystem", "random", "regex",
    "typeinfo", "cassert", "cmath", "cstddef", "cstdint", "cstdio",
    "limits", "numeric", "queue", "stack", "deque", "list", "array",
    "bitset", "complex", "exception", "stdexcept", "system_error",
    "type_traits", "cctype", "iomanip", "cstdarg", "cfloat", "climits",
    "ctime", "cerrno", "csetjmp", "csignal", "ciso646", "cstdbool",
    "ctgmath", "cuchar", "cwchar", "cwctype",
    # ROS2 runtime infra that's pulled via find_package(), not <depend>
    "rcutils",
    # SlideSLAM-specific vendored/system libs
    "sophus",
}


def _strip_cpp_comments(text: str) -> str:
    return "\n".join(_CPP_LINE_COMMENT_RE.sub("", line) for line in text.splitlines())


def _parse_depends(xml_path: Path) -> Set[str]:
    out: Set[str] = set()
    try:
        tree = ET.parse(xml_path)
    except (ET.ParseError, OSError):
        return out
    root = tree.getroot()
    tags = (
        "depend", "build_depend", "exec_depend", "test_depend",
        "buildtool_depend", "build_export_depend", "run_depend",
    )
    for tag in tags:
        for el in root.iter(tag):
            if el.text:
                out.add(el.text.strip())
    return out


def _package_name(xml_path: Path) -> str:
    try:
        tree = ET.parse(xml_path)
    except (ET.ParseError, OSError):
        return ""
    name_el = tree.getroot().find("name")
    return (name_el.text or "").strip() if name_el is not None else ""


def _collect_cpp_files(package_dir: Path) -> List[Path]:
    out: List[Path] = []
    for p in package_dir.rglob("*"):
        if not p.is_file():
            continue
        if p.suffix.lower() not in _CPP_SUFFIXES:
            continue
        # Exempt the vendored clipper tree. It lives under backend/sloam/ but
        # is added as a raw CMake add_subdirectory() rather than a ROS pkg,
        # so its #includes don't need to live in sloam's package.xml.
        if "clipper_semantic_object" in p.parts:
            continue
        out.append(p)
    return sorted(out)


def _build_include_to_package_map() -> Dict[str, str]:
    mapping: Dict[str, str] = {}
    for relpath, _ in MANAGED_PACKAGE_DIRS:
        pkg_dir = REPO_ROOT / relpath
        if not pkg_dir.is_dir():
            continue
        pkg_xml = pkg_dir / "package.xml"
        if not pkg_xml.is_file():
            continue
        name = _package_name(pkg_xml)
        if not name:
            continue
        include = pkg_dir / "include"
        if not include.is_dir():
            continue
        for sub in include.iterdir():
            if sub.is_dir():
                mapping[sub.name] = name
    return mapping


_INCLUDE_TO_PKG = _build_include_to_package_map()


def _extract_includes(text: str) -> Set[str]:
    stripped = _strip_cpp_comments(text)
    return set(_INCLUDE_RE.findall(stripped))


_PARAM_IDS = [relpath for relpath, _ in MANAGED_PACKAGE_DIRS]
_PARAM_VALUES = [(REPO_ROOT / relpath, pkg) for relpath, pkg in MANAGED_PACKAGE_DIRS]


@pytest.mark.parametrize("package_dir,package_name", _PARAM_VALUES, ids=_PARAM_IDS)
def test_includes_match_depends(package_dir: Path, package_name: str) -> None:
    """Every ``#include <pkg/...>`` in this package's sources is in package.xml."""

    if not package_dir.is_dir():
        pytest.skip(f"package directory does not exist: {package_dir}")

    pkg_xml = package_dir / "package.xml"
    if not pkg_xml.is_file():
        pytest.skip(f"no package.xml under {package_dir}")

    deps = _parse_depends(pkg_xml)
    owning_name = _package_name(pkg_xml) or package_name

    missing: Dict[str, List[str]] = {}
    for src in _collect_cpp_files(package_dir):
        try:
            text = src.read_text(errors="replace")
        except OSError:
            continue
        for pkg in _extract_includes(text):
            if pkg in EXEMPT_INCLUDE_PREFIXES:
                continue
            if pkg == owning_name:
                continue
            resolved = _INCLUDE_TO_PKG.get(pkg, pkg)
            if resolved == owning_name:
                continue
            if resolved in deps or pkg in deps:
                continue
            key = f"{pkg} (->{resolved})" if resolved != pkg else pkg
            missing.setdefault(key, []).append(
                str(src.relative_to(REPO_ROOT)).replace("\\", "/")
            )

    if missing:
        lines = [
            f"#include <{k}/...> not declared in package.xml:\n    "
            + "\n    ".join(sorted(set(v)))
            for k, v in sorted(missing.items())
        ]
        pytest.fail(
            f"{owning_name}: {len(missing)} missing <depend> entr(y/ies)\n  "
            + "\n  ".join(lines)
        )
