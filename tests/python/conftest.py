"""pytest fixtures for the SlideSLAM ros2_dev static-check suite.

These tests intentionally have ZERO ROS / ROS2 / rclpy / rclcpp imports — they
walk the source tree with stdlib only and pattern-match against the same rules
encoded in tests/static/check_ros2_port.sh. Anyone with Python 3.10+ and pytest
installed can run them; no workspace build is required.

Fixtures
--------
repo_root           absolute Path to the SlideSLAM repo (auto-detected from this
                    file's location: ../../).
exempt_paths        list of fnmatch-style globs (relative to repo_root) that
                    every check should skip. Mostly: vendored third-party libs,
                    rviz config, and the README banner.
package_dirs        list of Paths to the 5 ROS2 packages we actively maintain.
active_cpp_files    list of Paths to .cpp/.h/.hpp files we want to lint.
active_py_files     list of Paths to .py files we want to lint.
"""
from __future__ import annotations

import fnmatch
from pathlib import Path

import pytest


REPO_ROOT = Path(__file__).resolve().parents[2]


# Globs (relative to repo_root) that should be ignored by all checks.
EXEMPT_GLOBS = [
    "backend/sloam/clipper_semantic_object/*",
    "backend/sloam/clipper_semantic_object/**/*",
    "**/rviz/*",
    "**/*.rviz",
    "tests/*",
    "tests/**/*",
    "tools/*",
    "tools/**/*",
    "README.md",  # contains intentional "Noetic" mention in a banner
    ".git/**/*",
]


def _is_exempt(path: Path) -> bool:
    rel = path.relative_to(REPO_ROOT).as_posix()
    return any(fnmatch.fnmatch(rel, g) for g in EXEMPT_GLOBS)


@pytest.fixture(scope="session")
def repo_root() -> Path:
    return REPO_ROOT


@pytest.fixture(scope="session")
def exempt_paths() -> list[str]:
    return list(EXEMPT_GLOBS)


@pytest.fixture(scope="session")
def package_dirs() -> list[Path]:
    return [
        REPO_ROOT / "backend" / "sloam_msgs",
        REPO_ROOT / "backend" / "sloam",
        REPO_ROOT / "backend" / "multi_robot_utils_launch",
        REPO_ROOT / "frontend" / "object_modeller",
        REPO_ROOT / "frontend" / "scan2shape" / "scan2shape_launch",
    ]


def _walk_files(roots: list[Path], suffixes: tuple[str, ...]) -> list[Path]:
    out: list[Path] = []
    for r in roots:
        if not r.exists():
            continue
        for p in r.rglob("*"):
            if not p.is_file():
                continue
            if p.suffix not in suffixes:
                continue
            if _is_exempt(p):
                continue
            out.append(p)
    return out


@pytest.fixture(scope="session")
def active_cpp_files() -> list[Path]:
    roots = [
        REPO_ROOT / "backend" / "sloam" / "src",
        REPO_ROOT / "backend" / "sloam" / "include",
        REPO_ROOT / "frontend" / "object_modeller" / "src",
        REPO_ROOT / "frontend" / "object_modeller" / "include",
    ]
    return _walk_files(roots, (".cpp", ".h", ".hpp", ".cc"))


@pytest.fixture(scope="session")
def active_py_files() -> list[Path]:
    roots = [
        REPO_ROOT / "frontend" / "object_modeller",
        REPO_ROOT / "frontend" / "scan2shape",
    ]
    return _walk_files(roots, (".py",))


# ---------- comment-stripping helpers shared across tests ----------

def strip_cpp_comment(line: str) -> str:
    """Return only the code portion of a C++ source line.

    Strips // line comments and /* ... */ inline comments. Doesn't try to be
    string-literal-aware — that's good enough for the token-level checks here.
    """
    idx = line.find("//")
    if idx >= 0:
        line = line[:idx]
    # very small inline /* ... */ stripper
    while True:
        a = line.find("/*")
        if a < 0:
            break
        b = line.find("*/", a + 2)
        if b < 0:
            line = line[:a]
            break
        line = line[:a] + line[b + 2 :]
    return line


def strip_py_comment(line: str) -> str:
    """Return only the code portion of a Python source line.

    Drops trailing # comments. Not string-literal-aware (fine for our patterns).
    """
    idx = line.find("#")
    if idx >= 0:
        line = line[:idx]
    return line


def iter_code_lines(text: str):
    """Yield (lineno, code_text) for every Python source line, with triple-quoted
    docstrings blanked out.

    Intended for static checks that want to ignore prose inside docstrings
    (e.g. "Replaces ros_numpy.numpify ..." should not flag the ros_numpy check).
    Trailing # comments are also stripped via strip_py_comment().

    The implementation is a small hand-rolled state machine; it does NOT use
    Python's `tokenize` module because we want to gracefully handle files that
    aren't strictly valid Python. It's good enough for our pattern-matching
    use case but is NOT a full lexer (it ignores backslash escapes inside
    strings, doesn't handle f-strings specially, etc.).
    """
    in_doc = False
    q = ""
    for lineno, raw in enumerate(text.splitlines(), start=1):
        out_chars: list[str] = []
        rest = raw
        while rest:
            if in_doc:
                idx = rest.find(q)
                if idx < 0:
                    rest = ""
                    break
                rest = rest[idx + 3 :]
                in_doc = False
                q = ""
            else:
                i1 = rest.find('"""')
                i2 = rest.find("'''")
                if i1 < 0 and i2 < 0:
                    out_chars.append(rest)
                    rest = ""
                    break
                if i1 >= 0 and (i2 < 0 or i1 < i2):
                    out_chars.append(rest[:i1])
                    rest = rest[i1 + 3 :]
                    in_doc = True
                    q = '"""'
                else:
                    out_chars.append(rest[:i2])
                    rest = rest[i2 + 3 :]
                    in_doc = True
                    q = "'''"
        code = "".join(out_chars)
        code = strip_py_comment(code)
        yield lineno, code
