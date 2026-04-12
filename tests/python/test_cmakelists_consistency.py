"""Validate every CMakeLists.txt under backend/ and frontend/ is ament-clean.

Mirrors sections D and H of tests/static/check_ros2_port.sh.

What we check:
* no find_package(catkin ...)
* no catkin_package() / catkin_INCLUDE_DIRS / catkin_LIBRARIES
* no add_message_files / add_service_files / add_action_files / generate_messages
* every package CMakeLists ends with an ament_package() call
* every install(PROGRAMS ...) target points at a file that exists on disk

What we DO NOT check:
* CMake target dependencies (target_link_libraries / ament_target_dependencies)
* whether referenced executables actually compile
* the order of find_package() calls relative to add_executable()
"""
from __future__ import annotations

import re
from pathlib import Path

import pytest


def find_cmakelists(repo_root: Path) -> list[Path]:
    out: list[Path] = []
    for sub in ("backend", "frontend"):
        for p in (repo_root / sub).rglob("CMakeLists.txt"):
            if "clipper_semantic_object" in p.parts:
                continue
            out.append(p)
    return out


def pytest_generate_tests(metafunc):
    if "cmake_file" in metafunc.fixturenames:
        repo_root = Path(__file__).resolve().parents[2]
        files = find_cmakelists(repo_root)
        ids = [str(p.relative_to(repo_root)) for p in files]
        metafunc.parametrize("cmake_file", files, ids=ids)


CATKIN_PATTERNS = [
    ("find_package(catkin)", re.compile(r"find_package\s*\(\s*catkin\b")),
    ("catkin_package()",     re.compile(r"\bcatkin_package\s*\(")),
    ("catkin_INCLUDE_DIRS",  re.compile(r"\bcatkin_INCLUDE_DIRS\b")),
    ("catkin_LIBRARIES",     re.compile(r"\bcatkin_LIBRARIES\b")),
    ("add_message_files",    re.compile(r"\badd_message_files\s*\(")),
    ("add_service_files",    re.compile(r"\badd_service_files\s*\(")),
    ("add_action_files",     re.compile(r"\badd_action_files\s*\(")),
    ("generate_messages",    re.compile(r"\bgenerate_messages\s*\(")),
]


@pytest.mark.parametrize("name,pattern", CATKIN_PATTERNS, ids=[n for n, _ in CATKIN_PATTERNS])
def test_cmake_no_catkin_leftovers(cmake_file: Path, name: str, pattern: re.Pattern[str]):
    text = cmake_file.read_text(encoding="utf-8")
    assert not pattern.search(text), f"{cmake_file}: still uses {name}"


def test_cmake_calls_ament_package(cmake_file: Path):
    text = cmake_file.read_text(encoding="utf-8")
    assert re.search(r"^\s*ament_package\s*\(\s*\)", text, re.MULTILINE), (
        f"{cmake_file}: missing ament_package() call"
    )


# ----- install(PROGRAMS ...) target existence ------------------------------

INSTALL_PROGRAMS_RE = re.compile(
    r"install\s*\(\s*PROGRAMS\b(?P<body>.*?)\)",
    re.DOTALL,
)


def _extract_install_programs(text: str) -> list[list[str]]:
    """Return one list of file tokens per install(PROGRAMS ...) block.

    Strips trailing DESTINATION ... arguments and inline # comments. The
    tokens are returned in raw form (whatever appeared in the CMakeLists),
    so the caller has to resolve them relative to the CMakeLists' directory.
    """
    blocks: list[list[str]] = []
    for m in INSTALL_PROGRAMS_RE.finditer(text):
        body = m.group("body")
        # cut at the first DESTINATION keyword
        dest_idx = body.find("DESTINATION")
        if dest_idx >= 0:
            body = body[:dest_idx]
        # strip comments line by line
        cleaned_lines: list[str] = []
        for line in body.splitlines():
            comment_idx = line.find("#")
            if comment_idx >= 0:
                line = line[:comment_idx]
            cleaned_lines.append(line)
        tokens: list[str] = []
        for line in cleaned_lines:
            for tok in line.split():
                tok = tok.strip("()")
                if not tok:
                    continue
                tokens.append(tok)
        if tokens:
            blocks.append(tokens)
    return blocks


def test_cmake_install_programs_targets_exist(cmake_file: Path):
    text = cmake_file.read_text(encoding="utf-8")
    blocks = _extract_install_programs(text)
    if not blocks:
        pytest.skip("no install(PROGRAMS ...) blocks in this CMakeLists")
    missing: list[str] = []
    for tokens in blocks:
        for tok in tokens:
            # Resolve $-variables conservatively: skip if it contains '${'
            if "${" in tok:
                continue
            target = (cmake_file.parent / tok).resolve()
            if not target.is_file():
                missing.append(f"{tok} -> {target}")
    assert not missing, (
        f"{cmake_file}: install(PROGRAMS ...) references missing files:\n  "
        + "\n  ".join(missing)
    )
