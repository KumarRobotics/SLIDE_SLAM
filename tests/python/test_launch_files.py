"""Validate launch files conform to ROS2 conventions.

Mirrors sections E, I, and J of tests/static/check_ros2_port.sh.

What we check:
* zero `*.launch` XML files anywhere in the repo (excluding tools/ and tests/)
* every `*.launch.py` imports `LaunchDescription` from `launch`
* every `*.launch.py` defines `generate_launch_description`
* no `*.launch.py` contains a literal `<launch>` XML tag (sign of half port)
* no `nodelet_plugins.xml` files exist anywhere

What we DO NOT check:
* whether each launch file's referenced executable resolves at runtime
* parameter wiring or remap correctness
* TF tree consistency
"""
from __future__ import annotations

from pathlib import Path

import pytest


def find_launch_xml(repo_root: Path) -> list[Path]:
    out: list[Path] = []
    for p in repo_root.rglob("*.launch"):
        rel = p.relative_to(repo_root).as_posix()
        if rel.startswith(("tools/", "tests/")):
            continue
        out.append(p)
    return out


def find_launch_pys(repo_root: Path) -> list[Path]:
    out: list[Path] = []
    for sub in ("backend", "frontend"):
        for p in (repo_root / sub).rglob("*.launch.py"):
            out.append(p)
    return out


def test_no_xml_launch_files(repo_root: Path):
    leftovers = find_launch_xml(repo_root)
    assert not leftovers, (
        "Found XML .launch files (must be converted to .launch.py):\n  "
        + "\n  ".join(str(p) for p in leftovers)
    )


def test_no_nodelet_plugins_xml(repo_root: Path):
    leftovers = list(repo_root.rglob("nodelet_plugins.xml"))
    assert not leftovers, (
        "Found nodelet_plugins.xml file(s) — nodelets are not used in ROS2:\n  "
        + "\n  ".join(str(p) for p in leftovers)
    )


def pytest_generate_tests(metafunc):
    if "launch_py" in metafunc.fixturenames:
        repo_root = Path(__file__).resolve().parents[2]
        files = find_launch_pys(repo_root)
        ids = [str(p.relative_to(repo_root)) for p in files]
        metafunc.parametrize("launch_py", files, ids=ids)


def test_launch_py_imports_launchdescription(launch_py: Path):
    text = launch_py.read_text(encoding="utf-8")
    has_full = "from launch import LaunchDescription" in text
    has_partial = "from launch import" in text and "LaunchDescription" in text
    assert has_full or has_partial, (
        f"{launch_py}: missing 'from launch import LaunchDescription'"
    )


def test_launch_py_defines_generate_launch_description(launch_py: Path):
    text = launch_py.read_text(encoding="utf-8")
    assert "def generate_launch_description" in text, (
        f"{launch_py}: missing generate_launch_description function"
    )


def test_launch_py_no_xml_launch_tag(launch_py: Path):
    text = launch_py.read_text(encoding="utf-8")
    assert "<launch>" not in text, (
        f"{launch_py}: contains literal <launch> XML tag — half-converted"
    )
