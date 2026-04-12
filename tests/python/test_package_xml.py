"""Validate package.xml files match the ROS2 package format 3 conventions.

Mirrors section C of tests/static/check_ros2_port.sh.

What we check:
* file declares <package format="3">
* file declares an ament_* / rosidl buildtool_depend (NOT catkin)
* file declares <build_type>ament_cmake</build_type> or ament_python
* file does NOT reference message_generation / message_runtime (ROS1 only)

What we DO NOT check:
* completeness of <depend> entries — that's the colcon build's job
* whether dependencies are actually installable on Jazzy
"""
from __future__ import annotations

import re
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest


def find_package_xmls(repo_root: Path) -> list[Path]:
    out: list[Path] = []
    for sub in ("backend", "frontend"):
        for p in (repo_root / sub).rglob("package.xml"):
            # ignore vendored / out-of-tree
            if "clipper_semantic_object" in p.parts:
                continue
            out.append(p)
    return out


def pytest_generate_tests(metafunc):
    # Parameterise the per-package tests at collection time so that pytest
    # surfaces a separate test ID per package.xml.
    if "package_xml" in metafunc.fixturenames:
        repo_root = Path(__file__).resolve().parents[2]
        files = find_package_xmls(repo_root)
        ids = [str(p.relative_to(repo_root)) for p in files]
        metafunc.parametrize("package_xml", files, ids=ids)


def test_package_xml_format_is_3(package_xml: Path):
    text = package_xml.read_text(encoding="utf-8")
    assert '<package format="3">' in text, (
        f"{package_xml}: missing <package format=\"3\"> declaration"
    )
    assert 'format="2"' not in text, f"{package_xml}: legacy format=\"2\" still present"


def test_package_xml_no_catkin_buildtool(package_xml: Path):
    text = package_xml.read_text(encoding="utf-8")
    assert "<buildtool_depend>catkin</buildtool_depend>" not in text, (
        f"{package_xml}: still uses catkin buildtool_depend"
    )


def test_package_xml_has_ament_buildtool(package_xml: Path):
    text = package_xml.read_text(encoding="utf-8")
    pattern = re.compile(
        r"<buildtool_depend>(?:ament_cmake|ament_cmake_python|ament_python|rosidl_default_generators)</buildtool_depend>"
    )
    assert pattern.search(text), (
        f"{package_xml}: missing ament_* / rosidl buildtool_depend"
    )


def test_package_xml_has_build_type(package_xml: Path):
    text = package_xml.read_text(encoding="utf-8")
    pattern = re.compile(r"<build_type>(?:ament_cmake|ament_python)</build_type>")
    assert pattern.search(text), (
        f"{package_xml}: missing <build_type>ament_cmake|ament_python</build_type>"
    )


def test_package_xml_no_message_generation(package_xml: Path):
    text = package_xml.read_text(encoding="utf-8")
    assert "message_generation" not in text, (
        f"{package_xml}: references ROS1-only message_generation"
    )
    assert "message_runtime" not in text, (
        f"{package_xml}: references ROS1-only message_runtime"
    )


def test_package_xml_is_well_formed(package_xml: Path):
    """Cheap sanity: the file must parse as XML at all."""
    try:
        ET.parse(package_xml)
    except ET.ParseError as exc:
        pytest.fail(f"{package_xml}: malformed XML — {exc}")
