"""Detect any residual ROS1 idioms in active C++ and Python sources.

Mirrors sections A and B of tests/static/check_ros2_port.sh. The two layers
must stay in sync — if you add a forbidden pattern here, also add it to the
bash runner (and vice versa).

Rationale: the ros2_dev branch is a fresh port from ROS1 Noetic. The build
will fail if any of these slip through, but a static lint catches them faster
and gives a more legible error than a 200-line CMake/colcon log.
"""
from __future__ import annotations

import re
from pathlib import Path

import pytest

from conftest import strip_cpp_comment, iter_code_lines


# ---- C++ patterns ----------------------------------------------------------

CPP_FORBIDDEN = [
    ("ros/ros.h include",          re.compile(r"#include <ros/ros\.h>")),
    ("ros/package.h include",      re.compile(r"#include <ros/package\.h>")),
    ("old-style msg .h include",   re.compile(r"#include <(?:sensor_msgs|geometry_msgs|nav_msgs|std_msgs|visualization_msgs|sloam_msgs)/[A-Z][A-Za-z0-9_]*\.h>")),
    ("tf/ include",                re.compile(r"#include <tf/")),
    ("tf2 .h include (must .hpp)", re.compile(r"#include <tf2[^>]*\.h>")),
    ("nodelet/ include",           re.compile(r"#include <nodelet/")),
    ("pluginlib class_list_macros.h", re.compile(r"#include <pluginlib/class_list_macros\.h>")),
    ("actionlib/ include",         re.compile(r"#include <actionlib/")),
    ("ros::NodeHandle",            re.compile(r"\bros::NodeHandle\b")),
    ("ros::Publisher",             re.compile(r"\bros::Publisher\b")),
    ("ros::Subscriber",            re.compile(r"\bros::Subscriber\b")),
    ("ros::Time::now()",           re.compile(r"\bros::Time::now\(\)")),
    ("ros::Duration",              re.compile(r"\bros::Duration\b")),
    ("ros::Rate",                  re.compile(r"\bros::Rate\b")),
    ("ros::init",                  re.compile(r"\bros::init\(")),
    ("ros::spin/spinOnce",         re.compile(r"\bros::spin(?:Once)?\(\)")),
    ("ros::ok()",                  re.compile(r"\bros::ok\(\)")),
    ("ROS_LOG macros",             re.compile(r"\bROS_(?:INFO|WARN|ERROR|DEBUG|FATAL)(?:_STREAM|_THROTTLE|_STREAM_THROTTLE|_ONCE)?\b")),
    ("nodelet::Nodelet",           re.compile(r"\bnodelet::Nodelet\b")),
    ("PLUGINLIB_EXPORT_CLASS",     re.compile(r"\bPLUGINLIB_EXPORT_CLASS\b")),
    ("actionlib::",                re.compile(r"\bactionlib::")),
]


@pytest.mark.parametrize("name,pattern", CPP_FORBIDDEN, ids=[n for n, _ in CPP_FORBIDDEN])
def test_cpp_no_ros1_idioms(active_cpp_files, name, pattern):
    """No active C++ source under sloam/object_modeller may contain `name`."""
    hits: list[str] = []
    for path in active_cpp_files:
        try:
            text = path.read_text(errors="replace")
        except OSError:
            continue
        for lineno, raw in enumerate(text.splitlines(), start=1):
            code = strip_cpp_comment(raw)
            if pattern.search(code):
                hits.append(f"{path}:{lineno}:{code.strip()}")
    assert not hits, f"{len(hits)} {name} hit(s):\n  " + "\n  ".join(hits[:10])


# ---- Python patterns ------------------------------------------------------

PY_FORBIDDEN = [
    ("import rospy",         re.compile(r"^\s*(?:import\s+rospy|from\s+rospy)\b")),
    ("rospy.attr access",    re.compile(r"\brospy\.(?:Publisher|Subscriber|init_node|spin|Rate|loginfo|logwarn|logerr|logdebug|logfatal|get_param|has_param|set_param|Time|Duration|is_shutdown|on_shutdown|wait_for_message|wait_for_service|ServiceProxy|Service)\b")),
    ("bare 'import tf'",     re.compile(r"^\s*import\s+tf\s*$")),
    ("from tf.<sub>",        re.compile(r"^\s*from\s+tf\.")),
    ("ros_numpy",            re.compile(r"\bros_numpy\b")),
    ("rospkg.",              re.compile(r"\brospkg\.")),
]


@pytest.mark.parametrize("name,pattern", PY_FORBIDDEN, ids=[n for n, _ in PY_FORBIDDEN])
def test_py_no_ros1_idioms(active_py_files, name, pattern):
    """No active Python source under frontend/{object_modeller,scan2shape} may contain `name`.

    Uses iter_code_lines() so that triple-quoted docstrings (which often
    mention things like 'ros_numpy.numpify' as prose) don't false-positive.
    """
    hits: list[str] = []
    for path in active_py_files:
        try:
            text = path.read_text(errors="replace")
        except OSError:
            continue
        for lineno, code in iter_code_lines(text):
            if pattern.search(code):
                hits.append(f"{path}:{lineno}:{code.strip()}")
    assert not hits, f"{len(hits)} {name} hit(s):\n  " + "\n  ".join(hits[:10])
