"""Validate every reference to a sloam_msgs interface resolves to a real file.

Mirrors sections F and G of tests/static/check_ros2_port.sh.

What we check:
1. The 28 known .msg / 2 .srv / 2 .action interface files actually exist on
   disk under backend/sloam_msgs (defends against accidental deletion).
2. Every `#include <sloam_msgs/(msg|srv|action)/<snake>.hpp>` in active C++
   sources resolves to a real interface file (snake_case <-> CamelCase).
3. Every `from sloam_msgs.(msg|srv|action) import X` in active Python sources
   resolves to a real interface file.
4. No active source uses any of the 16 known camelCase field names from the
   ROS1 era — they should all have been renamed to snake_case during the port.
   The internal C++ struct PoseMstPair (in databaseManager.h) intentionally
   keeps fields like relativeRawOdomMotion / keyPose / poseTimestamp — these
   are NOT sloam_msgs fields and are excluded by virtue of \\b boundaries (the
   forbidden token `relativeRawOdom` does NOT match `relativeRawOdomMotion`
   because `M` is a word character).
"""
from __future__ import annotations

import re
from pathlib import Path

import pytest

from conftest import strip_cpp_comment, iter_code_lines


# ---- canonical interface tables (UpperCamelCase -> snake_case) ------------

KNOWN_MSGS = {
    "CubeMap": "cube_map",
    "CylinderMap": "cylinder_map",
    "InterRobotTf": "inter_robot_tf",
    "KeyPoses": "key_poses",
    "LoopClosure": "loop_closure",
    "MultiArrayPoseObjectEdges": "multi_array_pose_object_edges",
    "ObservationPair": "observation_pair",
    "PoseMst": "pose_mst",
    "PoseMstBundle": "pose_mst_bundle",
    "PoseObjectEdges": "pose_object_edges",
    "ROSCube": "ros_cube",
    "ROSCylinder": "ros_cylinder",
    "ROSCylinderArray": "ros_cylinder_array",
    "ROSEllipsoid": "ros_ellipsoid",
    "ROSGround": "ros_ground",
    "ROSObservation": "ros_observation",
    "ROSRangeBearing": "ros_range_bearing",
    "ROSRangeBearingSyncOdom": "ros_range_bearing_sync_odom",
    "ROSScan": "ros_scan",
    "ROSSubMap": "ros_sub_map",
    "ROSSweep": "ros_sweep",
    "ROSSyncOdom": "ros_sync_odom",
    "SemanticLoopClosure": "semantic_loop_closure",
    "SemanticMeasSyncOdom": "semantic_meas_sync_odom",
    "StampedRvizMarkerArray": "stamped_rviz_marker_array",
    "SyncPcOdom": "sync_pc_odom",
    "Vector4d": "vector4d",
    "Vector7d": "vector7d",
}

KNOWN_SRVS = {
    "EvaluateLoopClosure": "evaluate_loop_closure",
    "GraphTansmission": "graph_tansmission",  # sic — typo carried over from ROS1
}

KNOWN_ACTIONS = {
    "ActiveLoopClosure": "active_loop_closure",
    "DetectLoopClosure": "detect_loop_closure",
}

# camelCase fields whose ROS1 names should be gone after the rename pass
CAMEL_FIELDS = [
    "robotID",
    "hostRobotID",
    "targetRobotID",
    "TFfromTarget2Host",
    "objectType",
    "multiArrayEdges",
    "relativeRawOdom",  # NOT relativeRawOdomMotion (internal struct field)
    "poseMstPair",
    "map_of_labelXYZ",
    "interRobotTFs",
    "initialGuess",
    "treeModels",
    "treeFeatures",
    "groundFeatures",
    "labelXYZ",
    "relativeMotion",
]


# ---- 1: interface files exist on disk -------------------------------------

def test_known_msg_files_exist(repo_root: Path):
    msg_dir = repo_root / "backend" / "sloam_msgs" / "msg"
    missing = [name for name in KNOWN_MSGS if not (msg_dir / f"{name}.msg").is_file()]
    assert not missing, f"Missing .msg files: {missing}"


def test_known_srv_files_exist(repo_root: Path):
    srv_dir = repo_root / "backend" / "sloam_msgs" / "srv"
    missing = [name for name in KNOWN_SRVS if not (srv_dir / f"{name}.srv").is_file()]
    assert not missing, f"Missing .srv files: {missing}"


def test_known_action_files_exist(repo_root: Path):
    action_dir = repo_root / "backend" / "sloam_msgs" / "action"
    missing = [
        name for name in KNOWN_ACTIONS if not (action_dir / f"{name}.action").is_file()
    ]
    assert not missing, f"Missing .action files: {missing}"


# ---- 2: every C++ #include resolves --------------------------------------

CPP_INCLUDE_RE = re.compile(
    r"#include\s*<sloam_msgs/(msg|srv|action)/([a-z0-9_]+)\.hpp>"
)


def _snake_to_camel_lookup(kind: str) -> dict[str, str]:
    table = {"msg": KNOWN_MSGS, "srv": KNOWN_SRVS, "action": KNOWN_ACTIONS}[kind]
    return {snake: camel for camel, snake in table.items()}


def test_cpp_sloam_msgs_includes_resolve(repo_root: Path, active_cpp_files):
    bad: list[str] = []
    for path in active_cpp_files:
        text = path.read_text(errors="replace")
        for m in CPP_INCLUDE_RE.finditer(text):
            kind, snake = m.group(1), m.group(2)
            lookup = _snake_to_camel_lookup(kind)
            camel = lookup.get(snake)
            if camel is None:
                bad.append(f"{path}: unknown <sloam_msgs/{kind}/{snake}.hpp>")
                continue
            ext = {"msg": "msg", "srv": "srv", "action": "action"}[kind]
            target = repo_root / "backend" / "sloam_msgs" / kind / f"{camel}.{ext}"
            if not target.is_file():
                bad.append(f"{path}: <sloam_msgs/{kind}/{snake}.hpp> -> missing {target}")
    assert not bad, "\n".join(bad)


# ---- 3: every Python `from sloam_msgs.* import X` resolves ---------------

PY_IMPORT_RE = re.compile(
    r"^\s*from\s+sloam_msgs\.(msg|srv|action)\s+import\s+(.+?)(?:#.*)?$",
    re.MULTILINE,
)


def test_py_sloam_msgs_imports_resolve(repo_root: Path, active_py_files):
    bad: list[str] = []
    for path in active_py_files:
        text = path.read_text(errors="replace")
        for m in PY_IMPORT_RE.finditer(text):
            kind = m.group(1)
            names_blob = m.group(2)
            names = [n.strip().strip("()") for n in names_blob.split(",")]
            table = {"msg": KNOWN_MSGS, "srv": KNOWN_SRVS, "action": KNOWN_ACTIONS}[kind]
            ext = kind
            for name in names:
                if not name:
                    continue
                if name not in table:
                    bad.append(f"{path}: from sloam_msgs.{kind} import {name} (unknown)")
                    continue
                target = repo_root / "backend" / "sloam_msgs" / kind / f"{name}.{ext}"
                if not target.is_file():
                    bad.append(f"{path}: from sloam_msgs.{kind} import {name} -> missing {target}")
    assert not bad, "\n".join(bad)


# ---- 4: no camelCase field accessors remain ------------------------------

@pytest.mark.parametrize("field", CAMEL_FIELDS)
def test_no_camel_case_field_accessors(field: str, active_cpp_files, active_py_files):
    """No active source may use `.fieldName` or `->fieldName` for any of the
    camelCase fields the rename pass was supposed to eliminate.

    Note: \\b at the end of the field name is what excludes the legitimate
    internal struct field `relativeRawOdomMotion` (in databaseManager.h) from
    flagging the forbidden token `relativeRawOdom` — `M` is a word char so
    the boundary doesn't fire.
    """
    pattern = re.compile(rf"(?:\.|->){re.escape(field)}\b")
    hits: list[str] = []
    for path in list(active_cpp_files) + list(active_py_files):
        text = path.read_text(errors="replace")
        if path.suffix == ".py":
            for lineno, code in iter_code_lines(text):
                if pattern.search(code):
                    hits.append(f"{path}:{lineno}:{code.strip()}")
        else:
            for lineno, raw in enumerate(text.splitlines(), start=1):
                code = strip_cpp_comment(raw)
                if pattern.search(code):
                    hits.append(f"{path}:{lineno}:{code.strip()}")
    assert not hits, (
        f"{len(hits)} hit(s) for camelCase field .{field} (should be snake_case):\n  "
        + "\n  ".join(hits[:10])
    )
