#!/usr/bin/env bash
# tests/static/check_ros2_port.sh
#
# Static-check runner for the SlideSLAM ros2_dev branch (ROS1 Noetic -> ROS2 Jazzy port).
#
# Usage:
#   bash tests/static/check_ros2_port.sh [--verbose] [REPO_ROOT]
#
# Default REPO_ROOT is the parent of tests/, i.e. $(dirname $0)/../..
# Exits 0 if no failures, 1 otherwise.
#
# This script intentionally avoids Python and any ROS2 tooling. It relies on
# ripgrep (rg) when available and falls back to grep -r otherwise. find and awk
# from POSIX userland are also required.
#
# What this catches:
#   * residual ROS1 C++ idioms (rclcpp ports forgot to delete something)
#   * residual ROS1 Python idioms (rospy / tf / ros_numpy / rospkg)
#   * package.xml format=2 / catkin / message_generation leftovers
#   * CMakeLists.txt catkin leftovers and missing ament_package()
#   * .launch XML leftovers and broken .launch.py shells
#   * sloam_msgs camelCase field accessors (the rename table)
#   * sloam_msgs include-path / import-name typos
#   * install(PROGRAMS ...) targets that point at missing files
#
# What this does NOT catch:
#   * actual compilation / linking
#   * runtime topic/service/action behavior
#   * QoS, parameter, or TF correctness
#   * missing dependencies in CMakeLists target_link_libraries
#
# Add new checks at the bottom of the relevant section.

set -euo pipefail

# ---------- arg parsing ----------
VERBOSE=0
REPO_ROOT=""
for arg in "$@"; do
    case "$arg" in
        --verbose|-v)
            VERBOSE=1
            ;;
        -h|--help)
            sed -n '2,30p' "$0"
            exit 0
            ;;
        *)
            REPO_ROOT="$arg"
            ;;
    esac
done
export VERBOSE

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
if [ -z "$REPO_ROOT" ]; then
    REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
fi

if [ ! -d "$REPO_ROOT/backend" ] || [ ! -d "$REPO_ROOT/frontend" ]; then
    echo "ERROR: '$REPO_ROOT' does not look like the SlideSLAM repo root (no backend/ or frontend/)." >&2
    exit 2
fi

# shellcheck source=lib.sh
. "$SCRIPT_DIR/lib.sh"
pick_search_tool

printf "%sSlideSLAM ros2_dev static check%s\n" "${C_BOLD}" "${C_RESET}"
printf "Repo root: %s\n" "$REPO_ROOT"
printf "Search tool: %s\n" "$SEARCH_TOOL"
printf "Verbose: %s\n" "$VERBOSE"

# ---------- search roots ----------
SLOAM_SRC_GLOBS=(
    -g 'backend/sloam/src/**'
    -g 'backend/sloam/include/**'
    -g 'frontend/object_modeller/src/**'
    -g 'frontend/object_modeller/include/**'
    -g '!backend/sloam/clipper_semantic_object/**'
)

# Helper: rg with the C++ active-source globs (or grep fallback walking find).
cpp_search() {
    local pattern="$1"
    if [ "$SEARCH_TOOL" = "rg" ]; then
        rg -n --no-heading --color=never \
            -g 'backend/sloam/src/**' \
            -g 'backend/sloam/include/**' \
            -g 'frontend/object_modeller/src/**' \
            -g 'frontend/object_modeller/include/**' \
            -g '!backend/sloam/clipper_semantic_object/**' \
            -e "$pattern" "$REPO_ROOT" 2>/dev/null || true
    else
        find "$REPO_ROOT/backend/sloam/src" "$REPO_ROOT/backend/sloam/include" \
             "$REPO_ROOT/frontend/object_modeller/src" "$REPO_ROOT/frontend/object_modeller/include" \
             -type f \( -name '*.cpp' -o -name '*.h' -o -name '*.hpp' -o -name '*.cc' \) 2>/dev/null \
            | grep -v '/clipper_semantic_object/' \
            | xargs -r grep -EnH "$pattern" 2>/dev/null || true
    fi
}

py_search() {
    # Search active Python sources under frontend/{object_modeller,scan2shape}.
    # Preprocesses each file with awk to blank out triple-quoted docstrings,
    # then greps the result. This avoids false positives from prose like
    # "Replaces ros_numpy.numpify" living inside a function docstring.
    local pattern="$1"
    local files
    files=$(find "$REPO_ROOT/frontend/object_modeller" "$REPO_ROOT/frontend/scan2shape" \
                -type f -name '*.py' 2>/dev/null)
    [ -z "$files" ] && return 0
    while IFS= read -r f; do
        [ -z "$f" ] && continue
        awk '
            BEGIN { in_doc = 0; q = "" }
            {
                line = $0
                out = ""
                rest = line
                while (length(rest) > 0) {
                    if (in_doc) {
                        idx = index(rest, q)
                        if (idx == 0) { rest = ""; break }
                        rest = substr(rest, idx + 3)
                        in_doc = 0; q = ""
                    } else {
                        # find earliest """ or '\'\'\''
                        i1 = index(rest, "\"\"\"")
                        i2 = index(rest, "\x27\x27\x27")
                        if (i1 == 0 && i2 == 0) {
                            out = out rest
                            rest = ""
                            break
                        }
                        if (i1 != 0 && (i2 == 0 || i1 < i2)) {
                            out = out substr(rest, 1, i1 - 1)
                            rest = substr(rest, i1 + 3)
                            in_doc = 1; q = "\"\"\""
                        } else {
                            out = out substr(rest, 1, i2 - 1)
                            rest = substr(rest, i2 + 3)
                            in_doc = 1; q = "\x27\x27\x27"
                        }
                    }
                }
                printf "%s:%d:%s\n", FILENAME, NR, out
            }
        ' "$f" | grep -E "$pattern" 2>/dev/null || true
    done <<< "$files"
}

# Run a check that searches C++ active source for `pattern`, strips // and /* */
# comments, and fails if any code-line hit remains.
check_cpp_no_match() {
    local name="$1"
    local pattern="$2"
    local raw stripped hits
    raw="$(cpp_search "$pattern")"
    if [ -z "$raw" ]; then
        pass "$name"
        return 0
    fi
    stripped="$(printf "%s\n" "$raw" | strip_cpp_line_comments)"
    if [ -z "$stripped" ]; then
        pass "$name"
        return 0
    fi
    hits="$(printf "%s\n" "$stripped" | grep -E "$pattern" || true)"
    if [ -z "$hits" ]; then
        pass "$name"
        return 0
    fi
    local count sample
    count=$(printf "%s\n" "$hits" | wc -l | tr -d ' ')
    fail "$name" "$count code-line hit(s)"
    if [ "$VERBOSE" = "1" ]; then
        sample=$(printf "%s\n" "$hits" | head -5)
        printf "%s\n" "$sample" | sed 's/^/         /'
    fi
    return 0
}

check_py_no_match() {
    local name="$1"
    local pattern="$2"
    local raw stripped hits
    raw="$(py_search "$pattern")"
    if [ -z "$raw" ]; then
        pass "$name"
        return 0
    fi
    stripped="$(printf "%s\n" "$raw" | strip_python_line_comments)"
    if [ -z "$stripped" ]; then
        pass "$name"
        return 0
    fi
    hits="$(printf "%s\n" "$stripped" | grep -E "$pattern" || true)"
    if [ -z "$hits" ]; then
        pass "$name"
        return 0
    fi
    local count sample
    count=$(printf "%s\n" "$hits" | wc -l | tr -d ' ')
    fail "$name" "$count code-line hit(s)"
    if [ "$VERBOSE" = "1" ]; then
        sample=$(printf "%s\n" "$hits" | head -5)
        printf "%s\n" "$sample" | sed 's/^/         /'
    fi
    return 0
}

# ============================================================================
# A. ROS1 C++ idioms in active source
# ============================================================================
section "A. ROS1 C++ idioms"

check_cpp_no_match "A1  no #include <ros/ros.h>"             '#include <ros/ros\.h>'
check_cpp_no_match "A2  no #include <ros/package.h>"          '#include <ros/package\.h>'
check_cpp_no_match "A3  no old-style msg .h includes"         '#include <(sensor_msgs|geometry_msgs|nav_msgs|std_msgs|visualization_msgs|sloam_msgs)/[A-Z][A-Za-z0-9_]*\.h>'
check_cpp_no_match "A4  no #include <tf/...>"                 '#include <tf/'
check_cpp_no_match "A5  no #include <tf2*.h> (must be .hpp)"  '#include <tf2[^>]*\.h>'
check_cpp_no_match "A6  no #include <nodelet/...>"            '#include <nodelet/'
check_cpp_no_match "A7  no pluginlib/class_list_macros.h"     '#include <pluginlib/class_list_macros\.h>'
check_cpp_no_match "A8  no #include <actionlib/...>"          '#include <actionlib/'
check_cpp_no_match "A9  no ros::NodeHandle"                   '\bros::NodeHandle\b'
check_cpp_no_match "A10 no ros::Publisher"                    '\bros::Publisher\b'
check_cpp_no_match "A11 no ros::Subscriber"                   '\bros::Subscriber\b'
check_cpp_no_match "A12 no ros::Time::now()"                  '\bros::Time::now\(\)'
check_cpp_no_match "A13 no ros::Duration"                     '\bros::Duration\b'
check_cpp_no_match "A14 no ros::Rate"                         '\bros::Rate\b'
check_cpp_no_match "A15 no ros::init"                         '\bros::init\('
check_cpp_no_match "A16 no ros::spin / spinOnce"              '\bros::spin(Once)?\(\)'
check_cpp_no_match "A17 no ros::ok()"                         '\bros::ok\(\)'
check_cpp_no_match "A18 no ROS_INFO/WARN/ERROR/DEBUG/FATAL"   '\bROS_(INFO|WARN|ERROR|DEBUG|FATAL)(_STREAM|_THROTTLE|_STREAM_THROTTLE|_ONCE)?\b'
check_cpp_no_match "A19 no nodelet::Nodelet"                  '\bnodelet::Nodelet\b'
check_cpp_no_match "A20 no PLUGINLIB_EXPORT_CLASS"            '\bPLUGINLIB_EXPORT_CLASS\b'
check_cpp_no_match "A21 no actionlib::"                       '\bactionlib::'

# ============================================================================
# B. ROS1 Python idioms
# ============================================================================
section "B. ROS1 Python idioms"

check_py_no_match "B1 no 'import rospy' / 'from rospy'"  '^[[:space:]]*(import rospy|from rospy)\b'
check_py_no_match "B2 no rospy.* attribute access"        '\brospy\.(Publisher|Subscriber|init_node|spin|Rate|loginfo|logwarn|logerr|logdebug|logfatal|get_param|has_param|set_param|Time|Duration|is_shutdown|on_shutdown|wait_for_message|wait_for_service|ServiceProxy|Service)\b'
check_py_no_match "B3 no bare 'import tf'"                '^[[:space:]]*import tf$'
check_py_no_match "B4 no 'from tf.<sub>'"                 '^[[:space:]]*from tf\.'
check_py_no_match "B5 no ros_numpy usage"                 '\bros_numpy\b'
check_py_no_match "B6 no rospkg usage"                    '\brospkg\.'

# ============================================================================
# C. package.xml format
# ============================================================================
section "C. package.xml format"

check_package_xml() {
    local pkg_xml="$1"
    local rel="${pkg_xml#$REPO_ROOT/}"
    local name="C  $rel"
    local content
    content="$(tr -d '\r' < "$pkg_xml")"

    if ! printf "%s\n" "$content" | grep -q '<package format="3">'; then
        fail "$name" "missing <package format=\"3\">"
        return
    fi
    if printf "%s\n" "$content" | grep -q 'format="2"'; then
        fail "$name" "found legacy format=\"2\""
        return
    fi
    if printf "%s\n" "$content" | grep -q '<buildtool_depend>catkin</buildtool_depend>'; then
        fail "$name" "found <buildtool_depend>catkin</buildtool_depend>"
        return
    fi
    if ! printf "%s\n" "$content" | grep -qE '<buildtool_depend>(ament_cmake|ament_cmake_python|ament_python|rosidl_default_generators)</buildtool_depend>'; then
        fail "$name" "missing ament_cmake/ament_python/rosidl buildtool_depend"
        return
    fi
    if ! printf "%s\n" "$content" | grep -qE '<build_type>(ament_cmake|ament_python)</build_type>'; then
        fail "$name" "missing <build_type>ament_*</build_type>"
        return
    fi
    if printf "%s\n" "$content" | grep -qE '\bmessage_generation\b|\bmessage_runtime\b'; then
        fail "$name" "references message_generation/message_runtime"
        return
    fi
    pass "$name"
}

while IFS= read -r f; do
    check_package_xml "$f"
done < <(find "$REPO_ROOT/backend" "$REPO_ROOT/frontend" -name package.xml -type f 2>/dev/null)

# ============================================================================
# D. CMakeLists catkin leftovers + ament_package presence
# ============================================================================
section "D. CMakeLists catkin leftovers"

check_cmake() {
    local cml="$1"
    local rel="${cml#$REPO_ROOT/}"
    local name="D  $rel"
    local content
    content="$(tr -d '\r' < "$cml")"

    if printf "%s\n" "$content" | grep -qE 'find_package\(catkin\b'; then
        fail "$name" "found find_package(catkin ...)"; return
    fi
    if printf "%s\n" "$content" | grep -qE '\bcatkin_package\('; then
        fail "$name" "found catkin_package(...)"; return
    fi
    if printf "%s\n" "$content" | grep -qE '\bcatkin_INCLUDE_DIRS\b'; then
        fail "$name" "uses catkin_INCLUDE_DIRS"; return
    fi
    if printf "%s\n" "$content" | grep -qE '\bcatkin_LIBRARIES\b'; then
        fail "$name" "uses catkin_LIBRARIES"; return
    fi
    if printf "%s\n" "$content" | grep -qE '\b(add_message_files|add_service_files|add_action_files|generate_messages)\b'; then
        fail "$name" "uses add_message_files/add_service_files/add_action_files/generate_messages"; return
    fi
    if ! printf "%s\n" "$content" | grep -qE '^[[:space:]]*ament_package\(\)'; then
        fail "$name" "missing ament_package() call"; return
    fi
    pass "$name"
}

# Iterate package CMakeLists only (skip clipper_semantic_object/).
while IFS= read -r f; do
    case "$f" in
        */clipper_semantic_object/*) continue ;;
    esac
    check_cmake "$f"
done < <(find "$REPO_ROOT/backend" "$REPO_ROOT/frontend" -name CMakeLists.txt -type f 2>/dev/null)

# ============================================================================
# E. Launch files
# ============================================================================
section "E. Launch files"

# E1: zero .launch XML files anywhere under backend/ or frontend/
LAUNCH_XML=$(find "$REPO_ROOT/backend" "$REPO_ROOT/frontend" -type f -name '*.launch' 2>/dev/null || true)
if [ -z "$LAUNCH_XML" ]; then
    pass "E1  no XML .launch files under backend/ or frontend/"
else
    cnt=$(printf "%s\n" "$LAUNCH_XML" | wc -l | tr -d ' ')
    fail "E1  no XML .launch files under backend/ or frontend/" "$cnt file(s) remain"
fi

# E2..E4: every .launch.py must contain LaunchDescription import + generator
# function and must NOT contain a literal <launch> XML tag (sign of half port).
LAUNCH_PYS=$(find "$REPO_ROOT/backend" "$REPO_ROOT/frontend" -type f -name '*.launch.py' 2>/dev/null || true)
missing_import=0
missing_def=0
has_xml_tag=0
for lp in $LAUNCH_PYS; do
    if ! grep -q 'from launch import LaunchDescription' "$lp"; then
        missing_import=$((missing_import + 1))
    fi
    if ! grep -q 'def generate_launch_description' "$lp"; then
        missing_def=$((missing_def + 1))
    fi
    if grep -q '<launch>' "$lp"; then
        has_xml_tag=$((has_xml_tag + 1))
    fi
done

if [ "$missing_import" -eq 0 ]; then
    pass "E2  every .launch.py imports LaunchDescription"
else
    fail "E2  every .launch.py imports LaunchDescription" "$missing_import file(s) missing import"
fi
if [ "$missing_def" -eq 0 ]; then
    pass "E3  every .launch.py defines generate_launch_description"
else
    fail "E3  every .launch.py defines generate_launch_description" "$missing_def file(s) missing function"
fi
if [ "$has_xml_tag" -eq 0 ]; then
    pass "E4  no .launch.py contains literal <launch> XML tag"
else
    fail "E4  no .launch.py contains literal <launch> XML tag" "$has_xml_tag file(s) contain XML"
fi

# ============================================================================
# F. sloam_msgs camelCase field accessors
# ============================================================================
section "F. sloam_msgs camelCase field accessors"

# Each entry is "FieldName" — accessed as .Field or ->Field. We DO NOT include
# relativeRawOdom-without-Motion separately because \b after the regex naturally
# excludes relativeRawOdomMotion (M is a word char, so \b doesn't fire).
CAMEL_FIELDS=(
    "robotID"
    "hostRobotID"
    "targetRobotID"
    "TFfromTarget2Host"
    "objectType"
    "multiArrayEdges"
    "relativeRawOdom"
    "poseMstPair"
    "map_of_labelXYZ"
    "interRobotTFs"
    "initialGuess"
    "treeModels"
    "treeFeatures"
    "groundFeatures"
    "labelXYZ"
    "relativeMotion"
)

f_total_hits=0
for fname in "${CAMEL_FIELDS[@]}"; do
    pat="(\.|->)${fname}\\b"
    raw=""
    if [ "$SEARCH_TOOL" = "rg" ]; then
        raw="$(rg -n --no-heading --color=never \
            -g 'backend/sloam/**/*.cpp' -g 'backend/sloam/**/*.h' -g 'backend/sloam/**/*.hpp' \
            -g 'frontend/object_modeller/**/*.cpp' -g 'frontend/object_modeller/**/*.h' -g 'frontend/object_modeller/**/*.hpp' -g 'frontend/object_modeller/**/*.py' \
            -g 'frontend/scan2shape/**/*.py' \
            -g '!backend/sloam/clipper_semantic_object/**' \
            -e "$pat" "$REPO_ROOT" 2>/dev/null || true)"
    else
        raw="$(find "$REPO_ROOT/backend/sloam" "$REPO_ROOT/frontend/object_modeller" "$REPO_ROOT/frontend/scan2shape" \
              -type f \( -name '*.cpp' -o -name '*.h' -o -name '*.hpp' -o -name '*.py' \) 2>/dev/null \
            | grep -v '/clipper_semantic_object/' \
            | xargs -r grep -EnH "$pat" 2>/dev/null || true)"
    fi
    [ -z "$raw" ] && continue

    # Strip C++/Python comment lines.
    cpp_part="$(printf "%s\n" "$raw" | grep -E '\.(cpp|h|hpp):' || true)"
    py_part="$(printf "%s\n" "$raw" | grep -E '\.py:' || true)"
    cpp_clean="$(printf "%s\n" "$cpp_part" | strip_cpp_line_comments | grep -E "$pat" || true)"
    py_clean="$(printf "%s\n" "$py_part"  | strip_python_line_comments | grep -E "$pat" || true)"
    combined="$(printf "%s\n%s\n" "$cpp_clean" "$py_clean" | grep -v '^$' || true)"
    [ -z "$combined" ] && continue

    cnt=$(printf "%s\n" "$combined" | wc -l | tr -d ' ')
    f_total_hits=$((f_total_hits + cnt))
    sample=$(printf "%s\n" "$combined" | head -3)
    if [ "$VERBOSE" = "1" ]; then
        fail "F  camelCase field .$fname (snake_case rename missed)" "$cnt hit(s)" \
            $(printf "%s\n" "$sample")
    else
        fail "F  camelCase field .$fname (snake_case rename missed)" "$cnt hit(s)"
    fi
done
if [ "$f_total_hits" -eq 0 ]; then
    pass "F  no camelCase sloam_msgs field accessors remain"
fi

# ============================================================================
# G. sloam_msgs include / import name consistency
# ============================================================================
section "G. sloam_msgs include / import consistency"

# Hard-coded UpperCamelCase <-> snake_case map. Single source of truth, also
# duplicated in tests/python/test_sloam_msgs_consistency.py — keep them in sync.
SLOAM_MSGS=(
    "CubeMap=cube_map"
    "CylinderMap=cylinder_map"
    "InterRobotTf=inter_robot_tf"
    "KeyPoses=key_poses"
    "LoopClosure=loop_closure"
    "MultiArrayPoseObjectEdges=multi_array_pose_object_edges"
    "ObservationPair=observation_pair"
    "PoseMst=pose_mst"
    "PoseMstBundle=pose_mst_bundle"
    "PoseObjectEdges=pose_object_edges"
    "ROSCube=ros_cube"
    "ROSCylinder=ros_cylinder"
    "ROSCylinderArray=ros_cylinder_array"
    "ROSEllipsoid=ros_ellipsoid"
    "ROSGround=ros_ground"
    "ROSObservation=ros_observation"
    "ROSRangeBearing=ros_range_bearing"
    "ROSRangeBearingSyncOdom=ros_range_bearing_sync_odom"
    "ROSScan=ros_scan"
    "ROSSubMap=ros_sub_map"
    "ROSSweep=ros_sweep"
    "ROSSyncOdom=ros_sync_odom"
    "SemanticLoopClosure=semantic_loop_closure"
    "SemanticMeasSyncOdom=semantic_meas_sync_odom"
    "StampedRvizMarkerArray=stamped_rviz_marker_array"
    "SyncPcOdom=sync_pc_odom"
    "Vector4d=vector4d"
    "Vector7d=vector7d"
)
SLOAM_SRVS=(
    "EvaluateLoopClosure=evaluate_loop_closure"
    "GraphTansmission=graph_tansmission"  # sic — typo carried over from ROS1
)
SLOAM_ACTIONS=(
    "ActiveLoopClosure=active_loop_closure"
    "DetectLoopClosure=detect_loop_closure"
)

# Build snake-> exists mapping by simply listing the on-disk files. We then
# verify each #include / import resolves to a real file.
SLOAM_MSGS_DIR="$REPO_ROOT/backend/sloam_msgs"

# G1: every #include <sloam_msgs/(msg|srv|action)/<snake>.hpp> resolves
g1_bad=0
g1_examples=()
while IFS= read -r line; do
    # line: file:linenum:#include <sloam_msgs/msg/foo.hpp>  (file may be a Windows path)
    # Strip trailing :linenum:text — we only need the file part for error context.
    file_part=$(printf "%s" "$line" | sed -E 's/:[0-9]+:.*$//')
    # Use # as the sed delimiter so the alternation pipes inside the regex are unambiguous.
    incl=$(printf "%s" "$line" | sed -E 's#.*sloam_msgs/(msg|srv|action)/([a-z0-9_]+)\.hpp.*#\1/\2#')
    type_part="${incl%%/*}"
    snake="${incl##*/}"
    case "$type_part" in
        msg) target_dir="$SLOAM_MSGS_DIR/msg"; ext="msg" ;;
        srv) target_dir="$SLOAM_MSGS_DIR/srv"; ext="srv" ;;
        action) target_dir="$SLOAM_MSGS_DIR/action"; ext="action" ;;
        *) continue ;;
    esac
    # Map snake -> CamelCase via the lookup tables.
    found=0
    for entry in "${SLOAM_MSGS[@]}" "${SLOAM_SRVS[@]}" "${SLOAM_ACTIONS[@]}"; do
        camel="${entry%=*}"
        snk="${entry##*=}"
        if [ "$snk" = "$snake" ] && [ -f "$target_dir/$camel.$ext" ]; then
            found=1; break
        fi
    done
    if [ "$found" -eq 0 ]; then
        g1_bad=$((g1_bad + 1))
        g1_examples+=("$file_part: <sloam_msgs/$type_part/$snake.hpp>")
    fi
done < <(
    if [ "$SEARCH_TOOL" = "rg" ]; then
        rg -n --no-heading --color=never \
            -g '*.cpp' -g '*.h' -g '*.hpp' \
            -g '!backend/sloam/clipper_semantic_object/**' \
            -e '#include <sloam_msgs/(msg|srv|action)/[a-z0-9_]+\.hpp>' "$REPO_ROOT" 2>/dev/null || true
    else
        find "$REPO_ROOT/backend" "$REPO_ROOT/frontend" \
            -type f \( -name '*.cpp' -o -name '*.h' -o -name '*.hpp' \) \
            | grep -v '/clipper_semantic_object/' \
            | xargs -r grep -EnH '#include <sloam_msgs/(msg|srv|action)/[a-z0-9_]+\.hpp>' 2>/dev/null || true
    fi
)
if [ "$g1_bad" -eq 0 ]; then
    pass "G1  every #include <sloam_msgs/...> resolves to a real file"
else
    fail "G1  every #include <sloam_msgs/...> resolves to a real file" "$g1_bad missing"
    if [ "$VERBOSE" = "1" ]; then
        for e in "${g1_examples[@]}"; do printf "         %s\n" "$e"; done
    fi
fi

# G2: every `from sloam_msgs.{msg,srv,action} import X` resolves
g2_bad=0
g2_examples=()
while IFS= read -r line; do
    file_part=$(printf "%s" "$line" | sed -E 's/:[0-9]+:.*$//')
    rest=$(printf "%s" "$line" | sed -E 's/^.*:[0-9]+://')
    type_part=$(printf "%s" "$rest" | sed -E 's/.*from sloam_msgs\.(msg|srv|action)[[:space:]]+import[[:space:]]+.*/\1/')
    names=$(printf "%s" "$rest" | sed -E 's/.*import[[:space:]]+//; s/#.*//; s/,/ /g')
    case "$type_part" in
        msg) target_dir="$SLOAM_MSGS_DIR/msg"; ext="msg" ;;
        srv) target_dir="$SLOAM_MSGS_DIR/srv"; ext="srv" ;;
        action) target_dir="$SLOAM_MSGS_DIR/action"; ext="action" ;;
        *) continue ;;
    esac
    for nm in $names; do
        nm=$(printf "%s" "$nm" | tr -d ' ()')
        [ -z "$nm" ] && continue
        if [ ! -f "$target_dir/$nm.$ext" ]; then
            g2_bad=$((g2_bad + 1))
            g2_examples+=("$file_part: from sloam_msgs.$type_part import $nm")
        fi
    done
done < <(
    if [ "$SEARCH_TOOL" = "rg" ]; then
        rg -n --no-heading --color=never \
            -g '*.py' \
            -e '^[[:space:]]*from sloam_msgs\.(msg|srv|action)\s+import\b' "$REPO_ROOT" 2>/dev/null || true
    else
        find "$REPO_ROOT" -type f -name '*.py' 2>/dev/null \
            | xargs -r grep -EnH '^[[:space:]]*from sloam_msgs\.(msg|srv|action)[[:space:]]+import\b' 2>/dev/null || true
    fi
)
if [ "$g2_bad" -eq 0 ]; then
    pass "G2  every 'from sloam_msgs.* import X' resolves to a real interface"
else
    fail "G2  every 'from sloam_msgs.* import X' resolves to a real interface" "$g2_bad missing"
    if [ "$VERBOSE" = "1" ]; then
        for e in "${g2_examples[@]}"; do printf "         %s\n" "$e"; done
    fi
fi

# ============================================================================
# H. install(PROGRAMS ...) targets exist on disk
# ============================================================================
section "H. install(PROGRAMS ...) targets exist"

h_total_missing=0
h_examples=()
while IFS= read -r cml; do
    case "$cml" in */clipper_semantic_object/*) continue ;; esac
    cml_dir=$(dirname "$cml")
    # Extract install(PROGRAMS ... DESTINATION ...) blocks via awk.
    awk '
        BEGIN { in_block = 0 }
        /install[[:space:]]*\([[:space:]]*PROGRAMS/ { in_block = 1; sub(/.*PROGRAMS/, ""); }
        in_block {
            line = $0
            sub(/#.*/, "", line)
            n = split(line, toks, /[[:space:]]+/)
            for (i = 1; i <= n; i++) {
                t = toks[i]
                if (t == "" || t == "(" || t == ")") continue
                if (t == "DESTINATION") { in_block = 0; break }
                if (t ~ /^DESTINATION/) { in_block = 0; break }
                if (t == "PROGRAMS") continue
                # strip leading paren
                gsub(/^\(/, "", t)
                gsub(/\)$/, "", t)
                if (t == "") continue
                print t
            }
            if (line ~ /\)/) in_block = 0
        }
    ' "$cml" 2>/dev/null | while IFS= read -r tok; do
        [ -z "$tok" ] && continue
        # Resolve relative to the CMakeLists directory.
        case "$tok" in
            /*) abspath="$tok" ;;
            *)  abspath="$cml_dir/$tok" ;;
        esac
        # Normalise: collapse /./ and /../ via realpath if available, else manual.
        if command -v realpath >/dev/null 2>&1; then
            abspath_norm="$(realpath -m "$abspath" 2>/dev/null || echo "$abspath")"
        else
            abspath_norm="$abspath"
        fi
        if [ ! -f "$abspath_norm" ]; then
            printf "MISSING\t%s\t%s\n" "$cml" "$tok"
        fi
    done || true
done < <(find "$REPO_ROOT/backend" "$REPO_ROOT/frontend" -name CMakeLists.txt -type f 2>/dev/null) > /tmp/slideslam_h_check.$$ 2>/dev/null || true

if [ -s /tmp/slideslam_h_check.$$ ]; then
    h_total_missing=$(wc -l < /tmp/slideslam_h_check.$$ | tr -d ' ')
    fail "H  install(PROGRAMS ...) targets exist on disk" "$h_total_missing missing"
    if [ "$VERBOSE" = "1" ]; then
        while IFS= read -r line; do
            printf "         %s\n" "$line"
        done < /tmp/slideslam_h_check.$$
    fi
else
    pass "H  install(PROGRAMS ...) targets exist on disk"
fi
rm -f /tmp/slideslam_h_check.$$

# ============================================================================
# I. nodelet_plugins.xml removed
# ============================================================================
section "I. nodelet_plugins.xml removed"

NPX=$(find "$REPO_ROOT" -type f -name 'nodelet_plugins.xml' 2>/dev/null || true)
if [ -z "$NPX" ]; then
    pass "I  no nodelet_plugins.xml anywhere in repo"
else
    cnt=$(printf "%s\n" "$NPX" | wc -l | tr -d ' ')
    fail "I  no nodelet_plugins.xml anywhere in repo" "$cnt file(s) found"
fi

# ============================================================================
# J. .launch XML leftovers anywhere (excluding tools/ and tests/)
# ============================================================================
section "J. .launch XML leftovers"

LAUNCH_ALL=$(find "$REPO_ROOT" -type f -name '*.launch' 2>/dev/null \
    | grep -v -E '/(tools|tests)/' || true)
if [ -z "$LAUNCH_ALL" ]; then
    pass "J  no XML .launch files outside tools/ or tests/"
else
    cnt=$(printf "%s\n" "$LAUNCH_ALL" | wc -l | tr -d ' ')
    fail "J  no XML .launch files outside tools/ or tests/" "$cnt file(s) remain"
fi

# ============================================================================
# K. Launch-argument consistency (within-file only)
# ============================================================================
section "K. Launch-argument consistency"
#
# For every .launch.py: collect the set of LaunchConfiguration('x') strings and
# the set of DeclareLaunchArgument('x', ...) strings. FAIL if any LaunchConfig
# key is referenced without being declared in the SAME file.
#
# We don't follow IncludeLaunchDescription chains — that's hard to do in shell
# and would produce false positives for args that are declared in a parent
# launch file. The within-file check catches typos like
# LaunchConfiguration('robot_namee') when the arg was declared as 'robot_name'.
#
# ROS2 injects a handful of launch-level variables that are always available
# (log_level, launch_prefix, etc.). Treat those as pre-declared.

if ! command -v awk >/dev/null 2>&1; then
    skip "K  launch-argument consistency" "awk not available"
else
    # Known well-known launch-level variables that ROS2 / launch_ros injects.
    K_BUILTIN_ARGS="launch_prefix log_level use_sim_time output_log_level launch_prefix_filter node_log_level"

    k_total_missing=0
    k_files_with_issues=0
    k_examples=()
    while IFS= read -r lp; do
        [ -z "$lp" ] && continue
        # Read the whole file as a single awk record so multi-line
        # DeclareLaunchArgument( / LaunchConfiguration( calls are matched
        # correctly. Earlier per-line processing missed the common
        #     DeclareLaunchArgument(
        #         'name',
        #         default_value='...',
        #     ),
        # pattern and produced false positives.
        missing_raw=$(awk -v builtin="$K_BUILTIN_ARGS" '
            BEGIN {
                RS = "\x04"   # EOT: no regular file contains this, so RS effectively slurps the whole file
                n = split(builtin, arr, " ")
                for (i = 1; i <= n; i++) builtins[arr[i]] = 1
            }
            {
                text = $0
                # Collect DeclareLaunchArgument(..., "name", ...) names.
                # The paren may be followed by arbitrary whitespace/newlines
                # before the first positional string arg.
                tmp = text
                while (match(tmp, /DeclareLaunchArgument[[:space:]]*\([[:space:]\n]*[\x27"][a-zA-Z_][a-zA-Z0-9_]*[\x27"]/)) {
                    chunk = substr(tmp, RSTART, RLENGTH)
                    if (match(chunk, /[\x27"][a-zA-Z_][a-zA-Z0-9_]*[\x27"]/)) {
                        name = substr(chunk, RSTART + 1, RLENGTH - 2)
                        declared[name] = 1
                    }
                    tmp = substr(tmp, RSTART + RLENGTH)
                }
                # Collect LaunchConfiguration("x") / LaunchConfiguration(\x27x\x27).
                # These are always single-line in practice, but allow for
                # whitespace/newlines inside the parens just in case.
                tmp = text
                while (match(tmp, /LaunchConfiguration[[:space:]]*\([[:space:]\n]*[\x27"][a-zA-Z_][a-zA-Z0-9_]*[\x27"][[:space:]\n]*\)/)) {
                    chunk = substr(tmp, RSTART, RLENGTH)
                    if (match(chunk, /[\x27"][a-zA-Z_][a-zA-Z0-9_]*[\x27"]/)) {
                        name = substr(chunk, RSTART + 1, RLENGTH - 2)
                        # Compute a 1-indexed line number by counting newlines
                        # in the prefix of the whole-file slurp. This is O(n)
                        # per match but our launch files are small.
                        prefix_len = RSTART - 1
                        prefix = substr(text, 1, prefix_len)
                        lineno = gsub(/\n/, "\n", prefix) + 1
                        # Only record the first occurrence per name.
                        if (!(name in used)) used[name] = lineno
                    }
                    tmp = substr(tmp, RSTART + RLENGTH)
                }
            }
            END {
                for (k in used) {
                    if (!(k in declared) && !(k in builtins)) {
                        printf "%s:%d\n", k, used[k]
                    }
                }
            }
        ' "$lp")
        if [ -n "$missing_raw" ]; then
            cnt=$(printf "%s\n" "$missing_raw" | wc -l | tr -d ' ')
            k_total_missing=$((k_total_missing + cnt))
            k_files_with_issues=$((k_files_with_issues + 1))
            rel="${lp#$REPO_ROOT/}"
            while IFS= read -r miss; do
                [ -z "$miss" ] && continue
                k_examples+=("$rel: LaunchConfiguration('${miss%%:*}') not declared (line ${miss##*:})")
            done <<< "$missing_raw"
        fi
    done < <(find "$REPO_ROOT/backend" "$REPO_ROOT/frontend" -type f -name '*.launch.py' 2>/dev/null | grep -v '/tests/')

    if [ "$k_total_missing" -eq 0 ]; then
        pass "K  every LaunchConfiguration(x) has a matching DeclareLaunchArgument(x) in the same file"
    else
        fail "K  every LaunchConfiguration(x) has a matching DeclareLaunchArgument(x) in the same file" \
            "$k_total_missing undeclared reference(s) across $k_files_with_issues file(s)"
        if [ "$VERBOSE" = "1" ]; then
            for e in "${k_examples[@]}"; do printf "         %s\n" "$e"; done
        fi
    fi
fi

# ============================================================================
# L. Hardcoded user-specific absolute paths in source
# ============================================================================
section "L. Hardcoded user-specific absolute paths"
#
# Grep .cpp/.h/.hpp/.py under the main source roots for patterns that look
# like hardcoded user-specific Linux paths. Comment-stripped. These break for
# anyone except the original author — they should be configurable via
# parameters or launch arguments.

# The regex we search for. Use single pattern matched with ERE, then strip
# comments, then re-match to survive false positives.
L_PATTERN='/home/[a-z0-9_]+/|/opt/slideslam_docker_ws|/opt/bags/|/root/'

l_search() {
    # Collect source files from the given roots. Uses find + grep for
    # robustness. Returns "file:line:text" records.
    local roots=(
        "$REPO_ROOT/backend/sloam/src"
        "$REPO_ROOT/backend/sloam/include"
        "$REPO_ROOT/frontend/object_modeller/src"
        "$REPO_ROOT/frontend/object_modeller/include"
        "$REPO_ROOT/frontend/object_modeller/script"
        "$REPO_ROOT/frontend/object_modeller/object_detector_utils"
        "$REPO_ROOT/frontend/scan2shape/script"
        "$REPO_ROOT/frontend/scan2shape/scan2shape_launch/script"
    )
    if [ "$SEARCH_TOOL" = "rg" ]; then
        rg -n --no-heading --color=never \
            -g '*.cpp' -g '*.h' -g '*.hpp' -g '*.cc' -g '*.py' \
            -g '!*.launch.py' \
            -g '!backend/sloam/clipper_semantic_object/**' \
            -e "$L_PATTERN" "${roots[@]}" 2>/dev/null || true
    else
        find "${roots[@]}" -type f \
            \( -name '*.cpp' -o -name '*.h' -o -name '*.hpp' -o -name '*.cc' -o -name '*.py' \) \
            2>/dev/null \
            | grep -v '/clipper_semantic_object/' \
            | grep -v '\.launch\.py$' \
            | xargs -r grep -EnH "$L_PATTERN" 2>/dev/null || true
    fi
}

l_raw="$(l_search)"
if [ -z "$l_raw" ]; then
    pass "L  no hardcoded user-specific absolute paths in source"
else
    # Split into cpp and py parts, strip comments, re-match.
    l_cpp_part="$(printf "%s\n" "$l_raw" | grep -E '\.(cpp|h|hpp|cc):' || true)"
    l_py_part="$(printf "%s\n" "$l_raw" | grep -E '\.py:' || true)"
    l_cpp_clean="$(printf "%s\n" "$l_cpp_part" | strip_cpp_line_comments | grep -E "$L_PATTERN" || true)"
    l_py_clean="$(printf "%s\n" "$l_py_part"  | strip_python_line_comments | grep -E "$L_PATTERN" || true)"
    l_combined="$(printf "%s\n%s\n" "$l_cpp_clean" "$l_py_clean" | grep -v '^$' || true)"
    if [ -z "$l_combined" ]; then
        pass "L  no hardcoded user-specific absolute paths in source"
    else
        l_count=$(printf "%s\n" "$l_combined" | wc -l | tr -d ' ')
        fail "L  no hardcoded user-specific absolute paths in source" "$l_count code-line hit(s)"
        if [ "$VERBOSE" = "1" ]; then
            printf "%s\n" "$l_combined" | head -10 | sed 's/^/         /'
        fi
    fi
fi

# ============================================================================
# M. Launch Node(executable=...) resolves in target package
# ============================================================================
section "M. Launch Node executables resolve"
#
# For every .launch.py, extract every Node(..., package='x', executable='y')
# call. For each pair where the package is one of our 5 managed packages,
# verify that `y` matches either an add_executable() target or an
# install(PROGRAMS ...) file basename in that package's CMakeLists.txt.
# External packages (tf2_ros, topic_tools, rviz2, etc.) are SKIPPED.

if ! command -v awk >/dev/null 2>&1; then
    skip "M  launch Node executables resolve" "awk not available"
else
    # Map of managed package names to their CMakeLists.txt absolute path.
    M_MANAGED_PKGS="sloam sloam_msgs multi_robot_utils_launch object_modeller scan2shape_launch"
    # File lookup:
    m_cml_sloam="$REPO_ROOT/backend/sloam/CMakeLists.txt"
    m_cml_sloam_msgs="$REPO_ROOT/backend/sloam_msgs/CMakeLists.txt"
    m_cml_multi_robot_utils_launch="$REPO_ROOT/backend/multi_robot_utils_launch/CMakeLists.txt"
    m_cml_object_modeller="$REPO_ROOT/frontend/object_modeller/CMakeLists.txt"
    m_cml_scan2shape_launch="$REPO_ROOT/frontend/scan2shape/scan2shape_launch/CMakeLists.txt"

    # Build a list of "package:executable" valid entries from each managed
    # package's CMakeLists.txt.
    m_valid_entries_file=$(mktemp 2>/dev/null || printf '/tmp/m_valid_%d' $$)

    m_collect_pkg_executables() {
        local pkg="$1"
        local cml="$2"
        [ -f "$cml" ] || return 0
        awk -v pkg="$pkg" '
            BEGIN { in_install = 0 }
            {
                line = $0
                sub(/#.*/, "", line)
                # add_executable(name ...) — name is first token after the paren.
                if (match(line, /add_executable[[:space:]]*\([[:space:]]*[A-Za-z_][A-Za-z0-9_]*/)) {
                    chunk = substr(line, RSTART, RLENGTH)
                    if (match(chunk, /[A-Za-z_][A-Za-z0-9_]*[[:space:]]*$/) || match(chunk, /[A-Za-z_][A-Za-z0-9_]*$/)) {
                        name = substr(chunk, RSTART, RLENGTH)
                        gsub(/[[:space:]]+/, "", name)
                        # strip add_executable( if caught
                        sub(/^add_executable\(/, "", name)
                        printf "%s:%s\n", pkg, name
                    }
                }
            }
            # install(PROGRAMS ... DESTINATION ...) blocks — may span multiple lines.
            /install[[:space:]]*\([[:space:]]*PROGRAMS/ { in_install = 1; sub(/.*PROGRAMS/, "", line); }
            in_install >= 1 {
                work = line
                # Stop at DESTINATION.
                n = split(work, toks, /[[:space:]]+/)
                for (i = 1; i <= n; i++) {
                    t = toks[i]
                    if (t == "") continue
                    if (t == "(" || t == ")") continue
                    if (t == "PROGRAMS") continue
                    if (t == "DESTINATION" || t ~ /^DESTINATION/) { in_install = 0; break }
                    # strip surrounding parens
                    gsub(/^\(/, "", t)
                    gsub(/\)$/, "", t)
                    if (t == "") continue
                    # basename
                    sub(/.*\//, "", t)
                    if (t == "") continue
                    printf "%s:%s\n", pkg, t
                    # also strip .py extension
                    stripped = t
                    sub(/\.py$/, "", stripped)
                    if (stripped != t) printf "%s:%s\n", pkg, stripped
                }
                if (line ~ /\)/) in_install = 0
            }
        ' "$cml"
    }

    : > "$m_valid_entries_file"
    for pkg in $M_MANAGED_PKGS; do
        var="m_cml_${pkg}"
        m_collect_pkg_executables "$pkg" "${!var}" >> "$m_valid_entries_file"
    done

    m_bad=0
    m_bad_examples=()
    m_skipped=0
    m_checked=0

    # Parse each .launch.py, pulling (pkg, exe) pairs out of Node(...) calls.
    # The parser is tolerant of multi-line Node(...) blocks; it reads the
    # whole file into awk and searches for Node( ... ) expressions.
    m_extract() {
        local lp="$1"
        awk '
            BEGIN { RS = "Node[[:space:]]*\\("; first = 1 }
            {
                if (first) { first = 0; next }
                blk = $0
                # Find matching closing paren by walking the block until depth reaches 0.
                depth = 1
                end = 0
                for (i = 1; i <= length(blk); i++) {
                    c = substr(blk, i, 1)
                    if (c == "(") depth++
                    else if (c == ")") { depth--; if (depth == 0) { end = i; break } }
                }
                if (end == 0) next
                call = substr(blk, 1, end - 1)
                # Drop everything from # to end-of-line first (in case a kwarg
                # has a comment).
                # Not always safe, but good enough for our files.
                gsub(/#[^\n]*/, "", call)
                pkg = ""
                exe = ""
                if (match(call, /package[[:space:]]*=[[:space:]]*[\x27"][^\x27"]*[\x27"]/)) {
                    chunk = substr(call, RSTART, RLENGTH)
                    if (match(chunk, /[\x27"][^\x27"]*[\x27"]/)) {
                        pkg = substr(chunk, RSTART + 1, RLENGTH - 2)
                    }
                }
                if (match(call, /executable[[:space:]]*=[[:space:]]*[\x27"][^\x27"]*[\x27"]/)) {
                    chunk = substr(call, RSTART, RLENGTH)
                    if (match(chunk, /[\x27"][^\x27"]*[\x27"]/)) {
                        exe = substr(chunk, RSTART + 1, RLENGTH - 2)
                    }
                }
                if (pkg != "" && exe != "") {
                    printf "%s\t%s\n", pkg, exe
                }
            }
        ' "$lp"
    }

    while IFS= read -r lp; do
        [ -z "$lp" ] && continue
        rel="${lp#$REPO_ROOT/}"
        while IFS=$'\t' read -r pkg exe; do
            [ -z "$pkg" ] && continue
            # Only check managed packages.
            case " $M_MANAGED_PKGS " in
                *" $pkg "*)
                    m_checked=$((m_checked + 1))
                    if grep -Fxq "$pkg:$exe" "$m_valid_entries_file" 2>/dev/null; then
                        : # ok
                    else
                        # Also check without .py suffix.
                        noext="${exe%.py}"
                        if [ "$noext" != "$exe" ] && grep -Fxq "$pkg:$noext" "$m_valid_entries_file" 2>/dev/null; then
                            : # ok
                        else
                            m_bad=$((m_bad + 1))
                            m_bad_examples+=("$rel: Node(package='$pkg', executable='$exe') — no matching CMake target")
                        fi
                    fi
                    ;;
                *)
                    m_skipped=$((m_skipped + 1))
                    ;;
            esac
        done < <(m_extract "$lp")
    done < <(find "$REPO_ROOT/backend" "$REPO_ROOT/frontend" -type f -name '*.launch.py' 2>/dev/null | grep -v '/tests/')

    rm -f "$m_valid_entries_file"

    if [ "$m_bad" -eq 0 ]; then
        pass "M  every Node(package, executable) resolves in managed packages"
    else
        fail "M  every Node(package, executable) resolves in managed packages" \
            "$m_bad missing; checked $m_checked managed, skipped $m_skipped external"
        if [ "$VERBOSE" = "1" ]; then
            for e in "${m_bad_examples[@]}"; do printf "         %s\n" "$e"; done
        fi
    fi
fi

# ============================================================================
# N. Duplicate raw declare_parameter across files
# ============================================================================
section "N. Duplicate raw declare_parameter across files"
#
# Flag RAW declare_parameter("key", ...) keys that appear in 2+ C++ source
# files in the same sloam / object_modeller scope. A raw duplicate is a real
# runtime bug: ROS2 will throw rclcpp::exceptions::ParameterAlreadyDeclared
# on the second call.
#
# We deliberately DO NOT flag the `<prefix>_declare_or_get<T>(node, "key", ...)`
# wrapper family used throughout sloam. Those wrappers guard with
# `node->has_parameter(name)` before declaring, so multiple wrapper call sites
# on the same key are safe — the first caller declares, subsequent callers
# fall through to get_parameter. This is the idiomatic "declare-or-get" pattern
# and is not a bug. The check only runs on the raw, unguarded form.

if ! command -v awk >/dev/null 2>&1; then
    skip "N  duplicate declare_parameter keys across files" "awk not available"
else
    n_tmp=$(mktemp 2>/dev/null || printf '/tmp/n_decls_%d' $$)
    : > "$n_tmp"

    find "$REPO_ROOT/backend/sloam" "$REPO_ROOT/frontend/object_modeller" \
         -type f \( -name '*.cpp' -o -name '*.h' -o -name '*.hpp' -o -name '*.cc' \) 2>/dev/null \
      | grep -v '/clipper_semantic_object/' \
      | while IFS= read -r f; do
            awk -v file="$f" '
                { sub(/\/\/.*$/, "", $0) }
                # Skip safe-wrapper forms (*_declare_or_get<T>(...)). Those guard
                # with has_parameter() and are safe to call from multiple sites.
                /[a-zA-Z_]+declare_or_get[[:space:]]*(<[^>]*>)?[[:space:]]*\(/ { next }
                # Raw declare_parameter calls — these are the real hazard.
                /declare_parameter[[:space:]]*(<[^>]*>)?[[:space:]]*\(/ {
                    if (match($0, /"[^"]+"/)) {
                        key = substr($0, RSTART + 1, RLENGTH - 2)
                        if (key ~ /^[a-zA-Z_]/) {
                            print key "\t" file
                        }
                    }
                }
            ' "$f"
        done >> "$n_tmp"

    n_bad=""
    if [ -s "$n_tmp" ]; then
        n_bad=$(awk -F'\t' '
            {
                pair = $1 SUBSEP $2
                if (!(pair in seen)) {
                    seen[pair] = 1
                    file_count[$1]++
                    file_list[$1] = (file_list[$1] ? file_list[$1] " | " : "") $2
                }
            }
            END {
                for (k in file_count) {
                    if (file_count[k] >= 2) {
                        print k ": " file_list[k]
                    }
                }
            }
        ' "$n_tmp")
    fi
    rm -f "$n_tmp"

    if [ -z "$n_bad" ]; then
        pass "N  no declare_parameter key declared in multiple files"
    else
        n_count=$(printf "%s\n" "$n_bad" | wc -l | tr -d ' ')
        fail "N  no declare_parameter key declared in multiple files" "$n_count key(s) in 2+ files"
        if [ "$VERBOSE" = "1" ]; then
            printf "%s\n" "$n_bad" | head -10 | sed 's/^/         /'
        fi
    fi
fi


# ----- managed-package list (shared across sections L/M/O/P) -----
# Sections L and M already set this as a side-effect, but sections O/P
# must not depend on those branches having run. Redefine defensively so
# `set -u` is happy even when awk is unavailable.
M_MANAGED_PKGS="${M_MANAGED_PKGS:-sloam sloam_msgs multi_robot_utils_launch object_modeller scan2shape_launch}"

# ============================================================================
# O. Cross-package #include <pkg/...> vs package.xml <depend>pkg</depend>
# ============================================================================
section "O. #include <pkg/...> cross-referenced against package.xml <depend>"
#
# For every C/C++ source file in an in-tree managed package (sloam,
# sloam_msgs, multi_robot_utils_launch, object_modeller, scan2shape_launch),
# extract every `#include <pkg/...>` / `#include "pkg/..."` top-level package
# name, find the owning package.xml by walking up the directory tree, and
# verify every non-exempt `pkg` appears in the package.xml's <depend>-like
# tags. Self-includes, system headers, C++ stdlib, and certain known
# third-party libs that are pulled via find_package() are skipped.
#
# Carve-outs:
#   * backend/sloam/clipper_semantic_object/ — vendored third-party library
#     added via add_subdirectory() rather than as a ROS package. Its headers
#     live inside backend/sloam/ but it is not in scope for section O.

if ! command -v awk >/dev/null 2>&1; then
    skip "O  cross-package #include vs package.xml <depend>" "awk not available"
else
    # Exempt set: multi-token system headers resolved via find_package() +
    # target_link_libraries rather than <depend>.
    o_exempt_list=$'\nEigen\neigen3\nunsupported\nboost\ngtsam\npcl\nopencv2\nOpenCV\nfmt\nyaml-cpp\nglog\ntbb\ngtest\nbenchmark\nnetinet\narpa\nlinux\nfcntl\nunistd\npthread\nsys\nrcutils\nsophus\n'

    # Collect all managed package.xml paths.
    o_xmls_file=$(mktemp 2>/dev/null || printf '/tmp/o_xmls_%d' $$)
    find "$REPO_ROOT/backend" "$REPO_ROOT/frontend" -type f -name 'package.xml' 2>/dev/null \
        | LC_ALL=C sort > "$o_xmls_file"

    # Map: pkg_name -> package.xml path; pkg_name -> newline-joined depends.
    # We use flat files (not bash assoc arrays) because this is POSIX-ish.
    o_xml_by_name=$(mktemp 2>/dev/null || printf '/tmp/o_xmlbyname_%d' $$)
    o_deps_dir=$(mktemp -d 2>/dev/null || { d="/tmp/o_depsdir_$$"; mkdir -p "$d"; printf '%s' "$d"; })
    while IFS= read -r xml; do
        [ -z "$xml" ] && continue
        name="$(package_xml_name "$xml")"
        [ -z "$name" ] && continue
        printf "%s\t%s\n" "$name" "$xml" >> "$o_xml_by_name"
        package_xml_depends "$xml" > "$o_deps_dir/$name"
    done < "$o_xmls_file"

    # Build include-path top-dir -> owning pkg name map. For each managed
    # package we scan its include/* subdirs and record the top-level dir
    # name -> owning package name. This resolves cross-package includes
    # whose top-level include dir name differs from the package name.
    o_incl2pkg=$(mktemp 2>/dev/null || printf '/tmp/o_incl2pkg_%d' $$)
    : > "$o_incl2pkg"
    while IFS= read -r xml; do
        [ -z "$xml" ] && continue
        pname="$(package_xml_name "$xml")"
        [ -z "$pname" ] && continue
        pdir="$(dirname "$xml")"
        [ -d "$pdir/include" ] || continue
        find "$pdir/include" -mindepth 1 -maxdepth 1 -type d 2>/dev/null | while IFS= read -r sub; do
            [ -z "$sub" ] && continue
            b="$(basename "$sub")"
            printf "%s\t%s\n" "$b" "$pname" >> "$o_incl2pkg"
        done
    done < "$o_xmls_file"

    # Walk every C/C++ source file under the managed packages, skipping
    # the vendored clipper_semantic_object tree entirely.
    o_files=$(mktemp 2>/dev/null || printf '/tmp/o_files_%d' $$)
    find "$REPO_ROOT/backend" "$REPO_ROOT/frontend" \
        -type f \( -name '*.cpp' -o -name '*.cc' -o -name '*.h' -o -name '*.hpp' \) 2>/dev/null \
        | grep -v '/clipper_semantic_object/' \
        | LC_ALL=C sort > "$o_files"

    o_fail=$(mktemp 2>/dev/null || printf '/tmp/o_fail_%d' $$)
    o_hits=$(mktemp 2>/dev/null || printf '/tmp/o_hits_%d' $$)
    : > "$o_fail"
    : > "$o_hits"

    o_file_count=0
    while IFS= read -r f; do
        [ -z "$f" ] && continue
        owning_xml="$(find_owning_package_xml "$f")"
        [ -z "$owning_xml" ] && continue
        owning_name="$(package_xml_name "$owning_xml")"
        [ -z "$owning_name" ] && continue

        # Only check files belonging to one of the 5 managed packages.
        case " $M_MANAGED_PKGS " in
            *" $owning_name "*) : ;;
            *) continue ;;
        esac

        o_file_count=$((o_file_count + 1))

        # Extract top-level pkg names from #include lines (both <...> and "...").
        # Require a slash after pkg to skip C++ stdlib (`<string>`, `<vector>`, ...).
        # strip_cpp_line_comments needs file:line:text format, so we inline the
        # simpler approach: pipe file through a grep that ignores // commented
        # includes on pure-comment lines. Raw awk form is used here to keep
        # extraction self-contained.
        pkgs=$(awk '
            { line = $0 }
            # Strip inline // comments (simple, not string-literal aware).
            { sub(/\/\/.*$/, "", line) }
            # Match #include <pkg/...> or #include "pkg/..."
            match(line, /#include[[:space:]]*[<"][A-Za-z_][A-Za-z0-9_]*\//) {
                chunk = substr(line, RSTART, RLENGTH)
                # Pull the pkg token.
                if (match(chunk, /[<"][A-Za-z_][A-Za-z0-9_]*\//)) {
                    t = substr(chunk, RSTART + 1, RLENGTH - 2)
                    print t
                }
            }
        ' "$f" | LC_ALL=C sort -u)
        [ -z "$pkgs" ] && continue

        deps_file="$o_deps_dir/$owning_name"
        [ -f "$deps_file" ] || continue

        while IFS= read -r pkg; do
            [ -z "$pkg" ] && continue
            printf "%s\n" "$pkg" >> "$o_hits"
            # Skip exempt pkgs.
            case "$o_exempt_list" in
                *$'\n'"$pkg"$'\n'*) continue ;;
            esac
            # Skip self-include by raw name.
            if [ "$pkg" = "$owning_name" ]; then
                continue
            fi
            # Resolve include-path top-dir to package name if known.
            resolved_pkg="$pkg"
            rp="$(awk -F'\t' -v k="$pkg" '$1 == k { print $2; exit }' "$o_incl2pkg")"
            if [ -n "$rp" ]; then
                resolved_pkg="$rp"
            fi
            # Skip self-include via resolved name too.
            if [ "$resolved_pkg" = "$owning_name" ]; then
                continue
            fi
            # If declared in deps (raw or resolved), pass.
            if grep -qxF "$pkg" "$deps_file" 2>/dev/null; then
                continue
            fi
            if [ "$resolved_pkg" != "$pkg" ] && grep -qxF "$resolved_pkg" "$deps_file" 2>/dev/null; then
                continue
            fi
            # Record the failure.
            rel="${f#$REPO_ROOT/}"
            if [ "$resolved_pkg" != "$pkg" ]; then
                printf "%s: #include <%s/...> (-> pkg '%s') but '%s' not in %s\n" \
                    "$rel" "$pkg" "$resolved_pkg" "$resolved_pkg" "$owning_name/package.xml" >> "$o_fail"
            else
                printf "%s: #include <%s/...> but '%s' not in %s\n" \
                    "$rel" "$pkg" "$pkg" "$owning_name/package.xml" >> "$o_fail"
            fi
        done <<EOF
$pkgs
EOF
    done < "$o_files"

    o_unique_hits=$(LC_ALL=C sort -u "$o_hits" 2>/dev/null | awk 'NF' | wc -l | tr -d ' ')

    if [ -s "$o_fail" ]; then
        LC_ALL=C sort -u "$o_fail" -o "$o_fail"
        o_count=$(wc -l < "$o_fail" | tr -d ' ')
        fail "O  cross-package #include vs package.xml <depend>" \
            "$o_count missing <depend> entr(y/ies) across $o_unique_hits distinct top-level names ($o_file_count files)"
        if [ "$VERBOSE" = "1" ]; then
            head -20 "$o_fail" | sed 's/^/         /'
        fi
    else
        pass "O  every cross-package #include has a matching <depend> ($o_unique_hits top-level names, $o_file_count files)"
    fi
    rm -f "$o_fail" "$o_hits" "$o_files" "$o_xml_by_name" "$o_incl2pkg" "$o_xmls_file"
    rm -rf "$o_deps_dir"
fi

# ============================================================================
# P. Launch dict-parameter keys declared in target package source
# ============================================================================
section "P. Launch dict-parameter keys declared in target package source"
#
# For every Node(package=P, executable=E, parameters=[{...}]) call in a
# launch file, verify every literal dict key is declared in P's source tree
# as declare_parameter("key"...) (template or plain form), or via the
# get_param_or / declare_parameter_if_not_declared wrappers used in the repo.
# Catches the classic ROS2 bug where a launch file passes a parameter that
# the target node silently ignores because it never calls declare_parameter.

if ! command -v awk >/dev/null 2>&1; then
    skip "P  launch dict-parameter keys declared in target package source" "awk not available"
else
    # Build pkg_name -> declared keys file map for each managed package.
    p_decl_dir=$(mktemp -d 2>/dev/null || { d="/tmp/p_decldir_$$"; mkdir -p "$d"; printf '%s' "$d"; })

    # Map managed pkg -> its directory.
    p_pkg_dir() {
        case "$1" in
            sloam) printf '%s\n' "$REPO_ROOT/backend/sloam" ;;
            sloam_msgs) printf '%s\n' "$REPO_ROOT/backend/sloam_msgs" ;;
            multi_robot_utils_launch) printf '%s\n' "$REPO_ROOT/backend/multi_robot_utils_launch" ;;
            object_modeller) printf '%s\n' "$REPO_ROOT/frontend/object_modeller" ;;
            scan2shape_launch) printf '%s\n' "$REPO_ROOT/frontend/scan2shape/scan2shape_launch" ;;
            *) printf '\n' ;;
        esac
    }

    for pkg in $M_MANAGED_PKGS; do
        pdir=$(p_pkg_dir "$pkg")
        [ -z "$pdir" ] && continue
        [ -d "$pdir" ] || continue
        tmp_keys="$p_decl_dir/$pkg"
        : > "$tmp_keys"
        # Walk source and script files. Flatten with tr for multi-line
        # declare_parameter<T>(\n "key", ...) calls.
        find "$pdir" -type f \( -name '*.cpp' -o -name '*.cc' -o -name '*.h' -o -name '*.hpp' -o -name '*.py' \) 2>/dev/null \
            | grep -v '/clipper_semantic_object/' \
            | while IFS= read -r sf; do
                [ -z "$sf" ] && continue
                if [ "${sf##*.}" = "py" ]; then
                    # Strip # comments line-by-line (crude; fine for our use).
                    awk '{ sub(/#.*$/, ""); print }' "$sf" \
                        | tr '\n' ' ' \
                        | grep -oE "declare_parameter[[:space:]]*\([[:space:]]*['\"][^'\"]+['\"]" 2>/dev/null \
                        | sed -E "s/.*['\"]([^'\"]+)['\"].*/\1/" >> "$tmp_keys" || true
                else
                    # Strip // line comments and flatten multi-line calls.
                    awk '{ sub(/\/\/.*$/, ""); print }' "$sf" \
                        | tr '\n' ' ' > "$p_decl_dir/__flat"
                    grep -oE 'declare_parameter[[:space:]]*(<[^>]*>)?[[:space:]]*\([[:space:]]*"[^"]+"' "$p_decl_dir/__flat" 2>/dev/null \
                        | sed -E 's/.*"([^"]+)".*/\1/' >> "$tmp_keys" || true
                    # Also capture _declare_or_get<T>(node, "key", default) wrapper form.
                    grep -oE '_declare_or_get[[:space:]]*<[^>]*>[[:space:]]*\([[:space:]]*[A-Za-z_][A-Za-z0-9_>.]*[[:space:]]*,[[:space:]]*"[^"]+"' "$p_decl_dir/__flat" 2>/dev/null \
                        | sed -E 's/.*,[[:space:]]*"([^"]+)".*/\1/' >> "$tmp_keys" || true
                    # get_param_or(node, "key", var, default) and
                    # declare_parameter_if_not_declared(node, "key", ...).
                    grep -oE '\bget_param_or[[:space:]]*\([[:space:]]*[A-Za-z_][A-Za-z0-9_>.]*[[:space:]]*,[[:space:]]*"[^"]+"' "$p_decl_dir/__flat" 2>/dev/null \
                        | sed -E 's/.*,[[:space:]]*"([^"]+)".*/\1/' >> "$tmp_keys" || true
                    grep -oE '\bdeclare_parameter_if_not_declared[[:space:]]*\([[:space:]]*[A-Za-z_][A-Za-z0-9_>.]*[[:space:]]*,[[:space:]]*"[^"]+"' "$p_decl_dir/__flat" 2>/dev/null \
                        | sed -E 's/.*,[[:space:]]*"([^"]+)".*/\1/' >> "$tmp_keys" || true
                fi
            done
        rm -f "$p_decl_dir/__flat"
        LC_ALL=C sort -u "$tmp_keys" -o "$tmp_keys" 2>/dev/null || true
    done

    p_fail=$(mktemp 2>/dev/null || printf '/tmp/p_fail_%d' $$)
    : > "$p_fail"
    p_nodes=0

    # Walk every .launch.py and extract Node(...) blocks.
    find "$REPO_ROOT/backend" "$REPO_ROOT/frontend" -type f -name '*.launch.py' 2>/dev/null \
        | LC_ALL=C sort \
        | while IFS= read -r lf; do
            [ -z "$lf" ] && continue
            rel="${lf#$REPO_ROOT/}"
            # Slurp and flatten newlines.
            joined=$(tr '\n' ' ' < "$lf")
            # Extract Node( ... ) blocks. Node must not be preceded by a word char
            # (avoids ComposableNode / LifecycleNode).
            blocks=$(printf "%s" "$joined" \
                | grep -oE "(^|[^A-Za-z0-9_])Node[[:space:]]*\([^)]{0,2000}\)" 2>/dev/null || true)
            [ -z "$blocks" ] && continue
            printf '%s\n' "$blocks" | while IFS= read -r blk; do
                [ -z "$blk" ] && continue
                pkg=$(printf "%s" "$blk" \
                    | grep -oE "package[[:space:]]*=[[:space:]]*['\"][^'\"]+['\"]" \
                    | head -1 | sed -E "s/.*['\"]([^'\"]+)['\"]/\1/")
                exe=$(printf "%s" "$blk" \
                    | grep -oE "executable[[:space:]]*=[[:space:]]*['\"][^'\"]+['\"]" \
                    | head -1 | sed -E "s/.*['\"]([^'\"]+)['\"]/\1/")
                [ -z "$pkg" ] && continue
                case " $M_MANAGED_PKGS " in
                    *" $pkg "*) : ;;
                    *) continue ;;
                esac
                # Increment via file (subshell safe).
                printf 'x\n' >> "$p_fail.nodes"
                # Extract parameters=[...] list up to first closing bracket.
                plist=$(printf "%s" "$blk" \
                    | grep -oE "parameters[[:space:]]*=[[:space:]]*\[[^]]*\]" \
                    | head -1 || true)
                [ -z "$plist" ] && continue
                keys=$(printf "%s" "$plist" \
                    | grep -oE "['\"][A-Za-z_][A-Za-z0-9_.]*['\"][[:space:]]*:" \
                    | sed -E "s/['\"]([^'\"]+)['\"].*/\1/" \
                    | LC_ALL=C sort -u)
                [ -z "$keys" ] && continue
                declared_file="$p_decl_dir/$pkg"
                [ -f "$declared_file" ] || declared_file=/dev/null
                printf '%s\n' "$keys" | while IFS= read -r key; do
                    [ -z "$key" ] && continue
                    # Skip resource-path-like keys.
                    case "$key" in
                        /*|*.yaml|*.yml|*.launch.py) continue ;;
                    esac
                    if ! grep -qxF "$key" "$declared_file" 2>/dev/null; then
                        printf "%s: Node(package='%s', executable='%s') parameters=[{'%s': ...}] but no declare_parameter(\"%s\", ...) found in %s source tree\n" \
                            "$rel" "$pkg" "$exe" "$key" "$key" "$pkg" >> "$p_fail"
                    fi
                done
            done
        done

    if [ -f "$p_fail.nodes" ]; then
        p_nodes=$(wc -l < "$p_fail.nodes" | tr -d ' ')
        rm -f "$p_fail.nodes"
    fi

    if [ -s "$p_fail" ]; then
        LC_ALL=C sort -u "$p_fail" -o "$p_fail"
        p_count=$(wc -l < "$p_fail" | tr -d ' ')
        fail "P  launch dict-parameter keys declared in target package" \
            "$p_count undeclared key(s) across $p_nodes Node(...) call(s)"
        if [ "$VERBOSE" = "1" ]; then
            head -20 "$p_fail" | sed 's/^/         /'
        fi
    else
        pass "P  all launch dict-parameter keys are declared in their target package ($p_nodes Node calls analyzed)"
    fi
    rm -f "$p_fail"
    rm -rf "$p_decl_dir"
fi

# ============================================================================
# Q. Shell script syntax sweep (bash -n)
# ============================================================================
section "Q. Shell script syntax (bash -n)"
#
# Run `bash -n` on every .sh/.bash file under backend/, frontend/, tools/,
# tests/. Any syntactically-invalid script fails the check. Skipped if
# bash -n is unusable in the runtime sandbox.

q_scripts=$(find "$REPO_ROOT/backend" "$REPO_ROOT/frontend" "$REPO_ROOT/tools" "$REPO_ROOT/tests" \
    -type f \( -name '*.sh' -o -name '*.bash' \) 2>/dev/null \
    | grep -v '/clipper_semantic_object/' \
    | LC_ALL=C sort || true)

if [ -z "$q_scripts" ]; then
    skip "Q  bash -n shell script syntax sweep" "no shell scripts found"
else
    # Capability probe: bash -n on a trivially-correct script.
    q_probe=$(mktemp 2>/dev/null || printf '/tmp/q_probe_%d' $$)
    printf '#!/usr/bin/env bash\nexit 0\n' > "$q_probe"
    if ! bash -n "$q_probe" 2>/dev/null; then
        rm -f "$q_probe"
        skip "Q  bash -n shell script syntax sweep" "bash -n not usable in this environment"
    else
        rm -f "$q_probe"
        q_fail=$(mktemp 2>/dev/null || printf '/tmp/q_fail_%d' $$)
        : > "$q_fail"
        q_count=0
        while IFS= read -r f; do
            [ -z "$f" ] && continue
            q_count=$((q_count + 1))
            if ! bash -n "$f" 2>>"$q_fail"; then
                rel="${f#$REPO_ROOT/}"
                printf "  [syntax error in] %s\n" "$rel" >> "$q_fail"
            fi
        done <<EOF
$q_scripts
EOF
        if [ -s "$q_fail" ]; then
            q_err_count=$(grep -c '^' "$q_fail" 2>/dev/null || echo 0)
            fail "Q  bash -n shell script syntax sweep" \
                "bash -n reported errors in $q_count shell script(s)"
            if [ "$VERBOSE" = "1" ]; then
                head -20 "$q_fail" | sed 's/^/         /'
            fi
        else
            pass "Q  all $q_count shell script(s) parse cleanly with bash -n"
        fi
        rm -f "$q_fail"
    fi
fi


# ============================================================================
# Summary
# ============================================================================
print_summary

if [ "$FAILED" -gt 0 ]; then
    exit 1
fi
exit 0
