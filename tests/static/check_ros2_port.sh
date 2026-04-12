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
# Summary
# ============================================================================
print_summary

if [ "$FAILED" -gt 0 ]; then
    exit 1
fi
exit 0
