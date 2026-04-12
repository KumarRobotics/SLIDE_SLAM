#!/usr/bin/env bash
#
# launch_smoke_test.sh
#
# Run `ros2 launch --print-description` on every .launch.py in the repo
# to verify each launch file parses and all referenced packages /
# executables / config paths resolve. This is a SMOKE TEST: it does NOT
# actually start any nodes; it only validates the launch graph.
#
# Requirements:
#     - ROS2 Jazzy sourced (`source /opt/ros/jazzy/setup.bash`)
#     - The workspace must be built (`colcon build --symlink-install`)
#     - Source the workspace overlay (`source install/setup.bash`)
#
# Without these, the test will skip (exit code 77 == autotools skip).
#
# Usage:
#     tests/integration/launch_smoke_test.sh [repo_root]
#
# Optional env:
#     LAUNCH_TIMEOUT=<seconds>   per-file timeout (default: 20)
#     VERBOSE=1                  show full output on failure

set -uo pipefail

REPO_ROOT="${1:-$(cd "$(dirname "$0")/../.." && pwd)}"
TIMEOUT="${LAUNCH_TIMEOUT:-20}"
VERBOSE="${VERBOSE:-0}"

# ANSI colors (only if stdout is a tty)
if [ -t 1 ]; then
    C_RED=$'\033[31m'
    C_GREEN=$'\033[32m'
    C_YELLOW=$'\033[33m'
    C_BOLD=$'\033[1m'
    C_RESET=$'\033[0m'
else
    C_RED=""; C_GREEN=""; C_YELLOW=""; C_BOLD=""; C_RESET=""
fi

if ! command -v ros2 >/dev/null 2>&1; then
    echo "${C_YELLOW}SKIP${C_RESET}: ros2 not on PATH. Source /opt/ros/jazzy/setup.bash and"
    echo "      your workspace overlay before running this test."
    exit 77
fi

if ! command -v timeout >/dev/null 2>&1; then
    echo "${C_YELLOW}SKIP${C_RESET}: GNU coreutils 'timeout' not on PATH."
    exit 77
fi

echo "${C_BOLD}launch_smoke_test${C_RESET}: scanning ${REPO_ROOT} for *.launch.py"

# Prefer `find` over `rg --files` for portability; only backend/ and frontend/
# house the source launch files we care about.
LAUNCH_FILES=()
while IFS= read -r -d '' f; do
    LAUNCH_FILES+=("$f")
done < <(
    find "${REPO_ROOT}/backend" "${REPO_ROOT}/frontend" \
        -type f -name '*.launch.py' -print0 2>/dev/null | sort -z
)

total=${#LAUNCH_FILES[@]}
if [ "${total}" -eq 0 ]; then
    echo "${C_YELLOW}SKIP${C_RESET}: no .launch.py files found under ${REPO_ROOT}"
    exit 77
fi

pass=0
fail=0
FAILED=()
TMP_OUT="$(mktemp -t launch_smoke_XXXXXX.log)"
trap 'rm -f "${TMP_OUT}"' EXIT

for launch in "${LAUNCH_FILES[@]}"; do
    rel="${launch#${REPO_ROOT}/}"
    printf '  %-70s ' "${rel}"
    # Use the direct-file-path form of `ros2 launch --print-description`
    # so the test runs even without the workspace being installed.
    if timeout --signal=TERM "${TIMEOUT}" \
            ros2 launch --print-description "${launch}" \
            >"${TMP_OUT}" 2>&1; then
        echo "${C_GREEN}OK${C_RESET}"
        pass=$((pass + 1))
        if [ "${VERBOSE}" = "1" ]; then
            sed 's/^/      /' "${TMP_OUT}"
        fi
    else
        rc=$?
        if [ "${rc}" -eq 124 ]; then
            echo "${C_RED}TIMEOUT${C_RESET} (>${TIMEOUT}s)"
        else
            echo "${C_RED}FAIL${C_RESET} (rc=${rc})"
        fi
        fail=$((fail + 1))
        FAILED+=("${rel}")
        sed 's/^/      /' "${TMP_OUT}"
    fi
done

echo
echo "${C_BOLD}Summary${C_RESET}: ${pass}/${total} passed, ${fail} failed"
if [ "${fail}" -gt 0 ]; then
    echo "${C_RED}Failed launch files:${C_RESET}"
    for f in "${FAILED[@]}"; do
        echo "  - ${f}"
    done
    exit 1
fi
exit 0
