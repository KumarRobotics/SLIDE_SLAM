#!/usr/bin/env bash
#
# convert_ros1_bags.sh
#
# Convert ROS1 .bag files to the ROS2 bag format (sqlite3 by default).
# The existing SlideSLAM demo and benchmark bags were recorded under ROS1
# Noetic; on the ros2_dev branch (ROS2 Jazzy) you must convert them before
# they can be replayed with `ros2 bag play`.
#
# Prerequisites:
#     pip install rosbags
#
# (The `rosbags` PyPI package ships a standalone `rosbags-convert` binary
# that does not require a ROS1 installation to read a .bag file.)
#
# Usage:
#     tools/convert_ros1_bags.sh <input.bag>
#     tools/convert_ros1_bags.sh <input_dir>              # every .bag in the dir
#     tools/convert_ros1_bags.sh <input.bag> <out_dir>    # explicit output dir
#     tools/convert_ros1_bags.sh <input_dir>  <out_dir>   # explicit output dir
#
# By default the converted ROS2 bag is written next to the original as a
# directory with the same base name (minus the .bag extension). Existing
# output directories are skipped to avoid clobbering work in progress.
#
# Example:
#     tools/convert_ros1_bags.sh ~/bags/forest_multi_robot
#
# produces
#
#     ~/bags/forest_multi_robot/robot1/
#     ~/bags/forest_multi_robot/robot2/
#     ...
#
# alongside the source .bag files, each containing metadata.yaml + a .db3.
#
# Exit codes:
#     0  success
#     1  rosbags-convert not installed
#     2  bad arguments
#     3  no .bag files found in input directory
#     4  input path is neither a file nor a directory

set -euo pipefail

if ! command -v rosbags-convert >/dev/null 2>&1; then
  cat >&2 <<'EOF'
error: rosbags-convert not found.

Install it with:

    pip install rosbags

See https://gitlab.com/ternaris/rosbags for details. The `rosbags` package
ships a standalone converter that does NOT require a ROS1 installation to
read .bag files, so it works cleanly on Ubuntu 24.04 / ROS2 Jazzy.
EOF
  exit 1
fi

if [[ $# -lt 1 || $# -gt 2 ]]; then
  echo "usage: $0 <input.bag|input_dir> [output_dir]" >&2
  exit 2
fi

INPUT="$1"
OUTPUT_DIR="${2:-}"

convert_one() {
  local bag="$1"
  local base
  base="$(basename "${bag%.bag}")"
  local out
  if [[ -n "$OUTPUT_DIR" ]]; then
    mkdir -p "$OUTPUT_DIR"
    out="$OUTPUT_DIR/$base"
  else
    out="$(dirname "$bag")/$base"
  fi
  if [[ -e "$out" ]]; then
    echo "skip: $out already exists"
    return 0
  fi
  echo "converting: $bag -> $out"
  rosbags-convert --src "$bag" --dst "$out"
}

if [[ -d "$INPUT" ]]; then
  shopt -s nullglob
  bags=("$INPUT"/*.bag)
  if [[ ${#bags[@]} -eq 0 ]]; then
    echo "no .bag files found in $INPUT" >&2
    exit 3
  fi
  for bag in "${bags[@]}"; do
    convert_one "$bag"
  done
elif [[ -f "$INPUT" ]]; then
  convert_one "$INPUT"
else
  echo "error: $INPUT is not a file or directory" >&2
  exit 4
fi

echo "done."
