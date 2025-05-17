#!/bin/bash

# Get the directory where the script is located
SCRIPT_DIR="$(dirname "$(realpath "$0")")"

# Set the relative path to the parameter file
RELATIVE_PATH="ros2/launch/params.yaml"

# Construct the absolute path to the parameter file
PARAMETER_FILE="$SCRIPT_DIR/$RELATIVE_PATH"

# Default namespace
namespace="inertial_sense"

# Default debug setting
debug_logging=""

# Use getopt to parse command-line options
while getopts "dn:" opt; do
  case "$opt" in
    d)
      debug_logging="--log-level debug"
      ;;
    n)
      namespace="$OPTARG"
      ;;
    \?)
      echo "Usage: $0 [-d] [-n <namespace>]" >&2
      exit 1
      ;;
  esac
done

# Remove parsed options from positional parameters
shift $((OPTIND - 1))

# Run the ROS 2 node with the determined namespace, debug logging, and absolute parameter path
ros2 run inertial_sense_ros2 inertial_sense_ros2_node "$PARAMETER_FILE" --ros-args -r __node:=$namespace -r __ns:=/$namespace $debug_logging