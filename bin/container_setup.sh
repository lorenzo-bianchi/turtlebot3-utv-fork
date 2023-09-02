#!/usr/bin/env bash

# Setup script for persistent container on Nvidia Jetson Nano boards.
#
# Roberto Masocco <robmasocco@gmail.com>
# Intelligent Systems Lab <isl.torvergata@gmail.com>
#
# September 2, 2023

set -o errexit
set -o nounset
set -o pipefail
if [[ "${TRACE-0}" == "1" ]]; then set -o xtrace; fi

# Verify that the PWD is the project root directory
CURR_DIR=${PWD##*/}
INIT_DIR=$PWD
REQ_CURR_DIR="turtlebot3-utv"
if [[ $CURR_DIR != "$REQ_CURR_DIR" ]]; then
  echo >&2 "ERROR: Wrong path, this script must run inside $REQ_CURR_DIR"
  return 1
fi

cd "docker/container-jetson4c5/.devcontainer" || return 1

# Build the container
echo "Building the container..."
sleep 1
docker-compose build

# Start the container
echo "Starting the container..."
sleep 1
docker-compose up -d

cd "$INIT_DIR" || return 1
echo "Done! You can now access the running container with the 'dexecn' command as explained in the README."
