#!/usr/bin/env bash

# OpenCR board firmware update script.
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
if [[ $CURR_DIR != "$REQ_CURR_DIR" ]] || [[ $CURR_DIR != "workspace" ]]; then
  echo >&2 "ERROR: Wrong path, this script must run inside $REQ_CURR_DIR"
  return 1
fi

# Print a little warning and wait for confirmation
echo "WARNING: This script will update the OpenCR board firmware to the latest available official binary."
echo "This script must be ran on the SBC; it can run inside the Docker container."
echo "Please make sure that the board is connected to the SBC via USB."
echo "You can select the board port with the OPENCR_PORT environment variable, and the board model with the OPENCR_MODEL environment variable."
echo "Current settings are:"
echo " - OPENCR_PORT: ${OPENCR_PORT}"
echo " - OPENCR_MODEL: ${OPENCR_MODEL}"
echo "Press ENTER to continue, or CTRL+C to abort."
read -r

# Download the latest OpenCR firmware binary
echo "Downloading the latest OpenCR firmware binary..."
sleep 1
wget https://github.com/ROBOTIS-GIT/OpenCR-Binaries/raw/master/turtlebot3/ROS2/latest/opencr_update.tar.bz2 \
  -O tools/opencr_update.tar.bz2 \
  -nc \
  --progress=bar \
  --show-progress
echo "Uncompressing the downloaded archive..."
sleep 1
cd tools || return 1
tar -xvf opencr_update.tar.bz2

# Run the update script
echo "Running the update script..."
sleep 1
cd opencr_update || return 1
./update.sh "$OPENCR_PORT" "$OPENCR_MODEL.opencr"

cd "$INIT_DIR" || return 1
echo "Done!"
