#!/usr/bin/env bash
set -e  # Stop if any error

echo "Installing isaac_lab_tutorial in editable mode..."
${ISAACLAB_PATH}/_isaac_sim/python.sh -m pip install -e source/isaac_lab_tutorial


# Get the computer's host IP which is running the IsaacLab  and set it to PUBLIC_IP (important IsaacLab enviroment variable to stream)
LOCAL_IP=$(hostname -I | awk '{print $1}')
export PUBLIC_IP=$LOCAL_IP

# # Current streaming selection. Info: LIVESTREAM=2 is for no GUI and with WebRTC (--headless)
# export HEADLESS=0
# export LIVESTREAM=2
export ENABLE_CAMERAS=1

export KIT_ARGS="--/app/livestream/publicEndpointAddress=$LOCAL_IP --/app/livestream/port=49200"

echo "KIT_ARGS set as: $KIT_ARGS"

echo "Script finished."
