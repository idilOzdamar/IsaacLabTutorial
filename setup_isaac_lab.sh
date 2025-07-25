#!/usr/bin/env bash
set -e  # Hata olursa script dursun

echo "Installing isaac_lab_tutorial in editable mode..."
${ISAACLAB_PATH}/_isaac_sim/python.sh -m pip install -e source/isaac_lab_tutorial