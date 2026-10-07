#!/bin/bash
# Expand official FR3 + Franka Hand xacro to URDF (uses system ROS 2 Jazzy xacro only)
set -e
cd "$(dirname "$0")/.."
[ -d third_party/ament_prefix ] || bash scripts/10_fetch_model.sh
source /opt/ros/jazzy/setup.bash
export AMENT_PREFIX_PATH=$PWD/third_party/ament_prefix:$AMENT_PREFIX_PATH
xacro third_party/franka_description/robots/fr3/fr3.urdf.xacro hand:=true ee_id:=franka_hand with_sc:=false > models/urdf/fr3_hand.urdf
xacro third_party/franka_description/robots/fr3/fr3.srdf.xacro hand:=true ee_id:=franka_hand > models/urdf/fr3_hand.srdf
