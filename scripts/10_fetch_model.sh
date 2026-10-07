#!/bin/bash
# Fetch the official franka_description 2.9.0 (only the FR3 + Franka Hand files) and verify the commit.
set -e
cd "$(dirname "$0")/.."
TAG=2.9.0
COMMIT=7aeeddc449edf8d62b594f9e36a81da53e7796f9
DST=third_party/franka_description
if [ ! -d "$DST/.git" ]; then
  git clone -q --filter=blob:none --no-checkout --depth 1 --branch "$TAG" \
    https://github.com/frankarobotics/franka_description.git "$DST"
  git -C "$DST" sparse-checkout init --cone
  git -C "$DST" sparse-checkout set robots/common robots/fr3 end_effectors/common end_effectors/franka_hand \
    meshes/robots/fr3 meshes/robot_ee/franka_hand_white
  git -C "$DST" checkout -q
fi
[ "$(git -C "$DST" rev-parse HEAD)" = "$COMMIT" ] || { echo "unexpected commit"; exit 1; }
# minimal ament index so system xacro can resolve $(find franka_description) (used by 20_expand_xacro.sh)
mkdir -p third_party/ament_prefix/share/ament_index/resource_index/packages
touch third_party/ament_prefix/share/ament_index/resource_index/packages/franka_description
ln -sfn ../../franka_description third_party/ament_prefix/share/franka_description
echo "franka_description $TAG @ $COMMIT ready"
