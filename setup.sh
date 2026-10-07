#!/bin/bash
# One-shot preparation: venv -> official model -> MuJoCo conversion -> model checks.
# models/urdf/fr3_hand.{urdf,srdf} are committed (xacro output), so ROS is NOT required.
# To regenerate them from xacro (needs ROS 2 Jazzy xacro): bash scripts/20_expand_xacro.sh
set -e
cd "$(dirname "$0")"
bash scripts/00_setup_venv.sh
bash scripts/10_fetch_model.sh
./run_py.sh scripts/30_convert.py
./run_py.sh scripts/40_inspect_model.py
echo "setup done: ./run.sh"
