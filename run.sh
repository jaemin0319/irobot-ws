#!/bin/bash
# Isolated launcher: strips ROS PYTHONPATH/LD_LIBRARY_PATH, uses the workspace venv only.
cd "$(dirname "$0")"
exec env -u PYTHONPATH -u LD_LIBRARY_PATH PYTHONNOUSERSITE=1 .venv/bin/python -m irobot_sim.run "$@"
