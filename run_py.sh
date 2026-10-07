#!/bin/bash
# Isolated venv python (no ROS PYTHONPATH/LD_LIBRARY_PATH leakage)
cd "$(dirname "$0")"
exec env -u PYTHONPATH -u LD_LIBRARY_PATH PYTHONNOUSERSITE=1 TMPDIR="$PWD/.tmp" .venv/bin/python "$@"
