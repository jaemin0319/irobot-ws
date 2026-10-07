#!/bin/bash
# Create the workspace venv and install pinned dependencies (no pip cache, temp files in .tmp/).
set -e
cd "$(dirname "$0")/.."
mkdir -p .tmp
[ -x .venv/bin/python ] || python3 -m venv .venv
env -u PYTHONPATH -u LD_LIBRARY_PATH PYTHONNOUSERSITE=1 TMPDIR="$PWD/.tmp" \
  .venv/bin/pip install --no-cache-dir -r requirements.txt
rm -rf .tmp/pip-* 2>/dev/null || true
