#!/bin/bash
set -e
cd "$(dirname "$0")"
echo "Project directory: $PWD"
python3 tools/analyze_dual_quality.py --mode full
python3 tools/render_dual_heatmaps.py
