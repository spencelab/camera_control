# Author: Andrew England (andrewengland19)
"""Put the inner ``camera_control`` package dir on sys.path so ``import
automation.*`` (and ``import hiit.*``) works when running pytest from anywhere,
without requiring camera_control to be an installed/colcon package."""

import sys
from pathlib import Path

# this file: <repo>/camera_control/automation/tests/conftest.py
# parents[1] == <repo>/camera_control/automation ; .parent == the dir with the pkg
INNER_PKG_DIR = Path(__file__).resolve().parents[1].parent
if str(INNER_PKG_DIR) not in sys.path:
    sys.path.insert(0, str(INNER_PKG_DIR))
