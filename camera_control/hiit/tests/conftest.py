# Author: Andrew England (andrewengland19)
# Created: 2026-06-08
# Last updated: 2026-06-08
"""Put the inner ``camera_control`` package dir on sys.path so ``import hiit.*``
works when running pytest from anywhere, without requiring camera_control to be
an installed/colcon package."""

import sys
from pathlib import Path

# this file: <repo>/camera_control/hiit/tests/conftest.py
# parents[1] == <repo>/camera_control (the dir that contains the `hiit` package)
INNER_PKG_DIR = Path(__file__).resolve().parents[1].parent
if str(INNER_PKG_DIR) not in sys.path:
    sys.path.insert(0, str(INNER_PKG_DIR))
