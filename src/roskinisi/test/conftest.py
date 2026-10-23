# File: conftest.py
"""Allow portable driver tests to import the source package without a colcon build."""

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
