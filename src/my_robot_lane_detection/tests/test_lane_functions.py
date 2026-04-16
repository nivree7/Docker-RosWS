"""
CFG-Based Unit Tests for YOLOP Lane Detection Functions
=======================================================
Coverage criterion: Edge Coverage (Sections 1 & 2), Decision Coverage (Section 3)

Functions under test (all from yolop_lane_node.py):
  - extract_lane_edges_per_row
  - remove_small_components
  - hsv_white_lane_mask

Run from the my_robot_lane_detection directory:
  python3 -m pytest tests/test_lane_functions.py -v
"""

import os
import sys
import types
import importlib.util
from unittest import mock

# ---------------------------------------------------------------------------
# Mock heavy dependencies so the module loads without a full ROS/YOLOP stack.
# The three functions under test only use numpy and cv2 — nothing else.
# ---------------------------------------------------------------------------
_MOCK_MODULES = [
    "rclpy", "rclpy.node", "rclpy.qos",
    "sensor_msgs", "sensor_msgs.msg",
    "cv_bridge",
    "torch",
    "torchvision", "torchvision.transforms",
    "yolop_lane_ros2.msg",
    "lib", "lib.config", "lib.models",
    "lib.utils", "lib.utils.utils",
]
for _m in _MOCK_MODULES:
    if _m not in sys.modules:
        sys.modules[_m] = mock.MagicMock()

# Load yolop_lane_node.py directly by file path (avoids package import issues)
_NODE_PATH = os.path.abspath(
    os.path.join(
        os.path.dirname(__file__),
        "..", "src", "yolop_lane_ros2",
        "yolop_lane_ros2", "yolop_lane_node.py",
    )
)
_spec = importlib.util.spec_from_file_location("yolop_lane_node", _NODE_PATH)
_mod = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_mod)

extract_lane_edges_per_row = _mod.extract_lane_edges_per_row
remove_small_components    = _mod.remove_small_components
hsv_white_lane_mask        = _mod.hsv_white_lane_mask

import numpy as np


# ===========================================================================
# Section 1 — extract_lane_edges_per_row  (Edge Coverage)
# ===========================================================================
#
# CFG Edges:
#   TR1  Entry → loop test
#   TR2  loop: y < H → True  (enter body)
#   TR3  loop: y < H → False (exit loop)
#   TR4  xs computed
#   TR5  xs.size == 0 → True  (continue)
#   TR6  xs.size > 0  → True  (compute & append)
#   TR7  loop-back after continue
#   TR8  loop-back after append
# ===========================================================================

def test_empty_mask():
    """All-zero mask → no lane pixels anywhere → returns [].
    Covers TR1, TR2 (loop entered), TR3, TR4, TR5 (all rows empty), TR7."""
    mask = np.zeros((10, 10), dtype=np.uint8)
    assert extract_lane_edges_per_row(mask, row_step=5) == []


def test_full_white_row():
    """Row 0 is fully white (cols 0-9), row_step=5.
    Only row 0 is sampled. left_x=0, right_x=9, center_x=4.5.
    Covers TR1, TR2, TR3, TR4, TR6 (row has pixels), TR8."""
    mask = np.zeros((5, 10), dtype=np.uint8)
    mask[0, :] = 255
    result = extract_lane_edges_per_row(mask, row_step=5)
    assert result[0] == (0, 0, 9, 4.5)


def test_zero_height():
    """Mask with 0 rows → loop never entered → returns [].
    Covers TR1, TR3 (loop exits immediately)."""
    mask = np.zeros((0, 10), dtype=np.uint8)
    assert extract_lane_edges_per_row(mask, row_step=5) == []


def test_mixed_rows():
    """Row 0 has pixels at cols 2 and 7 (TR6, TR8).
    Row 5 is empty (TR5, TR7). left_x=2, right_x=7, center_x=4.5.
    Covers all TRs (TR1-TR8)."""
    mask = np.zeros((10, 10), dtype=np.uint8)
    mask[0, 2] = 255
    mask[0, 7] = 255
    result = extract_lane_edges_per_row(mask, row_step=5)
    assert result[0] == (0, 2, 7, 4.5)


# ===========================================================================
# Section 2 — remove_small_components  (Edge Coverage)
# ===========================================================================
#
# CFG Edges:
#   TR1   Entry
#   TR2   min_area <= 0 → True  (early return)
#   TR3   min_area > 0  → False (continue)
#   TR4   Enter loop test
#   TR5   i < num → True  (loop body)
#   TR6   i < num → False (loop exits)
#   TR7   area checked
#   TR8   area >= min_area → True  (keep component)
#   TR9   area <  min_area → False (skip component)
#   TR10  loop-back after keep
# ===========================================================================

def test_min_area_zero():
    """min_area=0 triggers early return — mask is returned unchanged.
    Covers TR1, TR2."""
    mask = np.ones((5, 5), dtype=np.uint8) * 255
    result = remove_small_components(mask, min_area=0)
    assert np.array_equal(result, mask)


def test_all_black():
    """All-zero mask → connected-components finds only background →
    output is all zero. Covers TR1, TR3, TR4, TR6 (no foreground components)."""
    mask = np.zeros((10, 10), dtype=np.uint8)
    result = remove_small_components(mask, min_area=100)
    assert np.count_nonzero(result) == 0


def test_large_kept():
    """5×5 white block = 25 pixels ≥ min_area=10 → all 25 kept.
    Covers TR1, TR3, TR4, TR5, TR6, TR7, TR8, TR10."""
    mask = np.zeros((10, 10), dtype=np.uint8)
    mask[2:7, 2:7] = 255
    result = remove_small_components(mask, min_area=10)
    assert np.count_nonzero(result) == 25


def test_small_removed():
    """Single-pixel component (1 px) < min_area=5 → removed → 0 nonzero.
    Covers TR1, TR3, TR4, TR5, TR6, TR7, TR9."""
    mask = np.zeros((10, 10), dtype=np.uint8)
    mask[0, 0] = 255
    result = remove_small_components(mask, min_area=5)
    assert np.count_nonzero(result) == 0


def test_mixed():
    """3×3 block (9 px) kept + single pixel (1 px) removed, min_area=5.
    Output has exactly 9 nonzero pixels. Covers all TRs (TR1-TR10)."""
    mask = np.zeros((10, 10), dtype=np.uint8)
    mask[2:5, 2:5] = 255   # 9-pixel component
    mask[8, 8]     = 255   # 1-pixel component
    result = remove_small_components(mask, min_area=5)
    assert np.count_nonzero(result) == 9


# ===========================================================================
# Section 3 — hsv_white_lane_mask  (Decision Coverage)
# ===========================================================================
#
# Decision nodes:
#   D1  close_ksize and close_ksize > 1 → True/False
#   D2  k % 2 == 0                      → True/False
#
# Test Requirements:
#   TR1  D1 → True  (morphology applied)
#   TR2  D1 → False (morphology skipped)
#   TR3  D2 → True  (even k, bump to odd)
#   TR4  D2 → False (k already odd, use as-is)
# ===========================================================================

def test_hsv_odd():
    """close_ksize=5 (odd, >1): D1=True, D2=False → morphology applied with k=5.
    Covers TR1, TR4."""
    img = np.ones((10, 10, 3), dtype=np.uint8) * 255
    result = hsv_white_lane_mask(img, close_ksize=5)
    assert result.shape == (10, 10)


def test_hsv_zero():
    """close_ksize=0: D1=False → morphology skipped entirely.
    Covers TR2."""
    img = np.ones((10, 10, 3), dtype=np.uint8) * 255
    result = hsv_white_lane_mask(img, close_ksize=0)
    assert result.shape == (10, 10)


def test_hsv_even():
    """close_ksize=4 (even, >1): D1=True, D2=True → k bumped to 5 then morphology applied.
    Covers TR1, TR3."""
    img = np.ones((10, 10, 3), dtype=np.uint8) * 255
    result = hsv_white_lane_mask(img, close_ksize=4)
    assert result.shape == (10, 10)
