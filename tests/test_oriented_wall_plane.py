"""Tests for the offline centred flat-wall orientation diagnostic."""

import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch

HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parent
HOST_PYTHON = REPO_ROOT / "host" / "python"
if str(HOST_PYTHON) not in sys.path:
    sys.path.insert(0, str(HOST_PYTHON))

import rangeweave_reference_plane as reference_plane
import view_oriented_wall_plane as wall_view


def normal_ry(degrees):
    angle = math.radians(float(degrees))
    return (math.sin(angle), 0.0, math.cos(angle))


def fit_with_normal(normal):
    return reference_plane.PlaneFit(
        normal=normal,
        centroid_mm=(0.0, 0.0, 500.0),
        offset_mm=500.0,
        rms_residual_mm=1.0,
        max_abs_residual_mm=2.0,
        point_count=64,
    )


class CentredWallPlaneTests(unittest.TestCase):
    def test_centred_window_uses_future_and_past_frames_offline(self):
        # Put a single angular transient two frames *after* index 5.  A centred
        # five-frame window at index 5 must include it; the old trailing window
        # (indices 1..5) would not.
        fits = [fit_with_normal(normal_ry(0.0)) for _ in range(11)]
        fits[7] = fit_with_normal(normal_ry(10.0))
        replay = SimpleNamespace(
            frames=tuple(SimpleNamespace(points_reference=(index,)) for index in range(11))
        )

        with patch.object(wall_view, "_accepted_fit", side_effect=fits):
            _, diagnostics, _ = wall_view.analyse_planes(replay, window=5)

        expected = reference_plane.mean_direction(fit.normal for fit in fits[3:8])
        old_trailing = reference_plane.mean_direction(fit.normal for fit in fits[1:6])
        actual = diagnostics[5].smoothed_normal

        self.assertIsNotNone(actual)
        self.assertLess(reference_plane.angle_deg(actual, expected), 1.0e-9)
        self.assertGreater(reference_plane.angle_deg(actual, old_trailing), 1.0)

    def test_edges_have_no_centred_value_until_full_window_exists(self):
        fits = [fit_with_normal(normal_ry(0.0)) for _ in range(11)]
        replay = SimpleNamespace(
            frames=tuple(SimpleNamespace(points_reference=(index,)) for index in range(11))
        )

        with patch.object(wall_view, "_accepted_fit", side_effect=fits):
            _, diagnostics, _ = wall_view.analyse_planes(replay, window=5)

        self.assertIsNone(diagnostics[0].smoothed_normal)
        self.assertIsNone(diagnostics[1].smoothed_normal)
        self.assertIsNotNone(diagnostics[2].smoothed_normal)
        self.assertIsNotNone(diagnostics[-3].smoothed_normal)
        self.assertIsNone(diagnostics[-2].smoothed_normal)
        self.assertIsNone(diagnostics[-1].smoothed_normal)

    def test_instantaneous_error_is_retained_alongside_smoothed_error(self):
        fits = [fit_with_normal(normal_ry(0.0)) for _ in range(11)]
        fits[5] = fit_with_normal(normal_ry(4.0))
        replay = SimpleNamespace(
            frames=tuple(SimpleNamespace(points_reference=(index,)) for index in range(11))
        )

        with patch.object(wall_view, "_accepted_fit", side_effect=fits):
            _, diagnostics, _ = wall_view.analyse_planes(replay, window=5)

        self.assertIsNotNone(diagnostics[5].instant_angular_error_deg)
        self.assertIsNotNone(diagnostics[5].smoothed_angular_error_deg)
        self.assertGreater(
            diagnostics[5].instant_angular_error_deg,
            diagnostics[5].smoothed_angular_error_deg,
        )

    def test_even_centred_window_is_rejected(self):
        replay = SimpleNamespace(frames=tuple(SimpleNamespace(points_reference=()) for _ in range(11)))
        with self.assertRaisesRegex(reference_plane.ReferencePlaneError, "must be odd"):
            wall_view.analyse_planes(replay, window=4)


if __name__ == "__main__":
    unittest.main()
