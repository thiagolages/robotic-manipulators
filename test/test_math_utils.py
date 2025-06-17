# isort: off
import sys
import os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../src")))  # noqa: E402
# isort: on
# flake8: noqa: E402

import unittest

import numpy as np
from parameterized import parameterized
from roboticstoolbox import ET

from math_utils import rotm2axang2


def axis_angle_close(a, b, atol=1e-8):
    a = np.atleast_1d(a)
    b = np.atleast_1d(b)
    axis_a, axis_b = a[:3], b[:3]
    angle_a, angle_b = a[3], b[3]
    return (np.allclose(axis_a, axis_b, atol=atol) and np.isclose(angle_a, angle_b, atol=atol)) or (
        np.allclose(axis_a, -axis_b, atol=atol) and np.isclose(angle_a, angle_b, atol=atol)
    )


def wrap_to_pi(angle_rad, tol=1e-8):
    """
    Wrap angle(s) in radians to the range (-π, π], with special handling for values close to π.
    """
    wrapped = (angle_rad + np.pi) % (2 * np.pi) - np.pi
    # If wrapped is close to -π, but original was close to +π, map to +π
    if np.isclose(wrapped, -np.pi, atol=tol) and np.isclose(angle_rad, np.pi, atol=tol):
        return np.pi
    return wrapped


class TestRotm2Axang2(unittest.TestCase):
    @parameterized.expand(
        [
            ("x", np.pi / 4),  # 0
            ("x", np.pi / 2),  # 1
            ("x", 3 * np.pi / 4),  # 2
            ("x", np.pi),  # 3
            ("x", 3 * np.pi / 2),  # 4
            ("x", 2 * np.pi - 0.01),  # 5
            ("y", np.pi / 4),  # 6
            ("y", np.pi / 2),  # 7
            ("y", 3 * np.pi / 4),  # 8
            ("y", np.pi),  # 9
            ("y", 3 * np.pi / 2),  # 10
            ("y", 2 * np.pi - 0.01),  # 11
            ("z", np.pi / 4),  # 12
            ("z", np.pi / 2),  # 13
            ("z", 3 * np.pi / 4),  # 14
            ("z", np.pi),  # 15
            ("z", 3 * np.pi / 2),  # 16
            ("z", 2 * np.pi - 0.01),  # 17
        ]
    )
    def test_rotation_about(self, axis, angle):
        if axis == "x":
            rot = ET.Rx()
        elif axis == "y":
            rot = ET.Ry()
        elif axis == "z":
            rot = ET.Rz()
        R = rot.A(angle)[:3, :3]  # Get the 3x3 rotation matrix
        axang = rotm2axang2(R)
        axis_vec = axang[:3]
        angle = axang[3]

        expected = [0.0] * 3
        expected[["x", "y", "z"].index(axis)] = 1.0  # Set the correct axis to 1.0
        expected.extend([angle])
        expected = np.array(expected)  # Add angle at the end
        print(
            "axis_vec = {}, angle = {}, expected = {}, result = {}".format(
                axis_vec, angle, expected, axis_angle_close(axang, expected)
            )
        )
        self.assertTrue(axis_angle_close(axang, expected))


if __name__ == "__main__":
    unittest.main()
