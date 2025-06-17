import math

import numpy as np
from numpy.linalg import norm  # svd


def rotm2axang2(R: np.ndarray) -> np.ndarray:
    T = np.eye(4)
    Td = np.eye(4)
    Td[:3, :3] = R
    e = angle_axis(T, Td)
    axang = np.zeros(4)
    axang[:3] = e[3:]  # Axis
    axang[3] = norm(e[3:])  # Angle
    if axang[3] < 1e-6:
        axang[:3] = np.array([1.0, 0.0, 0.0])  # Default axis if angle is very small
    else:
        axang[:3] /= axang[3]  # Normalize the axis

    return axang


# From the roboticstoolbox-python tutorials:
# https://github.com/jhavl/dkt/blob/main/Part%201/3%20Resolved-Rate%20Motion%20Control.ipynb
def angle_axis(T: np.ndarray, Td: np.ndarray) -> np.ndarray:
    """
    Returns the error vector between T and Td in angle-axis form.

    :param T: The current pose
    :param Td: The desired pose

    :returns e: the error vector between T and Td
    """

    e = np.empty(6)

    # The position error
    e[:3] = Td[:3, -1] - T[:3, -1]

    R = Td[:3, :3] @ T[:3, :3].T

    li = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])

    if norm(li) < 1e-6:
        # If li is a zero vector (or very close to it)

        # diagonal matrix case
        if np.trace(R) > 0:
            # (1,1,1) case
            a = np.zeros((3,))
        else:
            a = np.pi / 2 * (np.diag(R) + 1)
    else:
        # non-diagonal matrix case
        ln = norm(li)
        a = math.atan2(ln, np.trace(R) - 1) * li / ln

    e[3:] = a

    return e


# FIXME: Implementation done by @thiagolages, needs to be fixed.
# def rotm2axang2(R: np.ndarray) -> np.ndarray:
#     """
#     Convert one or more 3-D rotation matrices to axis-angle form.
#     Always returns a positive axis, and the angle may be positive or negative.
#     """

#     R = np.asanyarray(R, dtype=float)
#     R_flat = R.reshape(-1, 3, 3)
#     N = R_flat.shape[0]

#     trace = R_flat[:, 0, 0] + R_flat[:, 1, 1] + R_flat[:, 2, 2]
#     theta = np.arccos(np.clip((trace - 1.0) / 2.0, -1.0, 1.0))

#     axes = np.empty((N, 3))
#     v_raw = np.stack(
#         [
#             R_flat[:, 2, 1] - R_flat[:, 1, 2],
#             R_flat[:, 0, 2] - R_flat[:, 2, 0],
#             R_flat[:, 1, 0] - R_flat[:, 0, 1],
#         ],
#         axis=-1,
#     )

#     sin_theta = np.sin(theta)
#     eps = np.finfo(float).eps
#     non_sing = (np.abs(sin_theta) > eps) & (theta > eps) & (np.abs(theta - np.pi) > eps)
#     sing_zero = theta <= eps  # θ ≈ 0
#     sing_pi = np.abs(theta - np.pi) <= eps  # θ ≈ π

#     # Regular (non-singular) rotations
#     if np.any(non_sing):
#         axes[non_sing] = v_raw[non_sing] / (2.0 * sin_theta[non_sing, None])

#     # θ ≈ 0 (identity rotation)
#     if np.any(sing_zero):
#         axes[sing_zero] = np.tile(np.array([1.0, 0.0, 0.0]), (np.count_nonzero(sing_zero), 1))

#     # θ ≈ π (singular, 180 deg)
#     if np.any(sing_pi):
#         for idx in np.nonzero(sing_pi)[0]:
#             Ri = R_flat[idx]
#             _, _, Vt = svd(np.eye(3) - Ri)
#             axis = Vt[-1]
#             if norm(axis) < eps:
#                 axis = np.array([1.0, 0.0, 0.0])
#             axes[idx] = axis / norm(axis)

#     # Ensure axis is always positive (first nonzero component positive)
#     for i in range(N):
#         axis = axes[i]
#         angle = theta[i]
#         # Find first nonzero component
#         for j in range(3):
#             if np.abs(axis[j]) > eps:
#                 if axis[j] < 0:
#                     axis[j] = -axis[j]
#                     angle = -angle
#                 break
#         axes[i] = axis
#         theta[i] = angle

#     axang_flat = np.concatenate([axes, theta[:, None]], axis=1)
#     axang = axang_flat.reshape(*R.shape[:-2], 4)
#     return axang
