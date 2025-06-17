from dataclasses import dataclass
from typing import List, Tuple

import numpy as np
from spatialmath import SE3

from ComauRobot import ComauRobot
from math_utils import rotm2axang2


@dataclass
class KinematicControlResult:
    q_list: List
    err_list: List
    u_list: List


class KinematicControl:
    """
    Base class for kinematic control of robotic manipulators.
    This class provides a template for implementing kinematic control methods.
    """

    def __init__(self, robot: ComauRobot):
        self.robot = robot

    def loop(self, *args, **kwargs):
        raise NotImplementedError("This method should be implemented by subclasses.")


class RegulationControl(KinematicControl):
    def __init__(
        self,
        robot,
        K=3.5,
        dt=0.01,
        threshold=0.01,
    ):
        super().__init__(robot)
        self.K = K
        self.dt = dt
        self.threshold = threshold

    def loop(self, target_pose: SE3) -> Tuple[List, List, List]:

        u_list = []  # List to store control inputs for analysis
        err_list = []  # List to store errors for analysis
        q_list = [self.robot.q.copy()]  # List to store joint configurations for analysis

        print("Starting regulation control...")
        print("Target pose: \n", target_pose)
        print("Initial pose \n:", self.robot.fkine(self.robot.q))
        print("Initial joint configuration:", self.robot.q)

        # Initialize error
        error = np.ones(6)  # x,y,z and axis-angle (multiplied by angle)
        norm_pos_err = np.linalg.norm(error[:3])
        norm_ori_err = np.linalg.norm(error[3:])
        threshold_pos = 0.01
        threshold_ori = 0.03
        while norm_pos_err >= threshold_pos or norm_ori_err >= threshold_ori:
            eff_pose = self.robot.fkine(self.robot.q)  # Get current end-effector pose (SE3)
            print(f"eff_pose: {eff_pose}")

            J = self.robot.jacob0(self.robot.q)  # Get the Jacobian
            # print(f"J: {J}")

            pos_err = target_pose.t - eff_pose.t  # Position error
            print(f"pos_err: {pos_err}, pos_err.shape: {pos_err.shape}")

            print("target_pose.R @ eff_pose.R.T = \n", target_pose.R @ eff_pose.R.T)
            nphi = rotm2axang2(target_pose.R @ eff_pose.R.T)  # axis-angle form
            print(f"nphi: {nphi}")

            nphi_err = nphi[:3] * nphi[3]  # Orientation error (n*phi)
            print(f"nphi_err: {nphi_err}, nphi_err.shape: {nphi_err.shape}")

            error = np.concatenate((pos_err, nphi_err))
            print(f"error: {error}, error.shape: {error.shape}")

            u = self.K * np.linalg.pinv(J) @ error  # Compute control input
            print(f"u: {u}")

            self.robot.q += u * self.dt  # Trapezoidal integration for joint positions
            print(f"q: {self.robot.q}")

            q_list.append(self.robot.q.copy())  # Store joint configuration for analysis
            u_list.append(u)  # Store control input for analysis
            err_list.append(error)  # Store error for analysis
            norm_pos_err = np.linalg.norm(error[:3])
            norm_ori_err = np.linalg.norm(error[3:])
            print(
                f"Error norm: {norm_pos_err:.4f}, \
                norm_ori_error: {norm_ori_err:.4f}, \
                Control input: {u}"
            )

        print("Error below threshold, stopping control.")
        print("Final joint configuration:", self.robot.q)

        return KinematicControlResult(q_list, err_list, u_list)


# FIXME: implement loop properly
class TrajectoryControl(KinematicControl):
    def __init__(
        self,
        robot,
        K=1.0,
        dt=0.1,
    ):
        """
        KinematicControl class for controlling a robotic manipulator using kinematic control laws.

            Attributes:
                K (float): Gain for the control law.
                dt (float): Time step for the control loop.
                target_pose (SE3 or None): Desired end-effector pose as a SE3 object.

            Args:
                robot: The robot model to be controlled.
                initial_q: Initial joint configuration of the robot.
                pose_trajectory (List[SE3]): List of desired end-effector poses (Xd).
                velocity_trajectory (List[SE3]): List of desired end-effector velocities (Xd_dot).
                K (float, optional): Gain for the control law. Default is 1.0.
                dt (float, optional): Time step for the control loop. Default is 0.1.
            initial_q,
            pose_trajectory: List[SE3],
            velocity_trajectory: List[SE3],
            K=1.0,
            dt=0.1,
        """
        super().__init__(robot)
        self.K = K
        self.dt = dt

