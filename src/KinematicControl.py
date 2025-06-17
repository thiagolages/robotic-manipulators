from dataclasses import dataclass
from typing import List, Tuple

import numpy as np
import roboticstoolbox as rtb
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
        self, robot, K=1.0, dt=0.01, pos_threshold=0.001, ori_threshold=0.01, verbose=False
    ):
        super().__init__(robot)
        self.K = K
        self.dt = dt
        self.pos_threshold = pos_threshold
        self.ori_threshold = ori_threshold
        self.verbose = verbose  # Flag for verbose output

    def loop(self, target_pose: SE3) -> Tuple[List, List, List]:

        u_list = []  # List to store control inputs for analysis
        err_list = []  # List to store errors for analysis
        q_list = [self.robot.q.copy()]  # List to store joint configurations for analysis

        if self.verbose:
            print("Starting regulation control...")
            print("Target pose: \n", target_pose)
            print("Initial pose \n:", self.robot.fkine(self.robot.q))
            print("Initial joint configuration:", self.robot.q)

        # Initialize error
        error = np.ones(6)  # x,y,z and axis-angle (multiplied by angle)
        norm_pos_err = np.linalg.norm(error[:3])
        norm_ori_err = np.linalg.norm(error[3:])

        while norm_pos_err >= self.pos_threshold or norm_ori_err >= self.ori_threshold:
            eff_pose = self.robot.fkine(self.robot.q)  # Get current end-effector pose (SE3)

            J = self.robot.jacob0(self.robot.q)  # Get the Jacobian

            pos_err = target_pose.t - eff_pose.t  # Position error

            nphi = rotm2axang2(target_pose.R @ eff_pose.R.T)  # axis-angle form

            nphi_err = nphi[:3] * nphi[3]  # Orientation error (n*phi)

            error = np.concatenate((pos_err, nphi_err))

            u = self.K * np.linalg.pinv(J) @ error  # Compute control input

            self.robot.q += u * self.dt  # Trapezoidal integration for joint positions

            q_list.append(self.robot.q.copy())  # Store joint configuration for analysis
            u_list.append(u)  # Store control input for analysis
            err_list.append(error)  # Store error for analysis
            norm_pos_err = np.linalg.norm(error[:3])
            norm_ori_err = np.linalg.norm(error[3:])

            if self.verbose:
                print(f"eff_pose: {eff_pose}")
                print(f"J: {J}")
                print(f"pos_err: {pos_err}, pos_err.shape: {pos_err.shape}")
                print("target_pose.R @ eff_pose.R.T = \n", target_pose.R @ eff_pose.R.T)
                print(f"nphi: {nphi}")
                print(f"nphi_err: {nphi_err}, nphi_err.shape: {nphi_err.shape}")
                print(f"error: {error}, error.shape: {error.shape}")
                print(f"u: {u}")
                print(f"q: {self.robot.q}")
                print(
                    f"Error norm: {norm_pos_err:.4f}, \
                    norm_ori_error: {norm_ori_err:.4f}, \
                    Control input: {u}"
                )

        if self.verbose:
            print("Error below threshold, stopping control.")
            print("Final joint configuration:", self.robot.q)

        return KinematicControlResult(q_list, err_list, u_list)


class TrajectoryControl(KinematicControl):
    def __init__(
        self, robot, K=1.0, dt=0.05, pos_threshold=0.001, ori_threshold=0.01, verbose=False
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
        self.pos_threshold = pos_threshold
        self.ori_threshold = ori_threshold
        self.verbose = verbose  # Flag for verbose output

    def generate_circle_trajectory(self, radius=None, num_points=500) -> List[SE3]:
        """
        Generate a circular trajectory in the YZ plane with a fixed X value (X = 1.0).
        The circle is centered at (X=1.0, Y=0, Z=z_height) with a specified radius.
        """
        if radius is None:  # Account for no radius specified
            radius = 0.5

        eff_pose = self.robot.fkine(self.robot.q)  # Get the current end-effector pose
        x_fixed = eff_pose.t[0]  # Use the current X position of the end-effector
        y_curr, z_curr = eff_pose.t[1:3]
        # circle center
        y0 = y_curr
        z0 = z_curr + radius
        theta = np.linspace(0, 2 * np.pi, num_points)
        # Starting from P6
        y = y0 + radius * np.sin(theta)  # sin(theta) equals cos(theta-90)
        z = z0 + radius * (-np.cos(theta))  # -cos(theta) equals sin(theta-90)
        trajectory = []
        for i in range(num_points):
            T = SE3(np.eye(4)) @ SE3(rtb.ET.Ry(np.pi / 2).A())
            T.A[:3, 3] = np.array([x_fixed, y[i], z[i]])
            trajectory.append(SE3(T))
        return trajectory

    def generate_pose_trajectory(
        self, target_pose: SE3, min_num_points=150, radius=None
    ) -> List[SE3]:

        if radius:
            return self.generate_circle_trajectory(radius=radius)

        initial_pose = self.robot.fkine(self.robot.q)  # Get the initial pose

        num_steps = max(
            int(np.linalg.norm(target_pose.t - initial_pose.t) / self.dt), min_num_points
        )

        xd_t = []
        x_arr = np.linspace(initial_pose.t[0], target_pose.t[0], num_steps + 1)
        y_arr = np.linspace(initial_pose.t[1], target_pose.t[1], num_steps + 1)
        z_arr = np.linspace(initial_pose.t[2], target_pose.t[2], num_steps + 1)

        if self.verbose:
            print(f"Generating pose trajectory with {num_steps} steps...")
            print(f"Initial pose: {initial_pose}")
            print(f"Target pose: {target_pose}")

            print(
                f"Initial pose t[0]: \n{initial_pose.t[0]}, \
                t[1]: \n{initial_pose.t[1]}, \
                t[2]: \n{initial_pose.t[2]}"
            )
            print(
                f"Target pose t[0]: \n{target_pose.t[0]}, \
                t[1]: \n{target_pose.t[1]}, \
                t[2]: \n{target_pose.t[2]}"
            )
            print("x_arr = \n{}, y_arr = \n{}, z_arr = \n{}".format(x_arr, y_arr, z_arr))

        for i in range(num_steps + 1):
            # Get the orientation from the initial pose, which will remain constant
            R = initial_pose.R
            t = np.array([x_arr[i], y_arr[i], z_arr[i]])
            T = np.eye(4)  # Create a 4x4 identity matrix
            T[:3, :3] = R  # Set the rotation part
            T[:3, 3] = t  # Set the translation part
            pose = SE3(T)  # Create a new SE3 object with the same orientation and new translation
            xd_t.append(pose)  # Append the pose to the trajectory

        if self.verbose:
            print("Generated pose trajectory:")
            for i, xd_t_element in enumerate(xd_t):
                print(f"Step {i}: Pose = \n{xd_t_element}")

        return xd_t

    def generate_velocity_trajectory(self, pose_trajectory: List[SE3]) -> List[np.ndarray]:
        """
        Generate a velocity trajectory based on the pose trajectory.
        The velocity is computed as the diff between consecutive poses divided by the timestep.
        """
        velocity_trajectory = []
        for i in range(len(pose_trajectory) - 1):
            delta_pos = pose_trajectory[i + 1].t - pose_trajectory[i].t
            velocity = delta_pos / self.dt
            # angular velocity will be zero, since eff will keep its orientation
            velocity = np.concatenate((velocity, np.zeros(3)))  # Append zero angular velocity
            velocity_trajectory.append(velocity)

        if self.verbose:
            print(f"Generated velocity trajectory: {velocity_trajectory}")

        return velocity_trajectory

    def loop(self, target_pose: SE3, radius=None) -> Tuple[List, List]:
        """
        Regulate the robot's end-effector to a desired pose target_pose.
        """

        xd_arr = self.generate_pose_trajectory(
            target_pose, radius=radius
        )  # Generate the pose trajectory
        xd_dot_arr = self.generate_velocity_trajectory(xd_arr)  # Generate the velocity trajectory

        u_list = []  # List to store control inputs for analysis
        err_list = []  # List to store errors for analysis
        q_list = [self.robot.q.copy()]  # List to store joint configurations for analysis

        if self.verbose:
            print("Initial joint configuration:", self.robot.q)
            print("Initial pose:", self.robot.fkine(self.robot.q))
            print("Target pose for trajectory control: \n", target_pose)
            print(
                "Starting trajectory control between points \n{} \nand \n{}...".format(
                    xd_arr[0], xd_arr[-1]
                )
            )

        # Initialize error to a large value
        error = np.inf * np.ones(6)  # x,y,z and axis-angle (multiplied by angle)
        i = 0  # Index for trajectory points
        norm_pos_err = np.linalg.norm(error[:3])
        norm_ori_err = np.linalg.norm(error[3:])

        while i < len(xd_dot_arr):  # since velocity has 1 less element
            xd = xd_arr[i]  # Get the target pose from the trajectory
            xd_dot = xd_dot_arr[i]  # Get the target velocity from the trajectory

            eff_pose = self.robot.fkine(self.robot.q)  # Get current end-effector pose (SE3)

            pos_err = xd.t - eff_pose.t  # Position error

            # Calculate error vector
            nphi = rotm2axang2(xd.R @ eff_pose.R.T)  # axis-angle form
            nphi_err = nphi[:3] * nphi[3]  # Orientation error (n*phi)

            error = np.concatenate((pos_err, nphi_err))

            v = xd_dot + self.K * error  # Proportional control for combined error

            J = self.robot.jacob0(self.robot.q)  # Get the Jacobian

            u = np.linalg.pinv(J) @ v  # Compute control input

            self.robot.q += u * self.dt  # Update joint states

            q_list.append(self.robot.q.copy())
            u_list.append(u)  # Store control input for analysis
            err_list.append(error)  # Store error for analysis

            norm_pos_err = np.linalg.norm(error[:3])
            norm_ori_err = np.linalg.norm(error[3:])
            norm_err = np.linalg.norm(error)

            if norm_pos_err >= self.pos_threshold or norm_ori_err >= self.ori_threshold:
                i += 1

            # Print all variables and their shapes
            if self.verbose:
                print(f"Step {i}:")
                print(f"xd: {xd}, xd.t shape: {xd.t.shape}")
                print(f"xd_dot: {xd_dot}, xd_dot shape: {xd_dot.shape}")
                print(f"eff_pose: {eff_pose}, eff_pose.t shape: {eff_pose.t.shape}")
                print(f"pos_err: {pos_err}, pos_err shape: {pos_err.shape}")
                print(f"Error norm: {norm_err:.4f}, Control input: {u}")
                print(
                    f"Position error: {norm_pos_err:.4f}, \
                    Orientation error: {norm_ori_err:.4f}, \
                    Control input: {u}"
                )

        if self.verbose:
            print("Error below threshold, stopping control.")
            print("Final joint configuration:", self.robot.q)

        return KinematicControlResult(q_list, err_list, u_list)
