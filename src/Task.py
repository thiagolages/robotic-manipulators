from dataclasses import dataclass
from typing import List

import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3

from CoppeliaSimAPI import CoppeliaSimAPI
from KinematicControl import KinematicControlResult, RegulationControl, TrajectoryControl
from Plotter3D import MockPlotter, Plotter3D


@dataclass
class TaskResult:
    name: str = "Task Result"
    start_pose: SE3 = None
    target_pose: SE3 = None
    control_result: KinematicControlResult = None

    def __str__(self):
        return (
            f"TaskResult(name={self.name},\n"
            f"  start_pose={self.start_pose},\n"
            f"  target_pose={self.target_pose},\n"
            f"  control_result={self.control_result}\n)"
        )


class Task:
    def __init__(
        self,
        sim: CoppeliaSimAPI,
        robot,
        traj_control: TrajectoryControl,
        reg_control: RegulationControl,
        plotter: Plotter3D = None,
        verbose: bool = False,
    ):
        self.dt = sim.dt
        self.robot = robot
        self.traj_control = traj_control
        self.reg_control = reg_control
        self.plotter = plotter if plotter is not None else MockPlotter(verbose=verbose)
        self.verbose = verbose

        self.robot_curr_q = self.robot.qz  # Start at zero configuration
        self.scale = 1.0 / 1000  # Scale factor for the task (mm)

        self.orientation = SE3(rtb.ET.Ry(np.pi / 2).A())  # 90 degrees rotation around y-axis
        self.P0 = SE3(np.array([700, 0, 650]) * self.scale) @ self.orientation
        self.P1 = SE3(np.array([1000, 600, 1000]) * self.scale) @ self.orientation
        self.P2 = SE3(np.array([1000, 600, 300]) * self.scale) @ self.orientation
        self.P3 = SE3(np.array([1000, -600, 300]) * self.scale) @ self.orientation
        self.P4 = SE3(np.array([1000, -600, 1000]) * self.scale) @ self.orientation
        self.calculate_diamond_points()
        self.calculate_circle_points()
        self.fill_task_steps()
        self.print_attributes()

    def print_attributes(self):
        attrs = vars(self)
        for key, value in attrs.items():
            print(f"{key}: \n{value}")

    def fill_task_steps(self):
        # Fill the task steps with the methods that will be executed
        self.task_steps = [
            self.t0_ready_position,
            self.t1_qr_to_P0,
            self.t2_P0_to_P1,
            self.t3_draw_flag,
            self.t4_P1_to_P0,
            self.t5_P0_to_P5,
            self.t6_draw_diamond,
            self.t7_P5_to_P0,
            self.t8_P0_to_P6,
            self.t9_draw_circle,
        ]

    def calculate_diamond_points(self):
        # Calculate the diamond points based on P1, P2, P3, P4
        self.P5 = SE3((self.P4.t + self.P3.t) / 2) @ self.orientation  # Between P4 and P3
        # Skip P6 since it's part of the circle
        self.P7 = SE3((self.P3.t + self.P2.t) / 2) @ self.orientation  # Between P3 and P2
        self.P8 = SE3((self.P2.t + self.P1.t) / 2) @ self.orientation  # Between P2 and P1
        self.P9 = SE3((self.P1.t + self.P4.t) / 2) @ self.orientation  # Between P1 and P4

    def calculate_circle_points(self):
        self.center = (
            SE3((self.P1.t + self.P2.t + self.P3.t + self.P4.t) / 4) @ self.orientation
        )  # Center of P1-P2-P3-P4 square
        width = np.linalg.norm(self.P1.t - self.P4.t)
        height = np.linalg.norm(self.P1.t - self.P2.t)
        self.radius = (width / 2 * height / 2) / np.sqrt(
            (width / 2) ** 2 + (height / 2) ** 2
        )  # Radius of the circle inscribed in the square
        self.P6 = self.center
        self.P6.t[2] -= self.radius  # P6 is the center point minus radius in Z direction

    def compile(self, results: List[TaskResult] = None, name: str = "") -> TaskResult:
        # Compile all task results into a single TaskResult
        # Asign first result
        final_result = TaskResult(name=name, control_result=results[0].control_result)
        final_result.start_pose = results[0].start_pose
        final_result.target_pose = results[-1].target_pose
        for result in results[1:]:  # Exclude first result since it was already added
            final_result.control_result.q_list.extend(result.control_result.q_list)
            final_result.control_result.err_list.extend(result.control_result.err_list)
            final_result.control_result.u_list.extend(result.control_result.u_list)

        return final_result

    # 0 - Zero position to Ready position
    def t0_ready_position(self):
        # Default start joint configuration is zero
        self.plotter.off()
        target_pose = self.robot.fkine(self.robot.qr)  # Ready position
        control_result = self.reg_control.loop(target_pose=target_pose)
        return TaskResult(
            name="0-Zero to Ready",
            start_pose=self.robot.fkine(self.robot.qz),
            target_pose=target_pose,
            control_result=control_result,
        )

    # 1 - Ready position to P0
    def t1_qr_to_P0(self):
        control_result = self.reg_control.loop(target_pose=self.P0)
        return TaskResult(
            name="1-Ready to P0",
            start_pose=self.robot.fkine(self.robot.qr),
            target_pose=self.P0,
            control_result=control_result,
        )

    # 2 - P0 to P1 (first point in flag square)
    def t2_P0_to_P1(self):
        control_result = self.reg_control.loop(target_pose=self.P1)
        return TaskResult(
            name="2-P0 to P1",
            start_pose=self.P0,
            target_pose=self.P1,
            control_result=control_result,
        )

    # TODO: 3 - Draw flag (P1-P4)
    def t3_draw_flag(self):
        self.plotter.green()
        r1 = TaskResult(
            name="3.1-Draw Flag - P1 to P2",
            start_pose=self.P1,
            target_pose=self.P2,
            control_result=self.traj_control.loop(target_pose=self.P2),  # From P1 to P2
        )
        r2 = TaskResult(
            name="3.2-Draw Flag - P2 to P3",
            start_pose=self.P2,
            target_pose=self.P3,
            control_result=self.traj_control.loop(target_pose=self.P3),  # From P2 to P3
        )
        r3 = TaskResult(
            name="3.3-Draw Flag - P3 to P4",
            start_pose=self.P3,
            target_pose=self.P4,
            control_result=self.traj_control.loop(target_pose=self.P4),  # From P3 to P4
        )
        r4 = TaskResult(
            name="3.4-Draw Flag - P4 to P1",
            start_pose=self.P4,
            target_pose=self.P1,
            control_result=self.traj_control.loop(target_pose=self.P1),  # From P4 to P1
        )
        final_res = self.compile([r1, r2, r3, r4], name="3-Draw Flag")

        return final_res

    # 4 - P1 to P0
    def t4_P1_to_P0(self):
        self.plotter.off()
        control_result = self.reg_control.loop(target_pose=self.P0)
        return TaskResult(
            name="4-P1 to P0",
            start_pose=self.P1,
            target_pose=self.P0,
            control_result=control_result,
        )

    # 5 - P0 to P5
    def t5_P0_to_P5(self):
        control_result = self.reg_control.loop(target_pose=self.P5)
        return TaskResult(
            name="5-P0 to P5",
            start_pose=self.P0,
            target_pose=self.P5,
            control_result=control_result,
        )

    # 6 - TODO: Draw diamond (P5-P7-P8-P9, since P6 is in the circle)
    def t6_draw_diamond(self):
        self.plotter.yellow()
        r1 = TaskResult(
            name="6.1-Draw Diamond - P5 to P7",
            start_pose=self.P5,
            target_pose=self.P7,
            control_result=self.traj_control.loop(target_pose=self.P7),  # From P5 to P7
        )
        r2 = TaskResult(
            name="6.2-Draw Diamond - P7 to P8",
            start_pose=self.P7,
            target_pose=self.P8,
            control_result=self.traj_control.loop(target_pose=self.P8),  # From P7 to P8
        )
        r3 = TaskResult(
            name="6.3-Draw Diamond - P8 to P9",
            start_pose=self.P8,
            target_pose=self.P9,
            control_result=self.traj_control.loop(target_pose=self.P9),  # From P8 to P9
        )
        r4 = TaskResult(
            name="6.4-Draw Diamond - P9 to P5",
            start_pose=self.P9,
            target_pose=self.P5,
            control_result=self.traj_control.loop(target_pose=self.P5),  # From P9 to P5
        )
        final_res = self.compile([r1, r2, r3, r4], name="6-Draw Diamond")

        return final_res

    # 7 - P5 to P0
    def t7_P5_to_P0(self):
        self.plotter.off()
        control_result = self.reg_control.loop(target_pose=self.P0)
        return TaskResult(
            name="7-P5 to P0",
            start_pose=self.P5,
            target_pose=self.P0,
            control_result=control_result,
        )

    # 8 - P0 to P6
    def t8_P0_to_P6(self):
        control_result = self.reg_control.loop(target_pose=self.P6)
        return TaskResult(
            name="8-P0 to P6",
            start_pose=self.P0,
            target_pose=self.P6,
            control_result=control_result,
        )

    # TODO: 9 - Draw circle
    def t9_draw_circle(self):
        self.plotter.blue()
        control_result = self.traj_control.loop(target_pose=None, radius=self.radius)
        return TaskResult(
            name="9-Draw Circle",
            start_pose=self.P6,
            target_pose=self.P6,
            control_result=control_result,
        )

    def run(self):
        results = []
        results.append(self.t0_ready_position())
        results.append(self.t1_qr_to_P0())
        results.append(self.t2_P0_to_P1())
        results.append(self.t3_draw_flag())
        results.append(self.t4_P1_to_P0())
        results.append(self.t5_P0_to_P5())
        results.append(self.t6_draw_diamond())
        results.append(self.t7_P5_to_P0())
        results.append(self.t8_P0_to_P6())
        results.append(self.t9_draw_circle())

        final_result = self.compile(results, name="Brazilian Flag")

        return final_result
