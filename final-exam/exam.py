import numpy as np
from ExamRobots import CustomRobot, ElbowRobot, ThreeLinkRobot, TwoLinkPlanarDynamics

np.set_printoptions(linewidth=100, suppress=True)


class FinalExam:
    def __init__(self):
        self.threelink_robot = ThreeLinkRobot(
            l1=2, l2=3, l3=2, theta1=270, theta2=90, theta3=-270, Fx=3, Fy=1, Fz=2
        )
        self.elbow_robot = ElbowRobot(
            l1=1,
            l2=3,
            theta1=-180,
            d2=1,
            Fx=3,
            Fy=1,
        )
        self.twolink_dynamics_robot = TwoLinkPlanarDynamics(l1=1, l2=1, m1=1, m2=1, q1d=1, q2d=1)
        self.custom_robot = CustomRobot()

    def exc1(self):
        print("############################")
        print("Dynamics Control Theory Exercise")
        print("############################")
        print("This exercise is theoretical and does not involve code execution.")
        print("Torque is given by J.T @ F, where J is the Jacobian and F is the force vector.")
        print("It can be subtracted from the control input to achieve desired motion.")

    def exc2(self):
        print("############################")
        print("Three Link Robot Exercise")
        print(self.threelink_robot)
        print("############################")
        self.threelink_robot.exc()

    def exc3(self):
        print("############################")
        print("TwoLink Planar Robot Exercise (Dynamics)")
        print("############################")
        self.twolink_dynamics_robot.exc()

    def exc4(self):
        print("############################")
        print("Custom Robot Exercise")
        print("############################")
        self.custom_robot.exc()

    def exc5(self):
        print("############################")
        print("Elbow Robot Exercise")
        print(self.elbow_robot)
        print("############################")
        self.elbow_robot.exc()


exam = FinalExam()
exam.exc1()  # Theoretical, about dynamics control
exam.exc2()  # Three link robot (RRR) statics force analysis
exam.exc3()  # Two link planar dynamics analysis
exam.exc4()  # Custom Robot (RP) static force and manipulability analysis
exam.exc5()  # Elbow Robot (RP) static force and manipulability analysis
