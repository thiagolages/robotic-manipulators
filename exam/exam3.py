import numpy as np
import sympy as sp
from numpy import cos, pi, sin
from roboticstoolbox import DHRobot, PrismaticDH, RevoluteDH

np.set_printoptions(linewidth=100, suppress=True)

# q1, q2, q3 = sp.symbols('q1 q2 q3')
# l1, l2, l3 = sp.symbols('l1 l2 l3')


class ThreeLinkRobot(DHRobot):
    def __init__(self, l1, l2, l3, theta1, theta2, theta3, Fx, Fy, Fz):
        L1 = RevoluteDH(a=0, d=l1, alpha=-np.pi / 2, offset=-np.pi / 2)
        L2 = RevoluteDH(a=l2, d=0, alpha=np.pi)
        L3 = RevoluteDH(a=l3, d=0, alpha=-np.pi / 2)
        super().__init__([L1, L2, L3])

        self.l1 = l1
        self.l2 = l2
        self.l3 = l3
        self.theta1 = theta1
        self.theta2 = theta2
        self.theta3 = theta3
        self.Fx = Fx
        self.Fy = Fy
        self.Fz = Fz

    def __str__(self):
        return (
            f"ThreeLinkRobot(l1={self.l1}, l2={self.l2}, l3={self.l3}, "
            f"theta1={self.theta1}, theta2={self.theta2}, theta3={self.theta3}, "
            f"Fx={self.Fx}, Fy={self.Fy}, Fz={self.Fz})"
        )

    def exc(self):
        q_exc = [self.theta1, self.theta2, self.theta3]
        q_exc_rad = np.deg2rad(q_exc)
        F = np.array([self.Fx, self.Fy, self.Fz])
        J = self.jacob0(q_exc_rad)
        print(f"Jacobian at q={q_exc}:\n{np.round(J, 2)}")
        Jred = J[[0, 1, 2], :]  # Select relevant rows for planar robot
        print(f"Jred at q={q_exc}:\n{np.round(Jred, 2)}")
        t = Jred.T @ F
        print(f"Torque at q={q_exc} with force {F} is: {t}")
        print("\n\n")
        print(
            "Jacobian (geometric) = {}".format(
                self.jacob0(
                    [np.deg2rad(self.theta1), np.deg2rad(self.theta2), np.deg2rad(self.theta3)]
                )
            )
        )
        print(
            "Jacobian (analytical) = {}".format(
                self.jacob0_analytical(
                    [np.deg2rad(self.theta1), np.deg2rad(self.theta2), np.deg2rad(self.theta3)]
                )
            )
        )


class ElbowRobot(DHRobot):
    def __init__(self, l1, l2, d2, theta1, Fx, Fy):
        L1 = RevoluteDH(a=l2, d=0, alpha=np.pi / 2, offset=np.pi / 2)
        L2 = PrismaticDH(a=0, theta=0, alpha=0, offset=l1)
        super().__init__([L1, L2])

        self.Fx = Fx
        self.Fy = Fy
        self.theta1 = theta1
        self.d2 = d2
        self.l1 = l1
        self.l2 = l2

    def __str__(self):
        return (
            f"ElbowRobot(l1={self.l1}, l2={self.l2}, d2={self.d2}, "
            f"theta1={self.theta1}, Fx={self.Fx}, Fy={self.Fy})"
        )

    def exc(self):
        #       theta1, d2
        q_exc = [self.theta1, self.d2]
        q_exc_rad = [np.deg2rad(self.theta1), self.d2]
        F = np.array([self.Fx, self.Fy])
        J = self.jacob0(q_exc_rad)
        print(f"Jacobian at q={q_exc}:\n{np.round(J, 2)}")
        Jred = J[[0, 1], :]  # Select relevant rows for planar robot
        print(f"Jred at q={q_exc}:\n{np.round(Jred, 2)}")
        t = Jred.T @ F
        print(f"Torque at q={q_exc} with force {F} is: {t}")

        manipulability = np.abs(np.linalg.det(Jred))
        print(f"Manipulability at q={q_exc} is: {manipulability}")


class TwoLinkPlanarDynamics(DHRobot):
    def __init__(self, l1, l2, m1, m2, I1=None, I2=None, q1d=None, q2d=None):
        # link 1
        self.m1 = 1
        self.l1 = 1
        self.lc1 = 0.5
        # link 2
        self.m2 = 1
        self.l2 = 1
        self.lc2 = 0.5

        # joint configurations
        self.q1 = [pi, -pi]
        self.q2 = [-pi / 2, 3 * pi / 2]
        self.q3 = [pi / 2, -pi / 2]
        self.q4 = [-pi, 3 * pi / 2]
        self.qq = [self.q1, self.q2, self.q3, self.q4]
        self.z = [0, 0]

        # global parameters
        g = 9.81
        L1 = RevoluteDH(
            a=l1,
            m=m1,
            r=[-self.lc1, 0, 0],
            # inertia tensor with respect to center of mass.
            # [L_xx, L_yy, L_zz, L_xy, L_yz, L_xz]
            I=[1, 1, 1, 0, 0, 0],
        )
        L2 = RevoluteDH(a=l2, m=m2, r=[-self.lc2, 0, 0], I=[1, 1, 1, 0, 0, 0])

        super().__init__([L1, L2], gravity=[0, -g, 0])

    def exc(self):
        print(self)
        print(self.dynamics())

        print("Gravity load for each configuration:")
        for q in self.qq:
            tau = self.gravload(q)
            print("Config {}: {}".format(np.rad2deg(q), tau))


class CustomRobot:
    def __init__(self):
        # Define the robot structure here
        pass

    def exc(self):
        r = 3
        a = 2
        Fx = 3
        Fy = 1
        mz = 1
        # theta1, d2
        t1 = -np.pi  # theta1
        d2 = 3  # prismatic joint length
        beta = np.pi  # custom angle
        q = [t1, d2]

        orig_d2 = d2  # will be used later
        J = np.zeros((3, 2))
        J[0, 0] = -(r + a) * sin(t1) + d2 * cos(t1 + beta)
        J[0, 1] = sin(t1 + beta)
        J[1, 0] = (r + a) * cos(t1) + d2 * sin(t1 + beta)
        J[1, 1] = -cos(t1 + beta)
        J[2, 0] = 1
        J[2, 1] = 0

        F = np.array([Fx, Fy, mz])
        tau = J.T @ F  # Remember to use the transpose of Jacobian !
        print(f"Excercise 3 with Fx={Fx}, Fy={Fy}, mz={mz}")
        print(f"Torque at q={q} with force {F} is: {tau}")
        # Calculate force at radius
        fb = tau[0] / r
        print(f"Force Fb at radius r={r} is: {fb}")
        print(f"Force f2 is: {tau[1]}")

        # letter b)

        # values will change, and d2 will be a symbol
        d2 = sp.symbols("d2")
        beta = np.deg2rad(15)
        t1 = -beta
        r = 1
        a = 1
        J = sp.Matrix.zeros(2, 2)
        J[0, 0] = -(r + a) * sp.sin(t1) + d2 * sp.cos(t1 + beta)
        J[0, 1] = sp.sin(t1 + beta)
        J[1, 0] = (r + a) * sp.cos(t1) + d2 * sp.sin(t1 + beta)
        J[1, 1] = -sp.cos(t1 + beta)

        det = sp.det(J)  # Calculate the determinant of the Jacobian
        d2_solutions = sp.solve(det, d2)
        print(f"Values of d2 for which det(J)=0: {d2_solutions}")

        print(("Test to see matrix determinant is actually zero with solution:"))
        J = np.zeros((2, 2))

        J[0, 0] = -(r + a) * sin(t1) + orig_d2 * cos(t1 + beta)
        J[0, 1] = sin(t1 + beta)
        J[1, 0] = (r + a) * cos(t1) + orig_d2 * sin(t1 + beta)
        J[1, 1] = -cos(t1 + beta)
        print("Determinant with original orig_d2 = {}: {}".format(orig_d2, np.linalg.det(J)))

        calculated_d2 = d2_solutions[0] if len(d2_solutions) > 0 else orig_d2
        J[0, 0] = -(r + a) * sin(t1) + calculated_d2 * cos(t1 + beta)
        J[0, 1] = sin(t1 + beta)
        J[1, 0] = (r + a) * cos(t1) + calculated_d2 * sin(t1 + beta)
        J[1, 1] = -cos(t1 + beta)
        print("Determinant with calculated d2 = {}: {}".format(d2_solutions, np.linalg.det(J)))


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
