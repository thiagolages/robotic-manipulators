import sys

from CoppeliaSimAPI import CoppeliaSimAPI


class CoppeliaSimRobot:
    def __init__(self, api_object: CoppeliaSimAPI):
        """
        Initialize the CoppeliaSimRobot with the given API client.

        :param api_client: An instance of CoppeliaSimAPI to interact with the simulation.
        """
        self.smartsix_handle = "/Smartsix"
        self.api_client = api_object.client
        self.sim = api_object.sim
        self.num_joints = 6  # Number of joints for the Smartsix robot
        self.joint_handles = []

        # Ensure the robot is set up
        self.setupRobot()

    def setupRobot(self):
        """
        Setup the Smartsix robot in the simulation.
        """
        self.get_robot()
        self.get_joint_handles()
        print("Robot setup complete.")

    def get_robot(self):
        print("Getting {} handle...".format(self.smartsix_handle))
        self.robot_handle = self.sim.getObject(self.smartsix_handle)

        if self.robot_handle == -1:
            print("Failed to get Smartsix handle.")
            self.sim.stopSimulation()
            sys.exit("Exiting due to invalid Smartsix handle.")

        print("Got Smartsix Robot !")

    def get_joint_handles(self):
        # Assumes joint names are /Smartsix_joint1 ... /Smartsix_joint6
        self.joint_handles = []
        for i in range(self.num_joints):
            joint_name = "/junta{}".format(i + 1)
            print("Getting handle for joint:", joint_name)
            handle = self.sim.getObject(joint_name)
            if handle == -1:
                raise RuntimeError(f"Could not get handle for joint: {joint_name}")
            self.joint_handles.append(handle)
        print("Got all joint handles.")

    def setJointTargetPosition(self, q, verbose=False):
        """
        Set the target position for each joint of the Smartsix robot.

        :param q: A list or numpy array of joint angles in radians.
        :param verbose: If True, print the joint target positions.
        """
        if len(q) != self.num_joints:
            raise ValueError(f"Expected {self.num_joints} joint angles, got {len(q)}")

        for i in range(self.num_joints):
            self.sim.setJointTargetPosition(self.joint_handles[i], q[i])
        if verbose:
            print("Set joint target positions:", [round(angle, 4) for angle in q])
