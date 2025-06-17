from pathlib import Path

# numpy provides import array and linear algebra utilities
import numpy as np
import roboticstoolbox as rtb

# spatial math provides objects for representing transformations
import spatialmath as sm
from roboticstoolbox import DHLink, DHRobot
from roboticstoolbox.robot.Robot import Robot


class ComauRobot:
    """
    Comau Robot class that provides a basic interface for the Comau robot.
    This class is a placeholder and should be extended with specific functionalities.
    """

    def __init__(self):
        self.name = "Comau Robot"
        self.manufacturer = "Comau"
        self.qr = np.array([0, 0, -np.pi / 2, 0, np.pi / 2, 0])  # Ready configuration
        self.qz = np.zeros(6)  # Zero configuration

    def __str__(self):
        return (
            f"{self.name} by {self.manufacturer}\n"
            f"Ready configuration (qr): {self.qr}\n"
            f"Zero configuration (qz): {self.qz}"
        )


# Comau Definition using URDF
class ComauRobotURDF(Robot, ComauRobot):
    def __init__(self):

        config_dir = Path(__file__).resolve().parent.parent
        links, name, urdf_string, urdf_filepath = self.URDF_read(
            "smart5six_description/robot/comau.urdf", tld=config_dir.as_posix()
        )

        super().__init__(
            links,
            name=name,
            manufacturer="Comau",
            urdf_string=urdf_string,
            urdf_filepath=urdf_filepath,
        )
        ComauRobot.__init__(self)

        self.addconfiguration("qr", self.qr)
        self.addconfiguration("qz", self.qz)

        print("ComauRobotURDF initialized with the following links:")
        for i, link in enumerate(self.links):
            print(f"Link {i + 1}: {link}")


# Comau Definition using DH parameters
class ComauRobotDH(DHRobot, ComauRobot):
    def __init__(self):
        self.link1 = DHLink(d=-0.450, a=0.15, alpha=np.pi / 2, offset=0)
        self.link2 = DHLink(d=0, a=0.59, alpha=np.pi, offset=-np.pi / 2)
        self.link3 = DHLink(d=0, a=0.13, alpha=-np.pi / 2, offset=np.pi / 2)
        self.link4 = DHLink(d=-0.6471, a=0, alpha=-np.pi / 2, offset=0)
        self.link5 = DHLink(d=0, a=0, alpha=np.pi / 2, offset=0)
        self.link6 = DHLink(d=-0.095, a=0, alpha=np.pi, offset=np.pi)
        self.L = [self.link1, self.link2, self.link3, self.link4, self.link5, self.link6]

        super().__init__(
            self.L,
            name="Smart Six",
            manufacturer="Comau",
            # keywords=("dynamics", "symbolic", "mesh"),
            # symbolic=True,
            # meshdir="smart5six_description/meshes/dae/",
        )
        ComauRobot.__init__(self)

        rx = rtb.ET.Rx(np.pi).A()  # 180 degrees rotation around x-axis
        self.base = sm.SE3(rx)  # Set the base transformation

        print("ComauRobotDH initialized with the following links:")
        for i, link in enumerate(self.L):
            print(
                f"Link {i + 1}: d={link.d:.4f},\t \
                a={link.a:.4f},\t \
                alpha={link.alpha:.4f},\t \
                offset={link.offset:.4f}"
            )
