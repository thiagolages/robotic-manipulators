from pathlib import Path

import numpy as np
from roboticstoolbox.robot.Robot import Robot


# Comau Definition using URDF
class ComauRobot(Robot):
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

        self.qr = np.array([0, 0, -np.pi / 2, 0, np.pi / 2, 0])  # Ready configuration
        self.qz = np.zeros(6)  # Zero configuration

        self.addconfiguration("qr", self.qr)
        self.addconfiguration("qz", self.qz)


# Comau Definition using DH parameters

# # numpy provides import array and linear algebra utilities
# import numpy as np
# from roboticstoolbox import DHRobot, DHLink
# # spatial math provides objects for representing transformations
# import spatialmath as sm
# from roboticstoolbox.robot.ET import ET
# from roboticstoolbox.robot.Link import Link
# from spatialmath import SE3

# class ComauSmartSix(DHRobot):
#     def __init__(self):
#         self.link1 = DHLink(d=-0.45 , a=0.15 , alpha=pi/2 , offset=0)
#         self.link2 = DHLink(d=0      , a=0.59 , alpha=pi   , offset=-pi/2)
#         self.link3 = DHLink(d=0      , a=0.13 , alpha=-pi/2, offset=pi/2)
#         self.link4 = DHLink(d=-0.6471, a=0    , alpha=-pi/2, offset=0)
#         self.link5 = DHLink(d=0      , a=0    , alpha=pi/2 , offset=0)
#         self.link6 = DHLink(d=-0.095 , a=0    , alpha=pi   , offset=pi)
#         self.L = [self.link1, self.link2, self.link3, self.link4, self.link5, self.link6]

#         super().__init__(
#             self.L,
#             name="Smart Six",
#             manufacturer="Comau",
#             keywords=("dynamics", "symbolic", "mesh"),
#             symbolic=True,
#             meshdir="smart5six_description/meshes/dae/",
#         )
#         rx = rtb.ET.Rx(np.pi).A() # 180 degrees rotation around x-axis
#         self.base = sm.SE3(rx)  # Set the base transformation

#         # Pre-defined configurations
#         self.qz = np.array([0, 0, 0, 0, 0, 0])  # Zero configuration
#         self.qhome = np.array([0, 0, -np.pi/2, 0, -np.pi/2, 0])  # Home configuration
