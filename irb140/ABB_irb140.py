
"""
ABB IRB140
Purpose: Remove the wheel after the car has been elevated.
"""

import roboticstoolbox as rtb
import numpy as np
import swift
import spatialgeometry as sg
from spatialmath import SE3
from pathlib import Path

MESH_DIR = Path(__file__).resolve().parent / "visual"

ABB_ORANGE = [1.0, 0.43, 0.0, 1.0]
DARK_GREY = [0.2, 0.2, 0.2, 1.0]

# Reference pose for aligning the meshes
Q_MESH = np.zeros(6)

# The downloaded STL meshes use centimetre-based coordinates
MESH_SCALE = [0.01, 0.01, 0.01]


def mesh(name, pose=SE3(), color=ABB_ORANGE):
    path = MESH_DIR / (name + ".stl")
    if not path.exists():
        raise FileNotFoundError(f"Mesh not found: {path}")

    return [sg.Mesh(str(path), pose=pose,
                    scale=MESH_SCALE, color=color)]


class ABBIRB140:
    def __init__(self):

        links = [
            rtb.RevoluteDH(
                d=0.352, a=0.070, alpha=-np.pi/2,
                qlim=np.deg2rad([-180, 180])
            ),

            rtb.RevoluteDH(
                d=0, a=0.360, alpha=0,
                qlim=np.deg2rad([-100, 100])
            ),

            rtb.RevoluteDH(
                d=0, a=0, alpha=-np.pi/2,
                qlim=np.deg2rad([-220, 60])
            ),

            rtb.RevoluteDH(
                d=0.380, a=0, alpha=np.pi/2,
                qlim=np.deg2rad([-200, 200])
            ),

            rtb.RevoluteDH(
                d=0, a=0, alpha=-np.pi/2,
                qlim=np.deg2rad([-120, 120])
            ),

            rtb.RevoluteDH(
                d=0.065, a=0, alpha=0,
                qlim=np.deg2rad([-400, 400])
            ),
        ]

        self.Robot = rtb.DHRobot(links, name='ABB IRB140')

        # Attach meshes using the reference joint pose
        T_mesh = self.Robot.fkine_all(Q_MESH)

        for i, link in enumerate(self.Robot.links):
            color = DARK_GREY if i == 5 else ABB_ORANGE

            link.geometry = mesh(
                f"link_{i + 1}",
                T_mesh[i + 1].inv(),
                color
            )

        self.base_mesh = mesh(
            "base_link", self.Robot.base, ABB_ORANGE
        )[0]

        # Initial configuration
        self.qr = np.zeros(6)
        self.Robot.q = self.qr


robot = ABBIRB140()
print("ABB IRB140 robot model created.")

q_zero = np.zeros(6)
pose = robot.Robot.fkine(q_zero)

print("End-effector pose:")
print(pose)

env = swift.Swift()
env.launch(realtime=True)

env.add(robot.base_mesh)
env.add(robot.Robot)

env.step(0.05)

input("Press enter to close")
