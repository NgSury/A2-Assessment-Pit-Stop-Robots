"""
Stäubli TX90
Purpose: Position and install the replacement tyre.
"""

import roboticstoolbox as rtb
import numpy as np
import swift
import spatialgeometry as sg

from spatialmath import SE3
from pathlib import Path


MESH_DIR = Path(__file__).resolve().parent / "assets" / "staubli" / "tx90" / "visual"

STAUBLI_GREY = [0.75, 0.75, 0.75, 1.0]
DARK_GREY = [0.2, 0.2, 0.2, 1.0]

# Pose the STL files were exported in
Q_MESH = np.zeros(6)


def mesh(name, pose=SE3(), color=STAUBLI_GREY):
    path = MESH_DIR / f"{name}.stl"

    if not path.exists():
        raise FileNotFoundError(f"Mesh not found: {path}")

    return [sg.Mesh(str(path), pose=pose, color=color)]


class Lucasbot:
    def __init__(self):

        links = [
            rtb.RevoluteDH(
                d=0.478,
                a=0.050,
                alpha=-np.pi / 2
            ),

            rtb.RevoluteDH(
                d=0.0,
                a=0.425,
                alpha=0.0,
                offset=-np.pi / 2
            ),

            rtb.RevoluteDH(
                d=0.050,
                a=0.0,
                alpha=np.pi / 2,
                offset=np.pi / 2
            ),

            rtb.RevoluteDH(
                d=0.425,
                a=0.0,
                alpha=-np.pi / 2
            ),

            rtb.RevoluteDH(
                d=0.0,
                a=0.0,
                alpha=np.pi / 2
            ),

            rtb.RevoluteDH(
                d=0.100,
                a=0.0,
                alpha=0.0
            ),
        ]

        self.Robot = rtb.DHRobot(
            links,
            name="Staubli TX90"
        )

        # Get every DH link frame at the STL export pose
        T_mesh = self.Robot.fkine_all(Q_MESH)

        # Attach each STL to its matching DH link
        for i, link in enumerate(self.Robot.links):

            link.geometry = mesh(
                f"link_{i + 1}",
                T_mesh[i + 1].inv()
            )

        # Base is fixed, so it is added separately
        self.base_mesh = mesh(
            "base_link",
            self.Robot.base,
            DARK_GREY
        )[0]

        # Robot starting position
        self.qr = np.zeros(6)
        self.Robot.q = self.qr


robot = Lucasbot()

print("Staubli TX90 robot model created.")


# Forward kinematics test

q_zero = np.zeros(6)

pose = robot.Robot.fkine(q_zero)

print("\nEnd-effector pose:")
print(pose)


# Start Swift

env = swift.Swift()

env.launch(realtime=True)

env.add(robot.base_mesh)
env.add(robot.Robot)

env.step(0.05)


# Simple movement demonstration

q_start = robot.Robot.q.copy()

q_end = np.deg2rad([
    40,
    -25,
    35,
    30,
    -20,
    45
])

trajectory = rtb.jtraj(
    q_start,
    q_end,
    100
)

for q in trajectory.q:

    robot.Robot.q = q

    env.step(0.03)


input("Press Enter to close")
