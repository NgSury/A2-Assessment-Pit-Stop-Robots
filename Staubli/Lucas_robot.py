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

MESH_DIR = (
    Path(__file__).resolve().parent
    / "assets"
    / "staubli"
    / "tx90"
    / "visual"
)

STAUBLI_GREY = [0.75, 0.75, 0.75, 1.0]
DARK_GREY = [0.2, 0.2, 0.2, 1.0]


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
                alpha=-np.pi/2,
                qlim=np.deg2rad([-180, 180])
            ),

            rtb.RevoluteDH(
                d=0.0,
                a=0.425,
                alpha=0.0,
                offset=-np.pi/2,
                qlim=np.deg2rad([-130, 147.5])
            ),

            rtb.RevoluteDH(
                d=0.050,
                a=0.0,
                alpha=np.pi/2,
                offset=np.pi/2,
                qlim=np.deg2rad([-145, 145])
            ),

            rtb.RevoluteDH(
                d=0.425,
                a=0.0,
                alpha=-np.pi/2,
                qlim=np.deg2rad([-270, 270])
            ),

            rtb.RevoluteDH(
                d=0.0,
                a=0.0,
                alpha=np.pi/2,
                qlim=np.deg2rad([-115, 140])
            ),

            rtb.RevoluteDH(
                d=0.100,
                a=0.0,
                alpha=0.0,
                qlim=np.deg2rad([-270, 270])
            ),
        ]

        self.Robot = rtb.DHRobot(
            links,
            name="Staubli TX90"
        )

        # Fixed transforms from each DH link frame to the matching
        # STL/URDF link frame. These preserve the DH kinematic model
        # while aligning the visual meshes correctly.
        mesh_offsets = [
            SE3.Tx(-0.050) * SE3.Rx(np.pi/2),
            SE3.Tx(-0.425) * SE3.Rz(np.pi/2) * SE3.Rx(np.pi/2),
            SE3(),
            SE3.Ty(0.425) * SE3.Rx(np.pi/2),
            SE3(),
            SE3(),
        ]

        for i, link in enumerate(self.Robot.links):
            link.geometry = mesh(
                f"link_{i + 1}",
                mesh_offsets[i]
            )

        self.base_mesh = mesh(
            "base_link",
            self.Robot.base,
            DARK_GREY
        )[0]

        self.qr = np.zeros(6)
        self.Robot.q = self.qr


if __name__ == "__main__":
    robot = Lucasbot()

    print("Staubli TX90 robot model created.")
    print("End-effector pose:")
    print(robot.Robot.fkine(np.zeros(6)))

    env = swift.Swift()
    env.launch(realtime=True)

    env.add(robot.base_mesh)
    env.add(robot.Robot)

    env.step(0.05)

    q_end = np.deg2rad([40, -25, 35, 30, -20, 45])

    trajectory = rtb.jtraj(
        robot.Robot.q.copy(),
        q_end,
        100
    )

    for q in trajectory.q:
        robot.Robot.q = q
        env.step(0.03)

    input("Press Enter to close")
