"""
Stäubli TX90 robot model.
41013 Industrial Robotics - Lab Assignment 2
Purpose: Position and install the replacement tyre
"""

import roboticstoolbox as rtb
import numpy as np
import swift
import spatialgeometry as sg

from spatialmath import SE3
from pathlib import Path


# SECTION 1 — ROBOT MODEL

class Lucasbot(rtb.DHRobot):
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

        super().__init__(
            links,
            name="Staubli TX90"
        )


# SECTION 2 — CREATE ROBOT

robot = Lucasbot()

print(robot)


# SECTION 3 — FORWARD KINEMATICS TESTS

q_zero = np.zeros(6)

pose_zero = robot.fkine(q_zero)

print("\nZero pose:")
print(pose_zero)


q_test = np.deg2rad([
    30,
    -20,
    25,
    10,
    15,
    5
])

pose_test = robot.fkine(q_test)

print("\nTest pose:")
print(pose_test)


# SECTION 4 — START SWIFT

env = swift.Swift()

env.launch(
    realtime=True
)


# SECTION 5 — LOCATE STL FILES

project_folder = Path(__file__).parent

mesh_folder = (
    project_folder
    / "assets"
    / "staubli"
    / "tx90"
    / "visual"
)


base_path = mesh_folder / "base_link.stl"

link1_path = mesh_folder / "link_1.stl"
link2_path = mesh_folder / "link_2.stl"
link3_path = mesh_folder / "link_3.stl"
link4_path = mesh_folder / "link_4.stl"
link5_path = mesh_folder / "link_5.stl"
link6_path = mesh_folder / "link_6.stl"


# SECTION 6 — CHECK STL FILES

mesh_paths = [
    base_path,
    link1_path,
    link2_path,
    link3_path,
    link4_path,
    link5_path,
    link6_path
]


for mesh_path in mesh_paths:

    if not mesh_path.exists():

        raise FileNotFoundError(
            f"Could not find: {mesh_path}"
        )


# SECTION 7 — CREATE MESH OBJECTS

base_mesh = sg.Mesh(
    filename=str(base_path),
    pose=SE3()
)

link1_mesh = sg.Mesh(
    filename=str(link1_path),
    pose=SE3()
)

link2_mesh = sg.Mesh(
    filename=str(link2_path),
    pose=SE3()
)

link3_mesh = sg.Mesh(
    filename=str(link3_path),
    pose=SE3()
)

link4_mesh = sg.Mesh(
    filename=str(link4_path),
    pose=SE3()
)

link5_mesh = sg.Mesh(
    filename=str(link5_path),
    pose=SE3()
)

link6_mesh = sg.Mesh(
    filename=str(link6_path),
    pose=SE3()
)


# SECTION 8 — ADD MESHES TO SWIFT

env.add(base_mesh)

env.add(link1_mesh)
env.add(link2_mesh)
env.add(link3_mesh)
env.add(link4_mesh)
env.add(link5_mesh)
env.add(link6_mesh)


# SECTION 9 — UPDATE ROBOT MESH POSITION

def update_meshes(q):

    q1 = q[0]
    q2 = q[1]
    q3 = q[2]
    q4 = q[3]
    q5 = q[4]
    q6 = q[5]


    # Base stays fixed

    T0 = SE3()


    # Joint 1

    T1 = (
        T0
        * SE3.Tz(0.478)
        * SE3.Rz(q1)
    )


    # Joint 2

    T2 = (
        T1
        * SE3.Tx(0.050)
        * SE3.Ry(q2)
    )


    # Joint 3

    T3 = (
        T2
        * SE3.Tz(0.425)
        * SE3.Ty(0.050)
        * SE3.Ry(q3)
    )


    # Joint 4

    T4 = (
        T3
        * SE3.Rz(q4)
    )


    # Joint 5

    T5 = (
        T4
        * SE3.Tz(0.425)
        * SE3.Ry(q5)
    )


    # Joint 6

    T6 = (
        T5
        * SE3.Tz(0.100)
        * SE3.Rz(q6)
    )


    base_mesh.T = T0

    link1_mesh.T = T1
    link2_mesh.T = T2
    link3_mesh.T = T3
    link4_mesh.T = T4
    link5_mesh.T = T5
    link6_mesh.T = T6


# SECTION 10 — HOME POSITION

q_home = np.zeros(6)

update_meshes(
    q_home
)

env.step(
    0.05
)


# SECTION 11 — DEMO POSITION

q_demo = np.deg2rad([
    40,
    -25,
    35,
    30,
    -20,
    45
])


# SECTION 12 — MOVE TO DEMO POSITION

print(
    "\nStarting TX90 animation..."
)

steps = 100


for i in range(
    steps + 1
):

    s = i / steps

    blend = (
        3 * s**2
        -
        2 * s**3
    )

    q = (
        q_home
        +
        blend
        *
        (
            q_demo
            -
            q_home
        )
    )

    update_meshes(
        q
    )

    env.step(
        0.03
    )


# SECTION 13 — RETURN HOME

for i in range(
    steps + 1
):

    s = i / steps

    blend = (
        3 * s**2
        -
        2 * s**3
    )

    q = (
        q_demo
        +
        blend
        *
        (
            q_home
            -
            q_demo
        )
    )

    update_meshes(
        q
    )

    env.step(
        0.03
    )


print(
    "Animation complete."
)


# SECTION 14 — KEEP SWIFT OPEN

env.hold()