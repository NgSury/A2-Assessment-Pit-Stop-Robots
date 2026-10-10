"""
Stäubli TX90. robot model.
41013 Industrial Robotics - Lab Assignment 2
Purpose: Position and install the replacement tyre
"""
import roboticstoolbox as rtb
import numpy as np
import swift
import spatialgeometry as sg
from spatialmath import SE3
from pathlib import Path

class Lucasbot(rtb.DHRobot):
    def __init__(self):
        links = [
            rtb.RevoluteDH(d=0.478, a=0.050, alpha=-np.pi/2),
            rtb.RevoluteDH(d=0.0,   a=0.425, alpha=0.0,      offset=-np.pi/2),
            rtb.RevoluteDH(d=0.050, a=0.0,   alpha=np.pi/2,  offset=np.pi/2),
            rtb.RevoluteDH(d=0.425, a=0.0,   alpha=-np.pi/2),
            rtb.RevoluteDH(d=0.0,   a=0.0,   alpha=np.pi/2),
            rtb.RevoluteDH(d=0.100, a=0.0,   alpha=0.0),
        ]

        super().__init__(links, name="Staubli TX90")
        
robot = Lucasbot()

print(robot)

q_zero = np.zeros(6)

pose = robot.fkine(q_zero)

print(pose)

q_test = np.deg2rad([30, -20, 25, 10, 15, 5])

test_pose = robot.fkine(q_test)

print(test_pose)

env = swift.Swift()
env.launch(realtime=True)

mesh_path = Path(__file__).parent / "assets" / "staubli" / "tx90" / "visual" / "base_link.stl"

base_mesh = sg.Mesh(
    filename=str(mesh_path),
    pose=SE3()
)

env.add(base_mesh)
env.hold()
