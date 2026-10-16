# ABB IRB140
# Purpose: Remove the wheel after the car has been elevated.

import roboticstoolbox as rtb
import numpy as np
import swift
import spatialgeometry as sg
from spatialmath import SE3
from pathlib import Path
from swift import Slider, Button, Label

#mesh file
MESH_DIR = Path(__file__).resolve().parent / "visual"
ABB_ORANGE = [1.0, 0.43, 0.0, 1.0]
DARK_GREY = [0.2, 0.2, 0.2, 1.0]

#Reference pose to align meshes
Q_MESH = np.zeros(6)

#meshes use cm coords
MESH_SCALE = [0.01, 0.01, 0.01]

#load meshes
def mesh(name, pose=SE3(), color=ABB_ORANGE):
    path = MESH_DIR / (name + ".stl")
    if not path.exists():
        raise FileNotFoundError(f"Mesh not found: {path}")
    return [sg.Mesh(str(path), pose=pose, scale=MESH_SCALE, color=color)]

#Create robot
class ABBIRB140:
    def __init__(self):
        # DH parameters 
        #j2: -90 degree offset for mesh alignment.

        links = [
            rtb.RevoluteDH(d=0.352, a=0.070, alpha=-np.pi/2, qlim=np.deg2rad([-180, 180])),
            rtb.RevoluteDH(d=0, a=0.360, alpha=0, offset=-np.pi/2, qlim=np.deg2rad([-100, 100])),
            rtb.RevoluteDH(d=0, a=0, alpha=-np.pi/2, qlim=np.deg2rad([-220, 60])),
            rtb.RevoluteDH(d=0.380, a=0, alpha=np.pi/2, qlim=np.deg2rad([-200, 200])),
            rtb.RevoluteDH(d=0, a=0, alpha=-np.pi/2, qlim=np.deg2rad([-120, 120])),
            rtb.RevoluteDH(d=0.065, a=0, alpha=0, qlim=np.deg2rad([-400, 400]))
            ]

        self.Robot = rtb.DHRobot(links, name='ABB IRB140')

        # link positions at reference pose
        T_mesh = self.Robot.fkine_all(Q_MESH)

        for i, link in enumerate(self.Robot.links):
            if i == 5:
                color = DARK_GREY
            else:
                color = ABB_ORANGE

            # Attach mesh to links
            link.geometry = mesh(f"link_{i + 1}", T_mesh[i + 1].inv(), color)

        # base
        self.base_mesh = mesh("base_link", self.Robot.base, ABB_ORANGE)[0]

        #initial configuration
        self.qr = np.zeros(6)
        self.Robot.q = self.qr.copy()

#robot model
robot = ABBIRB140()
q_zero = np.zeros(6)
pose = robot.Robot.fkine(q_zero)

print("End-effector pose:")
print(pose)

#env
env = swift.Swift()
env.launch(realtime=True)
env.add(robot.base_mesh)
env.add(robot.Robot)
env.step(0.05)

#GUI 
#move selected robot joint
def move_joint(angle, joint):

    # get joint positions
    q = robot.Robot.q.copy()

    # slider degrees to radians
    angle_radians = np.deg2rad(float(angle))

    #Update joint
    q[joint] = angle_radians

    #Move robot
    robot.Robot.q = q

# Individual functions for each joint
def move_joint1(angle):
    move_joint(angle, 0)

def move_joint2(angle):
    move_joint(angle, 1)

def move_joint3(angle):
    move_joint(angle, 2)

def move_joint4(angle):
    move_joint(angle, 3)

def move_joint5(angle):
    move_joint(angle, 4)

def move_joint6(angle):
    move_joint(angle, 5)

#gui sliders

joint_functions = [move_joint1, move_joint2, move_joint3, move_joint4, move_joint5, move_joint6]

#store sliders for reset later
sliders = []

#make slider for each robot joint
for i in range(6):

    #joint limits in deg
    lower, upper = np.rad2deg(robot.Robot.links[i].qlim)

    #Create slider
    slider = Slider(cb=joint_functions[i], min=float(lower), max=float(upper), step=1, value=0, desc=f"Joint {i + 1}", unit="deg")

    env.add(slider)
    sliders.append(slider)

#return to initial pos
def return_home(value=None):

    #set joint angles to 0
    robot.Robot.q = np.zeros(6)

    #set slider values to 0
    for slider in sliders:
        slider.value = 0

#home button
home_button = Button(cb=return_home, desc="Return to initial position")

env.add(home_button)

# Camera pos
env.set_camera_pose([1.4, -1.8, 1.3], [0.15, 0, 0.45])

try:
    while True:
        env.step(0.05)
        
except KeyboardInterrupt:
    pass

input("Press enter to close")