"Kuka KR300"
"Purpose: Act as car jack."


import roboticstoolbox as rtb
import numpy as np
import swift
from swift import Slider, Button
import spatialgeometry as sg
from spatialmath import SE3
from pathlib import Path

#Mesh files
mech_loc = Path(__file__).resolve().parent / "kr300_r2700_2" / "visual"
coll_loc = Path(__file__).resolve().parent / "kr300_r2700_2" / "collision"
kuka_orange = [1.0, 0.36, 0.0, 1.0]
dark_grey = [0.2, 0.2, 0.2, 1.0]
lift_colour = [0.3, 0.3, 0.3, 1.0]

q_mesh = np.array([0, -np.pi/2, np.pi/2, 0, 0, 0])

# Front wing lifter end effector
lifter_stl = Path(__file__).resolve().parent / "front_wing_lifter_ASSEMBLED.stl"
lifter_scale = 0.001
lifter_flange_z = 0.450

# Lifter collision boxes in the lifter STL frame (mm): (x min, x max, y min, y max, z min, z max)
# simplified representation of the lifter for collision checking
lifter_boxes = [
    (-110, 110, -110, 110, -450, -424.5),  # mounting flange
    (-100, 0, -60, 60, -424.5, -390),      # gusset
    (-90, -20, -40, 40, -390, -130),       # post
    (-91, 6, -70, 70, -130, -50),          # bracket
    (-38, 0, -225, 225, -90, 90),          # paddle
]

#flipping the lifter to match the robot's flange orientation
flange = SE3.Rx(np.pi)

def mesh(name, pose=SE3(), color=kuka_orange):
    path = mech_loc / (name + ".dae")
    if not path.exists():
        raise FileNotFoundError(f"Mesh not found: {path}")
    return [sg.Mesh(str(path), pose=pose, color=color)]

def collision_mesh(name, pose=SE3()):
    path = coll_loc / (name + ".stl")
    if not path.exists():
        raise FileNotFoundError(f"Collision mesh not found: {path}")
    return [sg.Mesh(str(path), pose=pose)]

def lifter_collision(lifter_pose):
    # Turn the lifter boxes into cuboids placed relative to the last link.
    boxes = []
    for x0, x1, y0, y1, z0, z1 in lifter_boxes:
        size = np.array([x1 - x0, y1 - y0, z1 - z0]) * lifter_scale
        centre = np.array([x0 + x1, y0 + y1, z0 + z1]) / 2 * lifter_scale
        boxes.append(sg.Cuboid(size, pose=lifter_pose * SE3(*centre)))
    return boxes

class KukaKR300:
    def __init__(self):
        links = [
            rtb.RevoluteDH(d=.645, a=0.330, alpha=-np.pi/2, qlim=[-3.2289, 3.2289]),
            rtb.RevoluteDH(d=0, a=1.150, alpha=0, qlim=[-2.4435, -0.0873]),
            rtb.RevoluteDH(d=0, a=0.115, alpha=np.pi/2, offset=-np.pi/2, qlim=[-2.0944, 2.9322]),
            rtb.RevoluteDH(d=-1.220, a=0, alpha=-np.pi/2, qlim=[-6.1087, 6.1087]),
            rtb.RevoluteDH(d=0, a=0, alpha=np.pi/2, qlim=[-2.1817, 2.1817]),
            rtb.RevoluteDH(d=-0.240, a=0, alpha=0, qlim=[-6.1087, 6.1087]),
        ]
        self.Robot = rtb.DHRobot(links, name='Kuka KR300')

        T_mesh = self.Robot.fkine_all(q_mesh)
        for i, link in enumerate(self.Robot.links):
            color = dark_grey if i == 5 else kuka_orange
            link.geometry = mesh(f"link_{i + 1}", T_mesh[i + 1].inv(), color)
            # Collision STLs share the visual meshes' frame, so they use the same pose.
            link.collision = collision_mesh(f"link_{i + 1}", T_mesh[i + 1].inv())
        #Adding the base that the robot will be mounted on
        self.base_mesh = mesh("base_link", self.Robot.base, dark_grey)[0]
        # The base doesn't move with any link; move it with base_collision.T alongside base_mesh.T.
        self.base_collision = collision_mesh("base_link", self.Robot.base)[0]

        lifter_pose = flange * SE3(0, 0, lifter_flange_z)
        self.lifter = sg.Mesh(
            str(lifter_stl),
            pose=lifter_pose,
            scale=[lifter_scale] * 3,
            color=lift_colour
        )
        last_link = self.Robot.links[-1]
        last_link.geometry = list(last_link.geometry) + [self.lifter]
        last_link.collision = list(last_link.collision) + lifter_collision(lifter_pose)
        self.Robot.tool = lifter_pose

        self.qr = np.array([0, -np.pi/2, np.pi/2, 0, np.pi/2, 0])
        self.Robot.q = self.qr

    def collides(self, q, obstacles):
        # True if the robot (including the lifter) at joint angles q hits any obstacle.
        return any(self.Robot.iscollided(q, obstacle) for obstacle in obstacles)

    def path_collides(self, path, obstacles):
        # Index of the first colliding step in a list of joint angles, or None if the path is clear.
        for i, q in enumerate(path):
            if self.collides(q, obstacles):
                return i
        return None

def move_joint(robot, angle, joint):
    q = robot.Robot.q.copy()
    q[joint] = np.deg2rad(float(angle))  # Swift may send the value as a string
    robot.Robot.q = q


if __name__ == "__main__":
    robot = KukaKR300()
    print("Kuka KR300 robot model created.")
    q_zero = np.zeros(6)
    pose = robot.Robot.fkine(q_zero)
    env = swift.Swift()
    env.launch(realtime=True)
    env.add(robot.base_mesh)
    env.add(robot.Robot)
    env.step(0.05)

    # One slider per joint (0.1 deg resolution), starting at the robot's current pose
    sliders = []
    for i in range(6):
        lower, upper = np.rad2deg(robot.Robot.links[i].qlim)
        slider = Slider(
            cb=lambda angle, j=i: move_joint(robot, angle, j),
            min=float(np.ceil(lower * 10) / 10),   # round inwards so the
            max=float(np.floor(upper * 10) / 10),  # slider stays within qlim
            step=0.1,
            value=round(float(np.rad2deg(robot.Robot.q[i])), 1),
            desc=f"Joint {i + 1}",
            unit="deg"
        )
        sliders.append(slider)
        env.add(slider)

    reset_requested = [False]
    env.add(Button(cb=lambda _: reset_requested.__setitem__(0, True), desc="reset"))

    #slider input
    try:
        while True:
            if reset_requested[0]:
                reset_requested[0] = False
                for q in rtb.jtraj(robot.Robot.q, robot.qr, 50).q:
                    robot.Robot.q = q
                    env.step(0.05)
                for slider, q in zip(sliders, robot.qr):
                    slider.value = round(float(np.rad2deg(q)), 1)
            env.step(0.05)
    except KeyboardInterrupt:
        pass