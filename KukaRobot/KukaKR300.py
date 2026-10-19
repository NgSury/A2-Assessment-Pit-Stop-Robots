"Kuka KR300"
"Purpose: Act as car jack."


import roboticstoolbox as rtb
import numpy as np
import swift
from swift import Slider
import spatialgeometry as sg
from spatialmath import SE3
from pathlib import Path

#Mesh files
MESH_DIR = Path(__file__).resolve().parent / "kr300_r2700_2" / "visual"
KUKA_ORANGE = [1.0, 0.36, 0.0, 1.0]
DARK_GREY = [0.2, 0.2, 0.2, 1.0]

Q_MESH = np.array([0, -np.pi/2, np.pi/2, 0, 0, 0])

def mesh(name, pose=SE3(), color=KUKA_ORANGE):
    path = MESH_DIR / (name + ".dae")
    if not path.exists():
        raise FileNotFoundError(f"Mesh not found: {path}")
    return [sg.Mesh(str(path), pose=pose, color=color)]

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

        # Attach each mesh by undoing its link frame at the export pose
        T_mesh = self.Robot.fkine_all(Q_MESH)
        for i, link in enumerate(self.Robot.links):
            color = DARK_GREY if i == 5 else KUKA_ORANGE
            link.geometry = mesh(f"link_{i + 1}", T_mesh[i + 1].inv(), color)

        self.base_mesh = mesh("base_link", self.Robot.base, DARK_GREY)[0]

        self.qr = np.array([0, -np.pi/2, np.pi/2, 0, np.pi/2, 0])
        self.Robot.q = self.qr

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
    for i in range(6):
        lower, upper = np.rad2deg(robot.Robot.links[i].qlim)
        env.add(Slider(
            cb=lambda angle, j=i: move_joint(robot, angle, j),
            min=float(np.ceil(lower * 10) / 10),   # round inwards so the
            max=float(np.floor(upper * 10) / 10),  # slider stays within qlim
            step=0.1,
            value=round(float(np.rad2deg(robot.Robot.q[i])), 1),
            desc=f"Joint {i + 1}",
            unit="deg"
        ))

    #slider input
    try:
        while True:
            env.step(0.05)
    except KeyboardInterrupt:
        pass