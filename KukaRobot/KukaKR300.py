"Kuka KR300"
"Purpose: Act as car jack."


import roboticstoolbox as rtb
import numpy as np
import swift
import spatialgeometry as sg
from spatialmath import SE3
from pathlib import Path

MESH_DIR = Path(__file__).resolve().parent / "kr300_r2700_2" / "visual"
KUKA_ORANGE = [1.0, 0.36, 0.0, 1.0]
DARK_GREY = [0.2, 0.2, 0.2, 1.0]

def mesh(name, pose=SE3(), color=KUKA_ORANGE):
    path = MESH_DIR / (name + ".stl")
    if not path.exists():
        raise FileNotFoundError(f"Mesh not found: {path}")
    return [sg.Mesh(str(path), pose=pose, color=color)]

class KukaKR300:
    def __init__(self):
        links = [
            rtb.RevoluteDH(d=.645, a=0.330, alpha=-np.pi/2, qlim=[-3.2289, 3.2289], geometry=mesh("link_1", SE3(-0.33, 0.645, 0) * SE3.Rx(-np.pi/2))),
            rtb.RevoluteDH(d=0, a=1.150, alpha=0, qlim=[-2.4435, -0.0873], geometry=mesh("link_2", SE3(-1.15, 0, 0))),
            rtb.RevoluteDH(d=0, a=0.115, alpha=np.pi/2, offset=-np.pi/2, qlim=[-2.0944, 2.9322], geometry=mesh("link_3", SE3(-0.115, 0, 0) * SE3.Rx(-np.pi/2))),
            rtb.RevoluteDH(d=-1.220, a=0, alpha=-np.pi/2, qlim=[-6.1087, 6.1087], geometry=mesh("link_4", SE3(0, -1.22, 0) * SE3.Rx(np.pi/2))),
            rtb.RevoluteDH(d=0, a=0, alpha=np.pi/2, qlim=[-2.1817, 2.1817], geometry=mesh("link_5", SE3.Rx(-np.pi/2))),
            rtb.RevoluteDH(d=-0.240, a=0, alpha=0, qlim=[-6.1087, 6.1087], geometry=mesh("link_6", SE3(0, 0, 0.240), DARK_GREY)),
        ]
        self.Robot = rtb.DHRobot(links, name='Kuka KR300')

        self.base_mesh = mesh("base_link", self.Robot.base, DARK_GREY)[0]

        self.qr = np.array([0, -np.pi/2, np.pi/2, 0, np.pi/2, 0])
        self.Robot.q = self.qr

robot = KukaKR300()
print("Kuka KR300 robot model created.")
q_zero = np.zeros(6)
pose = robot.Robot.fkine(q_zero)
env = swift.Swift()
env.launch(realtime=True)
env.add(robot.base_mesh)
env.add(robot.Robot)
env.step(0.05)

input("Press enter to close")