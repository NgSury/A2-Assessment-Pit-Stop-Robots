# abb irb140
# removes the wheel after the car has been elevated.
# this file can be imported by pitstop.py or run on its own.

import roboticstoolbox as rtb
import numpy as np
import swift
import spatialgeometry as sg
from spatialmath import SE3
from pathlib import Path
from swift import Slider, Button, Label


# 1. mesh file location

MESH_DIR = Path(__file__).resolve().parent / "visual"

ABB_ORANGE = [1.0, 0.43, 0.0, 1.0]
DARK_GREY = [0.2, 0.2, 0.2, 1.0]

Q_MESH = np.zeros(6)
MESH_SCALE = [0.01, 0.01, 0.01]

#load stl meshes
def mesh(name, pose=SE3(), color=ABB_ORANGE):
    path = MESH_DIR / (name + ".stl")

    if not path.exists():
        raise FileNotFoundError(f"Mesh not found: {path}")
    return [sg.Mesh(str(path), pose=pose, scale=MESH_SCALE, color=color)]

#create abb irb140 robot
class ABBIRB140:
    def __init__(self):
        # original dh link dimensions and joint limits.
        links = [
            rtb.RevoluteDH(d=0.352, a=0.070, alpha=-np.pi / 2, qlim=np.deg2rad([-180, 180])),
            rtb.RevoluteDH(d=0, a=0.360, alpha=0, offset=-np.pi / 2, qlim=np.deg2rad([-100, 100])),
            rtb.RevoluteDH(d=0, a=0, alpha=-np.pi / 2, qlim=np.deg2rad([-220, 60])),
            rtb.RevoluteDH(d=0.380, a=0, alpha=np.pi / 2, qlim=np.deg2rad([-200, 200])),
            rtb.RevoluteDH(d=0, a=0, alpha=-np.pi / 2, qlim=np.deg2rad([-120, 120])),
            rtb.RevoluteDH(d=0.065, a=0, alpha=0, qlim=np.deg2rad([-400, 400]))
        ]

        self.Robot = rtb.DHRobot(links, name="ABB IRB140")

        # attach meshes in the correct link coordinate frames.
        T_mesh = self.Robot.fkine_all(Q_MESH)

        for i, link in enumerate(self.Robot.links):
            if i == 5:
                color = DARK_GREY
            else:
                color = ABB_ORANGE

            link.geometry = mesh(
                f"link_{i + 1}",
                T_mesh[i + 1].inv(),
                color
            )

        # attach the 19mm impact socket directly to joint 6.
        attachment_path = Path(__file__).resolve().parent / "Facom_NS19LA_19mm_socket.stl"

        # joint 6's local z axis is the outward wrist axis.
        # the stl is drawn in mm with the nut end at z=0 and the drive end at z=78 mm,
        # so Rx(pi) at the tip puts the nut end at the tool tip and the drive end on the link.
        socket_length = 0.078
        attachment = sg.Mesh(
            str(attachment_path),
            pose=SE3(0, 0, socket_length) * SE3.Rx(np.pi),
            scale=[0.001, 0.001, 0.001],
            color=DARK_GREY
        )

        # include the socket as link 6 geometry so it follows every joint movement.
        self.Robot.links[5].geometry = list(self.Robot.links[5].geometry) + [attachment]
        # tool frame at the socket's nut end, z pointing out of the flange.
        self.Robot.tool = SE3(0, 0, socket_length)

        self.base_mesh = mesh(
            "base_link",
            self.Robot.base,
            ABB_ORANGE
        )[0]

        self.qr = np.zeros(6)
        self.Robot.q = self.qr.copy()

#standalone swift gui
def main():
    robot = ABBIRB140()

    print("ABB IRB140 robot model created.")
    print("End-effector pose:")
    print(robot.Robot.fkine(robot.Robot.q))

    env = swift.Swift()
    env.launch(realtime=True)
    env.add(robot.base_mesh)
    env.add(robot.Robot)
    env.step(0.05)

    # 5. gui sliders
    returning_home = False

    def move_joint(angle, joint):
        nonlocal returning_home
        returning_home = False

        q = robot.Robot.q.copy()
        q[joint] = np.deg2rad(float(angle))
        robot.Robot.q = q

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

    joint_functions = [move_joint1, move_joint2, move_joint3, move_joint4, move_joint5, move_joint6]

    sliders = []

    for i in range(6):
        lower, upper = np.rad2deg(robot.Robot.links[i].qlim)

        slider = Slider(
            cb=joint_functions[i],
            min=float(lower),
            max=float(upper),
            step=1,
            value=0,
            desc=f"Joint {i + 1}",
            unit="deg"
        )

        env.add(slider)
        sliders.append(slider)

    # 6. smooth return to home

    RETURN_SPEED = 15    # degrees per second
    STEP_TIME = 0.1    # seconds per step

    def return_home(value=None):
        nonlocal returning_home
        returning_home = True

    home_button = Button(
        cb=return_home,
        desc="Return to initial position"
    )
    env.add(home_button)

    # 7. camera and simulation
    env.set_camera_pose(
        [1.4, -1.8, 1.3],
        [0.15, 0, 0.45]
    )

    try:
        while True:
            if returning_home:
                q = robot.Robot.q.copy()
                movement = np.deg2rad(RETURN_SPEED * STEP_TIME)
                all_home = True

                for i in range(6):
                    if q[i] > movement:
                        q[i] = q[i] - movement
                        all_home = False
                    elif q[i] < -movement:
                        q[i] = q[i] + movement
                        all_home = False
                    else:
                        q[i] = 0

                robot.Robot.q = q
                if all_home:
                    returning_home = False
                    for slider in sliders:
                        slider.value = 0
            env.step(STEP_TIME)

    except KeyboardInterrupt:
        pass
    input("Press enter to close")

# 8. run the standalone gui only when directly executed
if __name__ == "__main__":
    main()