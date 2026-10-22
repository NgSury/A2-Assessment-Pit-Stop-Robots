"""Stäubli TX90 model, collision checking and standalone Swift joint GUI."""

from pathlib import Path
import numpy as np
import roboticstoolbox as rtb
import spatialgeometry as sg
import swift
from swift import Slider, Button
from spatialmath import SE3

 
Robot_dir = Path(__file__).resolve().parent
Visual_dir = Robot_dir / "assets" / "staubli" / "tx90" / "visual"
Collision_dir = Robot_dir / "assets" / "staubli" / "tx90" / "collision"

Staubli_grey = [0.75, 0.75, 0.75, 1.0]
Dark_grey = [0.2, 0.2, 0.2, 1.0]


def mesh(name, pose=SE3(), color=Staubli_grey):
    return [sg.Mesh(str(Visual_dir / f"{name}.stl"), pose=pose, color=color)]


def stl_bounds(path):
    data = Path(path).read_bytes()
    count = int.from_bytes(data[80:84], "little")
    dtype = np.dtype([
        ("normal", "<f4", 3),
        ("vertices", "<f4", (3, 3)),
        ("attribute", "<u2"),
    ])
    vertices = np.frombuffer(
        data, dtype=dtype, count=count, offset=84
    )["vertices"].reshape(-1, 3)
    return vertices.min(axis=0), vertices.max(axis=0)


def world_bounds(pose, bounds):
    low, high = bounds
    corners = np.array([
        [x, y, z, 1.0]
        for x in (low[0], high[0])
        for y in (low[1], high[1])
        for z in (low[2], high[2])
    ])
    points = (pose.A @ corners.T).T[:, :3]
    return points.min(axis=0), points.max(axis=0)


class Staubli:
    def __init__(self):
        links = [
            rtb.RevoluteDH(d=.478, a=.050, alpha=-np.pi/2,
                           qlim=np.deg2rad([-180, 180])),
            rtb.RevoluteDH(a=.425, offset=-np.pi/2,
                           qlim=np.deg2rad([-130, 147.5])),
            rtb.RevoluteDH(d=.050, alpha=np.pi/2, offset=np.pi/2,
                           qlim=np.deg2rad([-145, 145])),
            rtb.RevoluteDH(d=.425, alpha=-np.pi/2,
                           qlim=np.deg2rad([-270, 270])),
            rtb.RevoluteDH(alpha=np.pi/2,
                           qlim=np.deg2rad([-115, 140])),
            rtb.RevoluteDH(d=.100,
                           qlim=np.deg2rad([-270, 270])),
        ]

        self.Robot = rtb.DHRobot(links, name="Staubli TX90")

        self.mesh_offsets = [
            SE3.Tx(-.050) * SE3.Rx(np.pi/2),
            SE3.Tx(-.425) * SE3.Rz(np.pi/2) * SE3.Rx(np.pi/2),
            SE3(),
            SE3.Ty(.425) * SE3.Rx(np.pi/2),
            SE3(),
            SE3(),
        ]

        for i, link in enumerate(self.Robot.links):
            link.geometry = mesh(f"link_{i + 1}", self.mesh_offsets[i])

        self.base_mesh = mesh("base_link", self.Robot.base, Dark_grey)[0]
        self.qr = np.zeros(6)
        self.Robot.q = self.qr.copy()

        self.collision_bounds = [
            stl_bounds(Collision_dir / f"link_{i + 1}.stl")
            for i in range(6)
        ]

    def check_box_collision(self, obstacle_pose, obstacle_size, q=None, margin=.02):
        q = self.Robot.q if q is None else q
        half = np.asarray(obstacle_size) / 2
        obstacle_min, obstacle_max = world_bounds(obstacle_pose, (-half, half))
        obstacle_min -= margin
        obstacle_max += margin

        frames = self.Robot.fkine_all(q)
        if len(frames) == 7:
            frames = frames[1:]

        for i, frame in enumerate(frames):
            link_min, link_max = world_bounds(
                frame * self.mesh_offsets[i], self.collision_bounds[i]
            )
            if np.all(link_max >= obstacle_min) and np.all(obstacle_max >= link_min):
                return True, i + 1

        return False, None


def move_joint(robot, angle, joint):
    q = robot.Robot.q.copy()
    q[joint] = np.deg2rad(float(angle))
    robot.Robot.q = q


if __name__ == "__main__":
    robot = Staubli()
    env = swift.Swift()
    env.launch(realtime=True)
    env.add(robot.base_mesh)
    env.add(robot.Robot)

    sliders = []
    for i, link in enumerate(robot.Robot.links):
        lower, upper = np.rad2deg(link.qlim)
        slider = Slider(
            cb=lambda angle, j=i: move_joint(robot, angle, j),
            min=float(np.ceil(lower * 10) / 10),
            max=float(np.floor(upper * 10) / 10),
            step=.1,
            value=round(float(np.rad2deg(robot.Robot.q[i])), 1),
            desc=f"Joint {i + 1}",
            unit="deg",
        )
        sliders.append(slider)
        env.add(slider)

    reset = [False]
    env.add(Button(cb=lambda _: reset.__setitem__(0, True), desc="Reset"))

    try:
        while True:
            if reset[0]:
                reset[0] = False
                for q in rtb.jtraj(robot.Robot.q, robot.qr, 50).q:
                    robot.Robot.q = q
                    env.step(.03)
                for slider in sliders:
                    slider.value = 0
            env.step(.03)
    except KeyboardInterrupt:
        pass

