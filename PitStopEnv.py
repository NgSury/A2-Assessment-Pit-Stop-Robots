"""
Pit Stop Environment
41013 Industrial Robotics - Lab Assignment 2
Purpose: Swift scene of a Mercedes W07 tyre change.
    Kuka KR300   - jacks up the front of the car
    ABB IRB140   - removes and refits the wheel nut
    Staubli TX90 - swaps the front right wheel for a new tyre
"""

import numpy as np
import roboticstoolbox as rtb
import swift
import spatialgeometry as sg
from spatialmath import SE3
from pathlib import Path

from KukaRobot.KukaKR300 import KukaKR300
from irb140.ABB_irb140 import ABBIRB140
from Staubli.Lucas_robot import Lucasbot

CAR_DIR = Path(__file__).resolve().parent / "Car"

# W07.STL and Wheel.STL share units; a 39 unit wheel is a 660 mm F1 tyre
MESH_TO_M = 0.66 / 39.0

# Landmarks measured on W07.STL (mesh units, Y up, nose along +X, Z across)
CAR_CENTRE = np.array([141.2, 0.0, 500.0])  # ground, midway between axles
FRONT_HUB_X = 239.9
REAR_HUB_X = 42.5
HUB_Y = 19.5
HUB_FACE_Z = {"front": 39.0, "rear": 38.0}  # hub face offset from centreline
JACK_POINT = np.array([285.0, 7.0, 500.0])  # underside of the nose

# Wheel.STL: axis along +Z, inner face at z=0, nut seat at z~22, outer face z=23
WHEEL_CENTRE_XY = 19.5
NUT_SEAT_Z = 22.0
WHEEL_OUTER_Z = 23.0
NUT_LENGTH = 0.06
NUT_RADIUS = 0.04

CAR_SILVER = [0.75, 0.77, 0.80, 1.0]
TYRE_BLACK = [0.08, 0.08, 0.08, 1.0]
NEW_TYRE = [0.35, 0.05, 0.05, 1.0]  # tinted so the fresh tyre stands out
STAND_GREY = [0.55, 0.55, 0.6, 1.0]
NUT_RED = [0.85, 0.1, 0.1, 1.0]
TOOL_GREY = [0.3, 0.3, 0.3, 1.0]
DARK_GREY = [0.2, 0.2, 0.2, 1.0]
FLOOR_GREY = [0.35, 0.35, 0.38, 1.0]

# Wheel being changed: front right (car -y side, where the ABB and Staubli stand)
WORK_WHEEL = "FR"
WHEELS = {"FL": ("front", 1), "FR": ("front", -1),
          "RL": ("rear", 1), "RR": ("rear", -1)}

# Front-end lift, pivoting about the rear axle (~70 mm at the front wheels)
LIFT_PITCH = np.deg2rad(1.2)

# Kuka stands off the front-left corner, not in front of the nose
KUKA_BASE_XY = (3.7, 1.9)

# Tools: tool frame at the tip, z pointing out of the flange
KUKA_TOOL_LENGTH = 0.45  # Kuka flange faces along -z6
ABB_TOOL_LENGTH = 0.18
LUCAS_TOOL_LENGTH = 0.10

# Tyre stands beside the Staubli (offsets from its base); wheels lie flat on top
STAND_HEIGHT = 0.25
STAND_RADIUS = 0.3
STAND_OFFSETS = {"old": (-0.55, -0.1), "new": (-0.1, -0.65)}

DT = 0.02
STEPS = 50


def mesh_point(p):
    """Convert a W07.STL point (mesh units) into the car frame (metres)."""
    d = (np.asarray(p, dtype=float) - CAR_CENTRE) * MESH_TO_M
    return np.array([d[0], -d[2], d[1]])


# Car frame: x forward, y left, z up; mesh Y-up is rotated to Z-up
CAR_MESH_POSE = SE3.Rx(np.pi/2) * SE3(*(-CAR_CENTRE * MESH_TO_M))
WHEEL_MESH_POSE = SE3(-WHEEL_CENTRE_XY * MESH_TO_M, -WHEEL_CENTRE_XY * MESH_TO_M, 0)
NUT_POSE = SE3(0, 0, NUT_SEAT_Z * MESH_TO_M + NUT_LENGTH / 2)
JACK_POSE = SE3(mesh_point(JACK_POINT)) * SE3.Ry(-np.pi/2)  # tool z into the nose
REAR_PIVOT = mesh_point([REAR_HUB_X, HUB_Y, CAR_CENTRE[2]])


def hub_pose(axle, side):
    """Hub face frame in the car frame, z pointing outboard. side: +1 left, -1 right."""
    x = FRONT_HUB_X if axle == "front" else REAR_HUB_X
    p = mesh_point([x, HUB_Y, CAR_CENTRE[2] - side * HUB_FACE_Z[axle]])
    return SE3(p) * SE3.Rx(-side * np.pi/2)


def back(d):
    """Offset that backs off d metres along the tool axis."""
    return SE3(0, 0, -d)


def facing(x, y, target):
    """Base pose at (x, y) with its x axis pointing at target (x, y)."""
    return SE3(x, y, 0) * SE3.Rz(np.arctan2(target[1] - y, target[0] - x))


def add_tool(robot, tool, shapes):
    """Set robot.tool and attach shapes, given in the tool frame, to the last link."""
    robot.tool = tool
    for s in shapes:
        s.T = (tool * SE3(s.T)).A
    robot.links[-1].geometry = list(robot.links[-1].geometry) + shapes


class PitStop:
    def __init__(self, env):
        self.env = env
        self.pitch = 0.0

        # --- Robots ---
        self.kuka = KukaKR300()
        self.abb = ABBIRB140()
        self.lucas = Lucasbot()
        kuka, abb, lucas = self.kuka.Robot, self.abb.Robot, self.lucas.Robot

        front_hub = hub_pose(*WHEELS[WORK_WHEEL]).t
        jack = JACK_POSE.t
        kuka.base = facing(*KUKA_BASE_XY, jack)
        abb.base = SE3(front_hub[0] + 0.35, -1.6, 0) * SE3.Rz(np.pi/2)
        lucas.base = SE3(front_hub[0] - 0.7, -1.75, 0) * SE3.Rz(np.pi/2)  # room to swing the wheel aside
        for robot in (self.kuka, self.abb, self.lucas):
            robot.base_mesh.T = robot.Robot.base.A

        # Jack fork, wheel gun and wheel gripper
        add_tool(kuka, SE3(0, 0, -KUKA_TOOL_LENGTH) * SE3.Rx(np.pi), [
            sg.Cuboid([0.12, 0.12, KUKA_TOOL_LENGTH],
                      pose=SE3(0, 0, -KUKA_TOOL_LENGTH / 2), color=TOOL_GREY),
            sg.Cuboid([0.03, 0.3, 0.2], pose=SE3(0, 0, -0.1), color=DARK_GREY)])
        add_tool(abb, SE3(0, 0, ABB_TOOL_LENGTH), [
            sg.Cylinder(0.045, ABB_TOOL_LENGTH - 0.04,
                        pose=SE3(0, 0, -(ABB_TOOL_LENGTH + 0.04) / 2), color=TOOL_GREY),
            sg.Cylinder(NUT_RADIUS + 0.008, 0.04, pose=SE3(0, 0, -0.02), color=DARK_GREY)])
        add_tool(lucas, SE3(0, 0, LUCAS_TOOL_LENGTH), [
            sg.Cylinder(0.035, LUCAS_TOOL_LENGTH,
                        pose=SE3(0, 0, -LUCAS_TOOL_LENGTH / 2), color=TOOL_GREY),
            sg.Cylinder(0.14, 0.02, pose=SE3(0, 0, -0.01), color=DARK_GREY)])

        self.kuka_home = self.kuka.qr
        self.abb_home = np.array([-np.pi/2, -np.pi/2, 0, 0, np.pi/2, 0])  # parked clear of the wheel
        self.lucas_home = np.array([0, 0, np.pi/2, 0, np.pi/2, 0])
        kuka.q = self.kuka_home
        abb.q = self.abb_home
        lucas.q = self.lucas_home

        # --- Car, wheels and wheel nut ---
        self.floor = sg.Cuboid([12, 7, 0.01], pose=SE3(0.5, 0, -0.006), color=FLOOR_GREY)
        self.car = sg.Mesh(str(CAR_DIR / "W07.STL"), scale=[MESH_TO_M] * 3, color=CAR_SILVER)
        self.wheels = {name: sg.Mesh(str(CAR_DIR / "Wheel.STL"), scale=[MESH_TO_M] * 3,
                                     color=TYRE_BLACK) for name in WHEELS}
        self.nut = sg.Cylinder(NUT_RADIUS, NUT_LENGTH, color=NUT_RED)
        self.new_tyre = sg.Mesh(str(CAR_DIR / "Wheel.STL"), scale=[MESH_TO_M] * 3, color=NEW_TYRE)

        # Stand tops: a wheel frame resting there has its inner face down, z up
        lx, ly = lucas.base.t[:2]
        self.stands = {name: SE3(lx + dx, ly + dy, STAND_HEIGHT)
                       for name, (dx, dy) in STAND_OFFSETS.items()}
        self.stand_shapes = [sg.Cylinder(STAND_RADIUS, STAND_HEIGHT,
                                         pose=T * SE3(0, 0, -STAND_HEIGHT / 2), color=STAND_GREY)
                             for T in self.stands.values()]

        # Each prop follows a parent pose: (parent function, offset)
        self.mounts = {"car": (self.car_pose, CAR_MESH_POSE)}
        for name, (axle, side) in WHEELS.items():
            self.mounts[name] = (lambda a=axle, s=side: self.car_pose() * hub_pose(a, s),
                                 WHEEL_MESH_POSE)
        self.mounts["nut"] = (lambda: self.hub_world(WORK_WHEEL), NUT_POSE)
        self.mounts["new_tyre"] = (lambda: self.stands["new"], WHEEL_MESH_POSE)
        self.props = {"car": self.car, "nut": self.nut, "new_tyre": self.new_tyre, **self.wheels}

        if env is not None:
            for shape in [self.floor, self.kuka.base_mesh, self.abb.base_mesh,
                          self.lucas.base_mesh, *self.stand_shapes, *self.props.values()]:
                env.add(shape)
            for robot in (kuka, abb, lucas):
                env.add(robot)
        self.render()

    # --- Scene state ---
    def car_pose(self):
        """Car frame in the world: pitched nose-up about the rear axle."""
        return SE3(REAR_PIVOT) * SE3.Ry(-self.pitch) * SE3(-REAR_PIVOT)

    def hub_world(self, wheel):
        return self.car_pose() * hub_pose(*WHEELS[wheel])

    def prop_pose(self, name):
        parent, offset = self.mounts[name]
        return parent() * offset

    def attach(self, name, parent):
        """Re-parent a prop without moving it."""
        self.mounts[name] = (parent, parent().inv() * self.prop_pose(name))

    def render(self):
        for name, shape in self.props.items():
            shape.T = self.prop_pose(name).A
        if self.env is not None:
            self.env.step(DT)

    # --- Motion ---
    @staticmethod
    def ee(robot):
        return lambda: robot.fkine(robot.q)

    def solve(self, robot, T, q0):
        sol = robot.ikine_LM(T, q0=q0, joint_limits=True, seed=0)
        if not sol.success:
            raise RuntimeError(f"{robot.name}: no IK solution for\n{T}")
        # Take the 2*pi equivalent closest to q0 (e.g. after spinning the wheel gun)
        q = q0 + (sol.q - q0 + np.pi) % (2 * np.pi) - np.pi
        lo, hi = robot.qlim
        return np.where((q >= lo) & (q <= hi), q, sol.q)

    def move_joint(self, robot, q_goal, steps=STEPS):
        for q in rtb.jtraj(robot.q, q_goal, steps).q:
            robot.q = q
            self.render()

    def move_to(self, robot, T, steps=STEPS):
        """Joint-space move to a pose (used for free-space moves)."""
        self.move_joint(robot, self.solve(robot, T, robot.q), steps)

    def move_line(self, robot, T, steps=STEPS):
        """Straight-line Cartesian move, solving IK at each step."""
        for Ti in rtb.ctraj(robot.fkine(robot.q), T, steps):
            robot.q = self.solve(robot, Ti, robot.q)
            self.render()

    def spin_tool(self, robot, angle, steps=STEPS):
        """Spin the last joint (the tool axis), e.g. a wheel gun turning a nut."""
        q = robot.q.copy()
        lo, hi = robot.qlim[:, -1]
        if not lo <= q[-1] + angle <= hi:
            angle = -angle
        for qi in rtb.jtraj(q, q + np.r_[np.zeros(5), angle], steps).q:
            robot.q = qi
            self.render()

    # --- Pit stop ---
    def jack_target(self):
        return self.car_pose() * JACK_POSE

    def set_lift(self, pitch, steps=STEPS):
        """Kuka holds the jack point while the car pitches with it."""
        kuka = self.kuka.Robot
        for p in np.linspace(self.pitch, pitch, steps):
            self.pitch = p
            kuka.q = self.solve(kuka, self.jack_target(), kuka.q)
            self.render()

    def nut_target(self):
        """Wheel gun tip on the nut face, pointing along the axle."""
        return self.hub_world(WORK_WHEEL) * NUT_POSE * SE3(0, 0, NUT_LENGTH / 2) * SE3.Rx(np.pi)

    def wheel_target(self):
        """Gripper plate on the wheel's outer face, pointing along the axle."""
        return self.hub_world(WORK_WHEEL) * SE3(0, 0, WHEEL_OUTER_Z * MESH_TO_M) * SE3.Rx(np.pi)

    def stand_target(self, stand):
        """Gripper plate on the outer face of a wheel lying on a stand."""
        return self.stands[stand] * SE3(0, 0, WHEEL_OUTER_Z * MESH_TO_M) * SE3.Rx(np.pi)

    def place_on_stand(self, name, stand):
        """Staubli lowers the wheel it holds onto a stand and lets go."""
        lucas = self.lucas.Robot
        self.move_to(lucas, self.stand_target(stand) * back(0.2))
        self.move_line(lucas, self.stand_target(stand), 30)
        self.attach(name, lambda: self.stands[stand])
        self.move_line(lucas, self.stand_target(stand) * back(0.2), 30)

    def pick_from_stand(self, name, stand):
        """Staubli grips a wheel lying on a stand and lifts it clear."""
        lucas = self.lucas.Robot
        self.move_to(lucas, self.stand_target(stand) * back(0.2))
        self.move_line(lucas, self.stand_target(stand), 30)
        self.attach(name, self.ee(lucas))
        self.move_line(lucas, self.stand_target(stand) * back(0.2), 30)

    def run(self):
        kuka, abb, lucas = self.kuka.Robot, self.abb.Robot, self.lucas.Robot
        hub = lambda: self.hub_world(WORK_WHEEL)

        print("Kuka KR300: jacking up the front of the car")
        self.move_to(kuka, self.jack_target() * back(0.3))
        self.move_line(kuka, self.jack_target(), 30)
        self.set_lift(LIFT_PITCH)

        print("ABB IRB140: removing the wheel nut")
        self.move_to(abb, self.nut_target() * back(0.15))
        self.move_line(abb, self.nut_target(), 30)
        self.attach("nut", self.ee(abb))
        self.spin_tool(abb, -2 * np.pi)
        self.move_line(abb, self.nut_target() * back(0.2), 30)
        self.move_joint(abb, self.abb_home)

        print("Staubli TX90: removing the old wheel")
        self.move_to(lucas, self.wheel_target() * back(0.15))
        self.move_line(lucas, self.wheel_target(), 30)
        self.attach(WORK_WHEEL, self.ee(lucas))
        self.move_line(lucas, self.wheel_target() * back(0.3))
        self.place_on_stand(WORK_WHEEL, "old")

        print("Staubli TX90: fitting the new tyre")
        self.pick_from_stand("new_tyre", "new")
        self.move_to(lucas, self.wheel_target() * back(0.3))
        self.move_line(lucas, self.wheel_target())
        self.attach("new_tyre", hub)
        self.move_line(lucas, self.wheel_target() * back(0.15), 30)
        self.move_joint(lucas, self.lucas_home)

        print("ABB IRB140: refitting the wheel nut")
        self.move_to(abb, self.nut_target() * back(0.2))
        self.move_line(abb, self.nut_target(), 30)
        self.spin_tool(abb, 2 * np.pi)
        self.attach("nut", hub)
        self.move_line(abb, self.nut_target() * back(0.15), 30)
        self.move_joint(abb, self.abb_home)

        print("Kuka KR300: lowering the car")
        self.set_lift(0.0)
        self.move_line(kuka, self.jack_target() * back(0.3), 30)
        self.move_joint(kuka, self.kuka_home)
        print("Pit stop complete")


if __name__ == "__main__":
    env = swift.Swift()
    env.launch(realtime=True)
    pit_stop = PitStop(env)
    input("Press enter to start the pit stop")
    pit_stop.run()
    env.hold()
