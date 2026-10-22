# Pit Stop Environment
# 41013 Industrial Robotics - Lab Assignment 2
# Purpose: Swift simulation of a Mercedes W07 tyre change.
# Kuka KR300:  Lifts the front of the car.
# ABB IRB140:  Removes and refits the wheel nut.
# Staubli TX90: Removes the old front-right wheel and fits a new one.

import numpy as np
import roboticstoolbox as rtb
import swift
import spatialgeometry as sg
from spatialmath import SE3
from KukaRobot.KukaKR300 import KukaKR300
from irb140.ABB_irb140 import ABBIRB140
from Staubli.Lucas_robot import Lucasbot
from Car.CarModel import CarModel

# Simulation speed.
dt = 0.02
steps = 50

# Robot placement.
Kuka_base_posXY = (3.7, 1.9)

# Lengths of the tools attached to the robots.
ARB_toolLength = 0.18
Lucas_toolLength = 0.10

# Wheel storage stands.
standHeight = 0.25
standRadius = 0.3
standOffset = {
    "old": (-0.55, -0.1),
    "new": (-0.1, -0.65)
}

# Colours.
tyre_stand_grey = [0.55, 0.55, 0.6, 1.0]
tool_grey = [0.3, 0.3, 0.3, 1.0]
dark_grey = [0.2, 0.2, 0.2, 1.0]
floor_grey = [0.35, 0.35, 0.38, 1.0]


# 2. SIMPLE POSITION FUNCTIONS

def back(distance):
    # Move backwards along the robot tool's axis.
    return SE3(0, 0, -distance)


def facing(x, y, target):
    # Rotate a robot base so it faces a target point.
    angle = np.arctan2(target[1] - y, target[0] - x)
    return SE3(x, y, 0) * SE3.Rz(angle)


def add_tool(robot, tool_pose, shapes):
    # Set the tool position and attach its visual parts.
    robot.tool = tool_pose

    for shape in shapes:
        shape.T = (tool_pose * SE3(shape.T)).A

    robot.links[-1].geometry = list(robot.links[-1].geometry) + shapes


# 3. CREATE THE PIT STOP

class PitStop:

    def __init__(self, env):
        self.env = env

        # Create the car and the three robots.
        self.car = CarModel()
        self.kuka = KukaKR300()
        self.abb = ABBIRB140()
        self.lucas = Lucasbot()

        car = self.car
        kuka = self.kuka.Robot
        abb = self.abb.Robot
        lucas = self.lucas.Robot

        # Position the robots around the car.
        front_hub = car.hub_pose("front", -1).t
        jack = car.jack_offset.t

        kuka.base = facing(Kuka_base_posXY[0], Kuka_base_posXY[1], jack)
        abb.base = SE3(front_hub[0] + 0.35, -1.6, 0) * SE3.Rz(np.pi / 2)
        lucas.base = SE3(front_hub[0] - 0.7, -1.75, 0) * SE3.Rz(np.pi / 2)

        # Move the base meshes into the same positions.
        self.kuka.base_mesh.T = kuka.base.A
        self.abb.base_mesh.T = abb.base.A
        self.lucas.base_mesh.T = lucas.base.A


        # Add a wheel-nut tool to the ABB.
        abb_tool = SE3(0, 0, ARB_toolLength)
        abb_shapes = [
            sg.Cylinder(
                0.045, ARB_toolLength - 0.04,
                pose=SE3(0, 0, -(ARB_toolLength + 0.04) / 2),
                color=tool_grey
            ),
            sg.Cylinder(
                car.nut_radius + 0.008, 0.04,
                pose=SE3(0, 0, -0.02),
                color=dark_grey
            )
        ]
        add_tool(abb, abb_tool, abb_shapes)

        # Add a wheel gripper to the Staubli.
        lucas_tool = SE3(0, 0, Lucas_toolLength)
        lucas_shapes = [
            sg.Cylinder(
                0.035, Lucas_toolLength,
                pose=SE3(0, 0, -Lucas_toolLength / 2),
                color=tool_grey
            ),
            sg.Cylinder(
                0.14, 0.02,
                pose=SE3(0, 0, -0.01),
                color=dark_grey
            )
        ]
        add_tool(lucas, lucas_tool, lucas_shapes)

        # Starting positions of the robots.
        self.kuka_home = self.kuka.qr.copy()
        self.abb_home = np.array([-np.pi / 2, -np.pi / 2, 0, 0, np.pi / 2, 0])
        self.lucas_home = np.array([0, 0, np.pi / 2, 0, np.pi / 2, 0])

        kuka.q = self.kuka_home.copy()
        abb.q = self.abb_home.copy()
        lucas.q = self.lucas_home.copy()

        # Create the floor.
        self.floor = sg.Cuboid(
            [12, 7, 0.01],
            pose=SE3(0.5, 0, -0.006),
            color=floor_grey
        )

        # Create stands next to the Staubli.
        lx = lucas.base.t[0]
        ly = lucas.base.t[1]
        self.stands = {}
        self.stand_shapes = []

        for name in standOffset:
            dx, dy = standOffset[name]
            stand_pose = SE3(lx + dx, ly + dy, standHeight)
            self.stands[name] = stand_pose

            stand_shape = sg.Cylinder(
                standRadius,
                standHeight,
                pose=stand_pose * SE3(0, 0, -standHeight / 2),
                color=tyre_stand_grey
            )
            self.stand_shapes.append(stand_shape)

        # Store each object's parent and its position relative to that parent.
        self.mounts = {}
        self.mounts["car"] = ("car", car.body_offset)

        for name in car.wheels:
            self.mounts[name] = ("hub_" + name, car.wheel_offset)

        self.mounts["nut"] = ("hub_" + car.work_wheel, car.nut_offset)
        self.mounts["new_wheel"] = ("stand_new", car.wheel_offset)

        self.props = {
            "car": car.body,
            "nut": car.nut,
            "new_wheel": car.new_wheel
        }
        self.props.update(car.wheel_meshes)

        # Place every object before adding it, so nothing appears at the origin.
        self.update_props()

        # Add all objects and robots to Swift.
        if self.env is not None:
            self.env.add(self.floor)
            self.env.add(self.kuka.base_mesh)
            self.env.add(self.abb.base_mesh)
            self.env.add(self.lucas.base_mesh)

            for shape in self.stand_shapes:
                self.env.add(shape)

            for shape in self.props.values():
                self.env.add(shape)

            self.env.add(kuka)
            self.env.add(abb)
            self.env.add(lucas)

        self.render()

    # 4. OBJECT POSITIONS AND ATTACHMENT

    def parent_pose(self, parent):
        # Find the position of whichever object a part is attached to.
        if parent == "car":
            return self.car.pose()

        if parent == "stand_old":
            return self.stands["old"]

        if parent == "stand_new":
            return self.stands["new"]

        if parent == "kuka":
            return self.kuka.Robot.fkine(self.kuka.Robot.q)

        if parent == "abb":
            return self.abb.Robot.fkine(self.abb.Robot.q)

        if parent == "lucas":
            return self.lucas.Robot.fkine(self.lucas.Robot.q)

        # For example, "hub_FR" refers to the front-right wheel hub.
        if parent.startswith("hub_"):
            wheel = parent[4:]
            return self.car.hub_world(wheel)

        raise ValueError("Unknown attachment: " + parent)

    def prop_pose(self, name):
        # Find the current world position of an object.
        parent, offset = self.mounts[name]
        return self.parent_pose(parent) * offset

    def attach(self, name, new_parent):
        # Attach an object to a new parent without making it jump.
        old_position = self.prop_pose(name)
        new_position = self.parent_pose(new_parent)
        offset = new_position.inv() * old_position
        self.mounts[name] = (new_parent, offset)

    def update_props(self):
        # Move all objects to match the latest robot and car positions.
        for name in self.props:
            self.props[name].T = self.prop_pose(name).A

    def render(self):
        self.update_props()

        if self.env is not None:
            self.env.step(dt)

    # 5. ROBOT MOVEMENT

    def solve(self, robot, target, starting_q):
        # Find joint angles that position the tool at a target.
        solution = robot.ikine_LM(
            target,
            q0=starting_q,
            joint_limits=True,
            seed=0
        )

        if not solution.success:
            raise RuntimeError(robot.name + ": no IK solution for\n" + str(target))

        # Choose the nearest equivalent joint angles where limits allow.
        q = starting_q + (solution.q - starting_q + np.pi) % (2 * np.pi) - np.pi
        lower, upper = robot.qlim

        for i in range(len(q)):
            if q[i] < lower[i] or q[i] > upper[i]:
                q[i] = solution.q[i]

        return q

    def move_joint(self, robot, goal, steps=steps):
        # Smoothly move the joints to their target angles.
        path = rtb.jtraj(robot.q, goal, steps)

        for q in path.q:
            robot.q = q
            self.render()

    def move_to(self, robot, target, steps=steps):
        # Find the required joint angles, then move to them.
        goal = self.solve(robot, target, robot.q)
        self.move_joint(robot, goal, steps)

    def move_line(self, robot, target, steps=steps):
        # Move the tool along a straight line using inverse kinematics.
        start = robot.fkine(robot.q)
        path = rtb.ctraj(start, target, steps)

        for position in path:
            robot.q = self.solve(robot, position, robot.q)
            self.render()

    def spin_tool(self, robot, angle, steps=steps):
        # Rotate the last joint to loosen or tighten the wheel nut.
        starting_q = robot.q.copy()
        goal = starting_q.copy()

        lower = robot.qlim[0, -1]
        upper = robot.qlim[1, -1]

        if goal[-1] + angle < lower or goal[-1] + angle > upper:
            angle = -angle

        goal[-1] = goal[-1] + angle
        self.move_joint(robot, goal, steps)

    # 6. LIFTING THE CAR

    def set_lift(self, pitch, steps=steps):
        # Slowly lift or lower the car while Kuka follows the jack point.
        kuka = self.kuka.Robot
        start_pitch = self.car.pitch

        for angle in np.linspace(start_pitch, pitch, steps):
            self.car.pitch = angle
            kuka.q = self.solve(kuka, self.car.jack_target(), kuka.q)
            self.render()

    # 7. PICKING UP AND PUTTING DOWN wheels

    def place_on_stand(self, name, stand):
        lucas = self.lucas.Robot
        target = self.car.wheel_target(self.stands[stand])

        # Approach, place the wheel, release it, and move away.
        self.move_to(lucas, target * back(0.2))
        self.move_line(lucas, target, 30)
        self.attach(name, "stand_" + stand)
        self.move_line(lucas, target * back(0.2), 30)

    def pick_from_stand(self, name, stand):
        lucas = self.lucas.Robot
        target = self.car.wheel_target(self.stands[stand])

        # Approach the wheel, grip it, and move away.
        self.move_to(lucas, target * back(0.2))
        self.move_line(lucas, target, 30)
        self.attach(name, "lucas")
        self.move_line(lucas, target * back(0.2), 30)

    # 8. RUN THE FULL PIT STOP

    def run(self):
        car = self.car
        kuka = self.kuka.Robot
        abb = self.abb.Robot
        lucas = self.lucas.Robot

        # STEP 1: Kuka lifts the front of the car.
        print("Kuka KR300: jacking up the front of the car")
        self.move_to(kuka, car.jack_target() * back(0.3))
        self.move_line(kuka, car.jack_target(), 30)
        self.set_lift(car.lift_pitch)

        # STEP 2: ABB removes the wheel nut.
        print("ABB IRB140: removing the wheel nut")
        self.move_to(abb, car.nut_target() * back(0.15))
        self.move_line(abb, car.nut_target(), 30)
        self.attach("nut", "abb")
        self.spin_tool(abb, -2 * np.pi)
        self.move_line(abb, car.nut_target() * back(0.2), 30)
        self.move_joint(abb, self.abb_home)

        # STEP 3: Staubli removes the old wheel.
        print("Staubli TX90: removing the old wheel")
        self.move_to(lucas, car.wheel_target() * back(0.15))
        self.move_line(lucas, car.wheel_target(), 30)
        self.attach(car.work_wheel, "lucas")
        self.move_line(lucas, car.wheel_target() * back(0.3))
        self.place_on_stand(car.work_wheel, "old")

        # STEP 4: Staubli picks up and fits the new wheel.
        print("Staubli TX90: fitting the new tyre")
        self.pick_from_stand("new_wheel", "new")
        self.move_to(lucas, car.wheel_target() * back(0.3))
        self.move_line(lucas, car.wheel_target())
        self.attach("new_wheel", "hub_" + car.work_wheel)
        self.move_line(lucas, car.wheel_target() * back(0.15), 30)
        self.move_joint(lucas, self.lucas_home)

        # STEP 5: ABB refits the wheel nut.
        print("ABB IRB140: refitting the wheel nut")
        self.move_to(abb, car.nut_target() * back(0.2))
        self.move_line(abb, car.nut_target(), 30)
        self.spin_tool(abb, 2 * np.pi)
        self.attach("nut", "hub_" + car.work_wheel)
        self.move_line(abb, car.nut_target() * back(0.15), 30)
        self.move_joint(abb, self.abb_home)

        # STEP 6: Kuka lowers the car.
        print("Kuka KR300: lowering the car")
        self.set_lift(0.0)
        self.move_line(kuka, car.jack_target() * back(0.3), 30)
        self.move_joint(kuka, self.kuka_home)

        print("Pit stop complete")


# 9. START THE SIMULATION

if __name__ == "__main__":
    env = swift.Swift()
    env.launch(realtime=True)

    pit_stop = PitStop(env)

    input("Press enter to start the pit stop")
    pit_stop.run()

    env.hold()
