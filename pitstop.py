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
from pathlib import Path
from KukaRobot.KukaKR300 import KukaKR300
from irb140.ABB_irb140 import ABBIRB140
from Staubli.Lucas_robot import Lucasbot
from Car.CarModel import CarModel

# Simulation speed.
dt = 0.02
steps = 50


# 2. SIMPLE POSITION FUNCTIONS

def mesh_point(point):
    # Convert a point from the STL coordinates to metres.
    difference = np.array(point) - car_centre
    difference = difference * mesh_conv
    return np.array([difference[0], -difference[2], difference[1]])


# The car STL is Y-up. Rotate it into the Swift Z-up world.
car_mesh_pose = SE3.Rx(np.pi / 2) * SE3(*(-car_centre * mesh_conv))
wheel_mesh_pose = SE3(
    -wheel_centre * mesh_conv,
    -wheel_centre * mesh_conv,
    0
)
nut_pose = SE3(0, 0, nut_seat * mesh_conv + nut_length / 2)
jack_pose = SE3(mesh_point(car_jackPoint)) * SE3.Ry(-np.pi / 2)
rear_pivot = mesh_point([car_rearHub_x, car_axel_y, car_centre[2]])


def hub_pose(axle, side):
    # Find the position of a wheel hub on the car.
    if axle == "front":
        x = car_frontHub_x
    else:
        x = car_rearHub_x

    z = car_centre[2] - side * car_axel_z[axle]
    position = mesh_point([x, car_axel_y, z])
    return SE3(position) * SE3.Rx(-side * np.pi / 2)


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
        self.pitch = 0.0

        # Create the three robots.
        self.kuka = KukaKR300()
        self.abb = ABBIRB140()
        self.lucas = Lucasbot()

        kuka = self.kuka.Robot
        abb = self.abb.Robot
        lucas = self.lucas.Robot

        # Position the robots around the car.
        front_hub = hub_pose("front", -1).t
        jack = jack_pose.t
        
        car_centre = CarModel.car_centre
        

        kuka.base = facing(Kuka_base_posXY[0], Kuka_base_posXY[1], jack)
        abb.base = SE3(front_hub[0] + 0.35, -1.6, 0) * SE3.Rz(np.pi / 2)
        lucas.base = SE3(front_hub[0] - 0.7, -1.75, 0) * SE3.Rz(np.pi / 2)

        # Move the base meshes into the same positions.
        self.kuka.base_mesh.T = kuka.base.A
        self.abb.base_mesh.T = abb.base.A
        self.lucas.base_mesh.T = lucas.base.A

        # Add a jack to the Kuka.
        kuka_tool = SE3(0, 0, -Kuka_toolLength) * SE3.Rx(np.pi)
        kuka_shapes = [
            sg.Cuboid(
                [0.12, 0.12, Kuka_toolLength],
                pose=SE3(0, 0, -Kuka_toolLength / 2),
                color=tool_grey
            ),
            sg.Cuboid(
                [0.03, 0.3, 0.2],
                pose=SE3(0, 0, -0.1),
                color=dark_grey
            )
        ]
        add_tool(kuka, kuka_tool, kuka_shapes)

        # Add a wheel-nut tool to the ABB.
        abb_tool = SE3(0, 0, ARB_toolLength)
        abb_shapes = [
            sg.Cylinder(
                0.045, ARB_toolLength - 0.04,
                pose=SE3(0, 0, -(ARB_toolLength + 0.04) / 2),
                color=tool_grey
            ),
            sg.Cylinder(
                nut_radius + 0.008, 0.04,
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

        # Create the floor, car, four wheels, nut and replacement wheel.
        self.floor = sg.Cuboid(
            [12, 7, 0.01],
            pose=SE3(0.5, 0, -0.006),
            color=floor_grey
        )

        self.car = sg.Mesh(
            str(car_mod / "W07.STL"),
            scale=[mesh_conv, mesh_conv, mesh_conv],
            color=car_silver
        )

        self.wheels = {}
        for name in wheels:
            self.wheels[name] = sg.Mesh(
                str(car_mod / "Wheel.STL"),
                scale=[mesh_conv, mesh_conv, mesh_conv],
                color=tyre_black
            )

        self.nut = sg.Cylinder(nut_radius, nut_length, color=nut_red)
        self.new_tyre_red = sg.Mesh(
            str(car_mod / "Wheel.STL"),
            scale=[mesh_conv, mesh_conv, mesh_conv],
            color=new_tyre_red
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
        # This replaces the more complicated lambda functions.
        self.mounts = {}
        self.mounts["car"] = ("car", car_mesh_pose)

        for name in wheels:
            self.mounts[name] = ("hub_" + name, wheel_mesh_pose)

        self.mounts["nut"] = ("hub_" + work_wheel, nut_pose)
        self.mounts["new_tyre_red"] = ("stand_new", wheel_mesh_pose)

        self.props = {
            "car": self.car,
            "nut": self.nut,
            "new_tyre_red": self.new_tyre_red
        }

        for name in self.wheels:
            self.props[name] = self.wheels[name]

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

    def car_pose(self):
        # Tilt the car around the rear axle when lifting its front.
        return SE3(rear_pivot) * SE3.Ry(-self.pitch) * SE3(-rear_pivot)

    def hub_world(self, wheel):
        # Find the wheel hub's position in the world.
        axle, side = wheels[wheel]
        return self.car_pose() * hub_pose(axle, side)

    def parent_pose(self, parent):
        # Find the position of whichever object a part is attached to.
        if parent == "car":
            return self.car_pose()

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
            return self.hub_world(wheel)

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

    def render(self):
        # Move all objects to match the latest robot and car positions.
        for name in self.props:
            self.props[name].T = self.prop_pose(name).A

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

    # 6. CAR AND WHEEL TARGETS

    def jack_target(self):
        return self.car_pose() * jack_pose

    def set_lift(self, pitch, steps=steps):
        # Slowly lift or lower the car while Kuka follows the jack point.
        kuka = self.kuka.Robot
        start_pitch = self.pitch

        for angle in np.linspace(start_pitch, pitch, steps):
            self.pitch = angle
            kuka.q = self.solve(kuka, self.jack_target(), kuka.q)
            self.render()

    def nut_target(self):
        # Position the ABB wheel gun against the nut.
        target = self.hub_world(work_wheel) * nut_pose
        target = target * SE3(0, 0, nut_length / 2)
        return target * SE3.Rx(np.pi)

    def wheel_target(self):
        # Position the Staubli gripper against the car's wheel.
        target = self.hub_world(work_wheel)
        target = target * SE3(0, 0, wheel_outer_z * mesh_conv)
        return target * SE3.Rx(np.pi)

    def stand_target(self, stand):
        # Position the Staubli gripper against a wheel on a stand.
        target = self.stands[stand]
        target = target * SE3(0, 0, wheel_outer_z * mesh_conv)
        return target * SE3.Rx(np.pi)

    # 7. PICKING UP AND PUTTING DOWN wheels

    def place_on_stand(self, name, stand):
        lucas = self.lucas.Robot
        target = self.stand_target(stand)

        # Approach, place the wheel, release it, and move away.
        self.move_to(lucas, target * back(0.2))
        self.move_line(lucas, target, 30)
        self.attach(name, "stand_" + stand)
        self.move_line(lucas, target * back(0.2), 30)

    def pick_from_stand(self, name, stand):
        lucas = self.lucas.Robot
        target = self.stand_target(stand)

        # Approach the wheel, grip it, and move away.
        self.move_to(lucas, target * back(0.2))
        self.move_line(lucas, target, 30)
        self.attach(name, "lucas")
        self.move_line(lucas, target * back(0.2), 30)

    # 8. RUN THE FULL PIT STOP

    def run(self):
        kuka = self.kuka.Robot
        abb = self.abb.Robot
        lucas = self.lucas.Robot

        # STEP 1: Kuka lifts the front of the car.
        print("Kuka KR300: jacking up the front of the car")
        self.move_to(kuka, self.jack_target() * back(0.3))
        self.move_line(kuka, self.jack_target(), 30)
        self.set_lift(left_pitch)

        # STEP 2: ABB removes the wheel nut.
        print("ABB IRB140: removing the wheel nut")
        self.move_to(abb, self.nut_target() * back(0.15))
        self.move_line(abb, self.nut_target(), 30)
        self.attach("nut", "abb")
        self.spin_tool(abb, -2 * np.pi)
        self.move_line(abb, self.nut_target() * back(0.2), 30)
        self.move_joint(abb, self.abb_home)

        # STEP 3: Staubli removes the old wheel.
        print("Staubli TX90: removing the old wheel")
        self.move_to(lucas, self.wheel_target() * back(0.15))
        self.move_line(lucas, self.wheel_target(), 30)
        self.attach(work_wheel, "lucas")
        self.move_line(lucas, self.wheel_target() * back(0.3))
        self.place_on_stand(work_wheel, "old")

        # STEP 4: Staubli picks up and fits the new wheel.
        print("Staubli TX90: fitting the new tyre")
        self.pick_from_stand("new_tyre_red", "new")
        self.move_to(lucas, self.wheel_target() * back(0.3))
        self.move_line(lucas, self.wheel_target())
        self.attach("new_tyre_red", "hub_" + work_wheel)
        self.move_line(lucas, self.wheel_target() * back(0.15), 30)
        self.move_joint(lucas, self.lucas_home)

        # STEP 5: ABB refits the wheel nut.
        print("ABB IRB140: refitting the wheel nut")
        self.move_to(abb, self.nut_target() * back(0.2))
        self.move_line(abb, self.nut_target(), 30)
        self.spin_tool(abb, 2 * np.pi)
        self.attach("nut", "hub_" + work_wheel)
        self.move_line(abb, self.nut_target() * back(0.15), 30)
        self.move_joint(abb, self.abb_home)

        # STEP 6: Kuka lowers the car.
        print("Kuka KR300: lowering the car")
        self.set_lift(0.0)
        self.move_line(kuka, self.jack_target() * back(0.3), 30)
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
