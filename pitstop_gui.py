# Pit Stop Swift GUI: robot select, joint sliders, jog/control buttons, demo obstacle
# 41013 Industrial Robotics - Lab Assignment 2

import numpy as np
from spatialmath import SE3
from swift import Slider, Button, Label



class GUIMixin:

    def robot_list(self):
        return [
            self.staubli.Robot,
            self.kuka.Robot,
            self.abb.Robot
        ]

    def select_robot(self, value):
        self.selected_robot = int(value)
        robot = self.robot_list()[self.selected_robot]

        for i, slider in enumerate(self.joint_sliders):
            low, high = np.rad2deg(robot.links[i].qlim)

            slider.min = float(low)
            slider.max = float(high)
            slider.value = float(np.rad2deg(robot.q[i]))

    def slider_joint(self, angle, joint):
        if self.safety.state != "READY" or self.recovery.complete:
            return

        robot = self.robot_list()[self.selected_robot]
        q = robot.q.copy()

        q[joint] = np.deg2rad(float(angle))
        if not self.path_clear(robot, [robot.q, q]):
            self.safety.reason = "Joint jog blocked: " + self.collision_manager.reason
            return
        self.recovery.begin()
        robot.q = q

    def request_jog(self, dx=0, dy=0, dz=0):
        if self.safety.state == "READY" and not self.recovery.complete:
            self.jog_request = (dx, dy, dz)

    def request_start(self, _=None):
        if self.safety.state in ("READY", "RECOVERY_READY"):
            self.run_requested = True

    def request_recovery(self, _=None):
        # Match controller A: queue recovery for the main loop, rather than
        # moving the simulation directly inside a Swift GUI callback.
        if self.safety.state not in ("ESTOP", "CURTAIN_STOP"):
            print("Recovery only available after an E-STOP or light-curtain stop")
            return
        if self.safety.light_curtain_broken:
            self.safety.reason = "Clear the light curtain before recovery"
            print(self.safety.reason)
            return
        if self.controller_estop.pressed():
            self.safety.reason = "Release controller B before recovery"
            print(self.safety.reason)
            return
        self.safety.recovery_requested = True
        print("GUI recovery requested: reverse recorded motion")

    # Demo obstacle

    def place_obstacle(self, _=None):
        if self.safety.state != "READY" or self.recovery.complete:
            print("RESET before arming the obstacle")
            return
        self.clear_obstacle()
        # Arm the first safe, eligible movement; the motion code validates the drop.
        self.obstacle_drop_at = 1
        print("Obstacle IN armed. Press START: obstacle will appear on the first eligible robot path.")

    def clear_obstacle(self, _=None):
        self.obstacle_drop_at = None
        self.obstacle_active = False
        self.obstacle_pose = SE3(0, 0, -5)
        self.obstacle.T = self.obstacle_pose.A
        self.random_boxes = []
        for shape in self.random_box_shapes:
            shape.T = self.obstacle_pose.A

    # Safety

    def trigger_light_curtain(self, _=None):
        self.safety.break_light_curtain()
        self.run_requested = False
        self.jog_request = None
        print("GUI: light curtain BREAK triggered")

    def clear_light_curtain(self, _=None):
        self.safety.clear_light_curtain()
        print("GUI: light curtain CLEAR triggered")

    # GUI setup

    def add_gui(self):
        if self.env is None:
            return

        self.status = Label(desc="STATE: READY")
        self.env.add(self.status)

        # Race: bring the silver car in before START
        self.race_status = Label(desc=self.driver.status())
        self.env.add(self.race_status)
        self.env.add(Button(cb=self.request_pit, desc="Pit now"))
        self.env.add(Button(cb=self.request_leave, desc="Leave pit"))

        # One button per robot chooses which robot the sliders and jog buttons move.
        for i, name in enumerate(["Staubli TX90", "KUKA KR300", "ABB IRB140"]):
            self.env.add(
                Button(
                    cb=lambda _, j=i: self.select_robot(j),
                    desc="Select " + name
                )
            )

        self.joint_sliders = []

        for i in range(6):
            low, high = np.rad2deg(
                self.staubli.Robot.links[i].qlim
            )

            slider = Slider(
                cb=lambda angle, j=i: self.slider_joint(angle, j),
                min=float(low),
                max=float(high),
                step=1,
                value=float(np.rad2deg(self.staubli.Robot.q[i])),
                desc=f"Joint {i + 1}",
                unit="deg"
            )

            self.joint_sliders.append(slider)
            self.env.add(slider)

        jogs = {
            "X +": (0.05, 0, 0),
            "X -": (-0.05, 0, 0),
            "Y +": (0, 0.05, 0),
            "Y -": (0, -0.05, 0),
            "Z +": (0, 0, 0.05),
            "Z -": (0, 0, -0.05)
        }

        for label, move in jogs.items():
            self.env.add(
                Button(
                    cb=lambda _, d=move: self.request_jog(*d),
                    desc=label
                )
            )

        buttons = [
            Button(
                cb=self.request_start,
                desc="START"
            ),
            Button(
                cb=lambda _: self.safety.stop(
                    "ESTOP",
                    "GUI emergency stop"
                ),
                desc="E-STOP"
            ),
            Button(
                cb=self.request_recovery,
                desc="RECOVER TO SAFE STATE"
            ),
            Button(
                cb=lambda _: setattr(self, "reset_requested", True),
                desc="RESET"
            ),
            Button(
                cb=self.place_obstacle,
                desc="Obstacle IN"
            ),
            Button(
                cb=self.clear_obstacle,
                desc="Obstacle OUT"
            ),
            Button(
                cb=self.trigger_light_curtain,
                desc="LIGHT CURTAIN BREAK"
            ),
            Button(
                cb=self.clear_light_curtain,
                desc="LIGHT CURTAIN CLEAR"
            )
        ]

        for button in buttons:
            self.env.add(button)
