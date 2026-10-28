# Race around the pit lane: red car keeps lapping, silver car pits on "Pit now"
# 41013 Industrial Robotics - Lab Assignment 2
#
# Driving logic from F1_Race_Pit_Simulation.py. The silver car is the pit stop's
# CarModel: it races until "Pit now" is pressed, takes the next pit entry and
# stops in its box at this scene's origin, where the robots work on it.

import math
from spatialmath import SE3

from Car.CarModel import CarModel
from pit_lane import place, Pit_box_x
from race_track import (build_race_track, circuit_pose, pit_lane_pose, pit_exit_pose,
                        Lap_length, Pit_entry_x, Pit_exit_x, Pit_exit_end, Straight_half)

Race_speed = 18.0   # m/s, both cars while racing
Pit_limit = 6.0     # m/s, pit lane speed limit
Entry_decel = 7.0   # m/s^2, braking from race speed to the limiter
Box_brake = 2.5     # m/s^2, braking into the pit box
Exit_accel = 6.0    # m/s^2, pulling away from the box and back up to race speed
Start_distance = 1.0  # both cars start side by side near the start of the lower straight

red_paint = [0.91, 0.10, 0.12, 1]


def track_pose(x, y, yaw):
    # F1 simulation coordinates and heading -> car pose in this scene
    return place(x, y, 0) * SE3.Rz(yaw)


class SilverDriver:
    """Laps on the outside lane until a pit is requested, then takes the next
    pit entry, slows to the pit limiter and stops in the box. leave() drives
    it out of the pit exit and back into the race."""

    def __init__(self):
        self.state = "racing"  # racing -> pit -> stopped -> exit -> racing
        self.pit_requested = False
        self.s = Start_distance  # distance along the circuit while racing
        self.x = None            # x position once in the pit lane
        self.speed = Race_speed

    def status(self):
        if self.state == "stopped":
            return "Silver: stopped in pit box"
        if self.state == "pit":
            return "Silver: in pit lane"
        if self.state == "exit":
            return "Silver: leaving the pits"
        if self.pit_requested:
            return "Silver: box, box! Pitting at next entry"
        return "Silver: racing"

    def leave(self):
        if self.state == "stopped":
            self.state = "exit"
            self.pit_requested = False

    def step(self, dt):
        if self.state == "exit":
            # Pit limiter down the lane, then speed back up on the exit road.
            limit = Pit_limit if self.x < Pit_exit_x else Race_speed
            self.speed = min(limit, self.speed + Exit_accel * dt)
            self.x += self.speed * dt
            if self.x < Pit_exit_end:
                return pit_exit_pose(self.x)
            # The exit ends on the lower straight: carry on racing from there.
            self.state = "racing"
            self.s = self.x + Straight_half
            return circuit_pose(self.s, lane_offset=-3.0)

        if self.state == "racing":
            self.speed = min(Race_speed, self.speed + Exit_accel * dt)
            self.s += self.speed * dt
            x, y, yaw, steer = circuit_pose(self.s, lane_offset=-3.0)
            # Turn in only on the lower straight before the pit entry; otherwise wait a lap.
            if self.pit_requested and self.s % Lap_length < Pit_entry_x + Straight_half:
                self.state = "pit"
                self.x = x
                return pit_lane_pose(x)
            return x, y, yaw, steer

        if self.state == "pit":
            # Slow to the limiter, then brake to stop exactly in the box.
            remaining = max(0.0, Pit_box_x - self.x)
            target = min(Pit_limit, math.sqrt(2.0 * Box_brake * remaining))
            self.speed = max(target, self.speed - Entry_decel * dt)
            self.x = min(Pit_box_x, self.x + self.speed * dt)
            if self.x >= Pit_box_x - 1e-3:
                self.x, self.speed, self.state = Pit_box_x, 0.0, "stopped"

        return pit_lane_pose(self.x)


class RaceMixin:

    def setup_race(self):
        self.race_track = build_race_track()
        self.driver = SilverDriver()

        # Second car: same model, red paint, no pit stop.
        self.red_car = CarModel(red_paint)
        self.red_s = Start_distance
        self.red_car_meshes = [self.red_car.body, *self.red_car.wheel_meshes.values()]

        self.race_step(0.0)

    def car_in_box(self):
        return self.driver.state == "stopped"

    def request_pit(self, _=None):
        if self.driver.state == "racing":
            self.driver.pit_requested = True

    def request_leave(self, _=None):
        # Only once the car is down, nothing is moving and the work wheel and its nut are on.
        if not self.car_in_box():
            return
        hub = "hub_" + self.car.work_wheel
        wheel_on = hub in (self.mounts[self.car.work_wheel][0], self.mounts["new_wheel"][0])
        if (self.safety.state != "READY" or self.car.pitch != 0
                or not wheel_on or self.mounts["nut"][0] != hub):
            self.safety.reason = "Finish the pit stop before leaving"
            print(self.safety.reason)
            return
        self.driver.leave()

    def race_step(self, dt):
        self.red_s += Race_speed * dt
        x, y, yaw, steer = circuit_pose(self.red_s, lane_offset=3.0)
        red = self.red_car
        red.roll_to(track_pose(x, y, yaw), steer)
        red.body.T = (red.pose() * red.body_offset).A
        for name, mesh in red.wheel_meshes.items():
            mesh.T = (red.wheel_frame(name) * red.wheel_offset).A

        # The silver car's meshes are drawn by update_props, like the rest of the props.
        if not self.car_in_box():
            x, y, yaw, steer = self.driver.step(dt)
            self.car.roll_to(track_pose(x, y, yaw), steer)
