import swift
import spatialgeometry as sg
from spatialmath import SE3
from pathlib import Path
import numpy as np
    
class CarModel:
    car_mod = Path(__file__).resolve().parent / "stl" / "car.stl"

    #change stl dimensions to meters
    car_conv = 0.66/39.0 #

    #measurements of car in meters
    mesh_conv = 0.66/39.0 
    #based on the tyre stl file, 39 is the diameter of the tyre from the stl file and we need the tyre to be 0.66m

    # Points measured from the car STL file.
    car_centre = np.array([141.2, 0.0, 500.0])
    car_frontHub_x = 239.9 #x distance of the front axel from the origin
    car_rearHub_x = 42.5 #y distance of the rear axel from the origin
    car_axel_y = 19.5 #y distance of the axels from the ground
    car_axel_z = {"front": 39.0, "rear": 38.0} #track of the tyres
    car_jackPoint = np.array([285.0, 7.0, 500.0]) #where the Kuka bot needs to lift the car from, measured from the origin of the STL file

    # Wheel dimensions.
    wheel_centre = 19.5
    nut_seat = 22.0
    wheel_outer_z = 23.0
    nut_length = 0.06
    nut_radius = 0.04

    # Colours.
    car_silver = [0.75, 0.77, 0.80, 1.0]
    tyre_black = [0.08, 0.08, 0.08, 1.0]
    new_tyre_red = [0.35, 0.05, 0.05, 1.0]
    tyre_stand_grey = [0.55, 0.55, 0.6, 1.0]
    nut_red = [0.85, 0.1, 0.1, 1.0]
    tool_grey = [0.3, 0.3, 0.3, 1.0] #if stl files for kuka end eff found remove
    dark_grey = [0.2, 0.2, 0.2, 1.0] #if stl files for kuka end eff found remove
    floor_grey = [0.35, 0.35, 0.38, 1.0]

    # The front-right wheel is changed.
    work_wheel = "FR"
    wheels = {
        "FL": ("front", 1),
        "FR": ("front", -1),
        "RL": ("rear", 1),
        "RR": ("rear", -1)
    }
    # Colours.
    car_silver = [0.75, 0.77, 0.80, 1.0]
    tyre_black = [0.08, 0.08, 0.08, 1.0]
    new_tyre_red = [0.35, 0.05, 0.05, 1.0]
    tyre_stand_grey = [0.55, 0.55, 0.6, 1.0]
    nut_red = [0.85, 0.1, 0.1, 1.0]
    tool_grey = [0.3, 0.3, 0.3, 1.0] #if stl files for kuka end eff found remove
    dark_grey = [0.2, 0.2, 0.2, 1.0] #if stl files for kuka end eff found remove
    floor_grey = [0.35, 0.35, 0.38, 1.0]

    # The front-right wheel is changed.
    work_wheel = "FR"
    wheels = {
        "FL": ("front", 1),
        "FR": ("front", -1),
        "RL": ("rear", 1),
        "RR": ("rear", -1)}
        
    # The car tilts slightly when the front is lifted.
    left_pitch = np.deg2rad(1.2)
    Kuka_base_posXY = (3.7, 1.9)

    # Lengths of the tools attached to the robots.
    Kuka_toolLength = 0.45
    ARB_toolLength = 0.18
    Lucas_toolLength = 0.10

    # Wheel storage stands.
    standHeight = 0.25
    standRadius = 0.3
    standOffset = {
        "old": (-0.55, -0.1),
        "new": (-0.1, -0.65)
    }

if __name__ == "__main__":
    Car = CarModel()
    env = swift.Swift()
    env.launch(realtime=True)
    env.add(Car.car_mod, SE3(0, 0, 0.5), Car.car_silver, Car.car_conv)
