# Race circuit around the pit lane: oval track, kerbs, barriers, pit entry, scenery
# 41013 Industrial Robotics - Lab Assignment 2
#
# Taken from F1_Race_Pit_Simulation.py (make_circuit and make_scenery), without the
# pit lane parts that are already in pit_lane.py. As there, the coordinates are the
# original ones from that file and pit_lane.place() shifts them so the silver car's
# pit box lands on this scene's origin. The path functions below (circuit_pose,
# pit_lane_pose) also work in the original coordinates.

import math
import numpy as np

from pit_lane import (box, cylinder, sphere, Pit_y,
                      red, black, white, yellow, barrier, dark, pit_asphalt)

# Oval: straights along the bottom and top, semicircles on the ends
Straight_half = 44.0
Track_radius = 23.0
Track_width = 12.0
Bottom_y = -Track_radius
Top_y = Track_radius
Lap_length = 4.0 * Straight_half + 2.0 * math.pi * Track_radius

Pit_entry_x = -18.0  # where the pit entry leaves the lower straight
Pit_exit_x = 25.0    # where the pit exit leaves the pit lane, just past the box
Pit_exit_end = 44.0  # where it rejoins the lower straight, before the right bend
Wheelbase = 3.34     # m, front to rear axle (steering angle = atan(wheelbase * curvature))

# Colours from F1_Race_Pit_Simulation.py (RGBA, 0-1)
asphalt = [0.21, 0.23, 0.26, 1]
grass = [0.19, 0.40, 0.19, 1]
dark_green = [0.08, 0.32, 0.14, 1]
kerb_red = [0.85, 0.11, 0.11, 1]
blue = [0.13, 0.40, 0.76, 1]


# Paths

def smoothstep(u):
    u = max(0.0, min(1.0, u))
    return u * u * (3.0 - 2.0 * u)


def pit_offset(x):
    # Smooth lane change from the outside race lane (y = -26) into the pit lane
    u = (x - Pit_entry_x) / 30.0
    return -26.0 + (Pit_y + 26.0) * smoothstep(u)


def pit_slope(x):
    # dy/dx of the pit entry, used to turn the car along it
    u = (x - Pit_entry_x) / 30.0
    if u <= 0.0 or u >= 1.0:
        return 0.0
    return (Pit_y + 26.0) * 6.0 * u * (1.0 - u) / 30.0


def circuit_pose(distance, lane_offset=0.0):
    """(x, y, heading, steering) after driving `distance` around the oval.

    +lane_offset moves the car to the left of its direction of travel.
    """
    straight = 2.0 * Straight_half
    arc_length = math.pi * Track_radius
    s = distance % Lap_length

    if s < straight:
        x, y, yaw, curvature = -Straight_half + s, Bottom_y, 0.0, 0.0
    elif s < straight + arc_length:
        phi = -math.pi / 2 + (s - straight) / Track_radius
        x = Straight_half + Track_radius * math.cos(phi)
        y = Track_radius * math.sin(phi)
        yaw, curvature = phi + math.pi / 2, 1.0 / (Track_radius - lane_offset)
    elif s < 2 * straight + arc_length:
        x, y, yaw, curvature = Straight_half - (s - straight - arc_length), Top_y, math.pi, 0.0
    else:
        phi = math.pi / 2 + (s - 2 * straight - arc_length) / Track_radius
        x = -Straight_half + Track_radius * math.cos(phi)
        y = Track_radius * math.sin(phi)
        yaw, curvature = phi + math.pi / 2, 1.0 / (Track_radius - lane_offset)

    x -= lane_offset * math.sin(yaw)
    y += lane_offset * math.cos(yaw)
    return x, y, yaw, math.atan(Wheelbase * curvature)


def pit_lane_pose(x):
    # (x, y, heading, steering) on the pit entry and lane. Before the entry this is
    # the outside race lane of the lower straight.
    curvature = (pit_slope(x + 0.3) - pit_slope(x - 0.3)) / 0.6
    steering = max(-0.30, min(0.30, Wheelbase * curvature))
    return x, pit_offset(x), math.atan(pit_slope(x)), steering


def exit_offset(x):
    # Smooth lane change from the pit lane back to the outside race lane (y = -26)
    u = (x - Pit_exit_x) / (Pit_exit_end - Pit_exit_x)
    return Pit_y + (-26.0 - Pit_y) * smoothstep(u)


def exit_slope(x):
    length = Pit_exit_end - Pit_exit_x
    u = (x - Pit_exit_x) / length
    if u <= 0.0 or u >= 1.0:
        return 0.0
    return (-26.0 - Pit_y) * 6.0 * u * (1.0 - u) / length


def pit_exit_pose(x):
    # (x, y, heading, steering) from the pit box, along the lane and out the exit
    curvature = (exit_slope(x + 0.3) - exit_slope(x - 0.3)) / 0.6
    steering = max(-0.30, min(0.30, Wheelbase * curvature))
    return x, exit_offset(x), math.atan(exit_slope(x)), steering


# Track

def bends():
    # Centres and start angles of the right and left semicircles
    return ((Straight_half, -math.pi / 2), (-Straight_half, math.pi / 2))


def ground_and_road():
    shapes = [
        # Grass sits just below the asphalt (road top at z=0), darker infield
        box([180, 155, 0.20], 0, -4, -0.18, grass),
        box([104, 35, 0.04], 0, 0, -0.15, dark_green),
    ]
    # Two straights
    shapes += [box([2 * Straight_half + 0.9, Track_width, 0.10], 0, y, -0.05, asphalt)
               for y in (Bottom_y, Top_y)]
    # The bends, from overlapping boxes
    n = 44
    da = math.pi / n
    for xc, start in bends():
        for i in range(n):
            angle = start + (i + 0.5) * da
            shapes.append(box([Track_radius * da + 0.46, Track_width, 0.10],
                              xc + Track_radius * math.cos(angle), Track_radius * math.sin(angle),
                              -0.05, asphalt, angle + math.pi / 2))
    return shapes


def kerbs_and_lines():
    shapes = []
    # Red/white kerbs along both edges of both straights
    kerb = 5.2
    for x in np.arange(-42.0, 44.0, kerb):
        color = white if int((x + 42.0) / kerb) % 2 == 0 else kerb_red
        for y in (Bottom_y - Track_width / 2, Bottom_y + Track_width / 2,
                  Top_y - Track_width / 2, Top_y + Track_width / 2):
            shapes.append(box([kerb + 0.03, 0.45, 0.045], x + kerb / 2, y, 0.018, color))

    # Kerbs around both radii of both bends
    n = 38
    da = math.pi / n
    for xc, start in bends():
        for i in range(n):
            angle = start + (i + 0.5) * da
            color = white if i % 2 == 0 else kerb_red
            for radius in (Track_radius - Track_width / 2, Track_radius + Track_width / 2):
                shapes.append(box([radius * da + 0.14, 0.45, 0.045],
                                  xc + radius * math.cos(angle), radius * math.sin(angle),
                                  0.018, color, angle + math.pi / 2))

    # Broken white centre line on the straights
    for y in (Bottom_y, Top_y):
        shapes += [box([4.2, 0.13, 0.012], x, y, 0.012, white) for x in range(-39, 43, 10)]

    # Chequered start/finish line across the lower straight
    for i in range(12):
        for j in range(2):
            shapes.append(box([0.72, 0.95, 0.012], -33.5 + j * 0.72, Bottom_y - 5.5 + i * 0.98,
                              0.014, white if (i + j) % 2 == 0 else black))
    return shapes


def pit_entry():
    # Curved road from the lower straight into the pit lane, following the car's path
    return [box([1.75, 7.0, 0.102], x, pit_offset(x), -0.049, pit_asphalt, math.atan(pit_slope(x)))
            for x in np.linspace(Pit_entry_x, 12, 33)]


def pit_exit():
    # Curved road from the pit lane back onto the lower straight
    return [box([1.75, 7.0, 0.102], x, exit_offset(x), -0.049, pit_asphalt, math.atan(exit_slope(x)))
            for x in np.linspace(Pit_exit_x, Pit_exit_end, 22)]


def barriers():
    shapes = [
        # Top straight, and the lower straight either side of the pit lane
        box([95, 0.34, 1.2], 0, 30.9, 0.60, barrier),
        box([16, 0.34, 1.2], -42, -30.9, 0.60, barrier),
        box([14, 0.34, 1.2], 51, -30.9, 0.60, barrier),
    ]
    # Outside of both bends
    n = 26
    da = math.pi / n
    radius = Track_radius + Track_width / 2 + 1.4
    for xc, start in bends():
        for i in range(n):
            angle = start + (i + 0.5) * da
            shapes.append(box([radius * da + 0.10, 0.32, 1.2],
                              xc + radius * math.cos(angle), radius * math.sin(angle),
                              0.6, barrier, angle + math.pi / 2))
    # Tyre walls in front of the grandstands
    shapes += [cylinder(0.55, 0.49, float(x), 33.0, z, black)
               for x in np.arange(-38, 43, 4.0) for z in (0.25, 0.74)]
    return shapes


# Scenery

def grandstand(x, y, width=31):
    # Stepped grandstand facing the track, with coloured seat rows and a roof
    shapes = [box([width + 2, 12.0, 0.35], x, y, 0.0, [0.35, 0.37, 0.39, 1])]
    for level in range(5):
        yy = y - 4.5 + 2.0 * level
        height = 0.6 + 0.48 * level
        shapes += [
            box([width, 2.0, height], x, yy, 0.2 + height / 2, [0.42, 0.44, 0.47, 1]),
            box([width - 1.0, 0.8, 0.18], x, yy, 0.34 + height, blue if level % 2 == 0 else red),
        ]
    shapes += [box([0.35, 0.35, 6.2], xx, y + 4.5, 3.15, dark)
               for xx in (x - width / 2 + 1.5, x + width / 2 - 1.5)]
    shapes += [
        box([width + 5, 13.5, 0.35], x, y + 0.8, 6.3, white),
        box([width + 5, 0.35, 1.2], x, y + 7.45, 5.5, red),
    ]
    return shapes


def start_gantry():
    # Overhead start/finish gantry with start lights, posts outside the track
    shapes = [box([0.65, 0.65, 7.0], -33, y, 3.5, dark) for y in (-31.6, -14.6)]
    shapes += [
        box([1.5, 18.0, 1.3], -33, -23.1, 7.0, red),
        box([1.56, 17.5, 0.30], -33, -23.1, 7.8, white),
    ]
    shapes += [sphere(0.30, -32.12, y, 6.9, yellow)
               for y in (-27.0, -25.4, -23.8, -22.2, -20.6, -19.0)]
    return shapes


def floodlights_and_trees():
    shapes = []
    for x, y in ((-66, -40), (-60, 39), (-20, 59), (20, 59),
                 (65, 39), (75, -20), (10, -64), (-40, -64)):
        shapes += [
            cylinder(0.20, 9.0, x, y, 4.5, [0.59, 0.62, 0.65, 1]),
            box([2.8, 0.7, 0.65], x, y, 9.0, white),
        ]
    for x, y in ((-80, -50), (-78, 49), (82, 47), (83, -44),
                 (-10, 8), (6, -4), (-29, 3), (28, 8),
                 (0, 12), (-70, 3), (70, 5), (5, -68),
                 (-60, -66), (61, -66)):
        shapes += [
            cylinder(0.26, 3.4, x, y, 1.7, [0.37, 0.25, 0.14, 1]),
            sphere(1.8, x, y, 4.0, dark_green),
            sphere(1.45, x + 0.7, y + 0.2, 4.6, grass),
        ]
    return shapes


def build_race_track():
    """All circuit shapes outside the pit lane, positioned around this scene's pit box."""
    return (ground_and_road() + kerbs_and_lines() + pit_entry() + pit_exit() + barriers()
            + grandstand(-22, 42) + grandstand(23, 42) + start_gantry()
            + floodlights_and_trees())
