#make the scene look more believable

import spatialgeometry as sg
from spatialmath import SE3

#location of pitbox
pitbox_x = 21
pitbox_y = -37.2

#Colours for pretty
silver = [0.77, 0.80, 0.86, 1]
red = [0.91, 0.10, 0.12, 1]
black = [0.075, 0.080, 0.090, 1]
pit_asphalt = [0.27, 0.29, 0.32, 1]
white = [0.96, 0.96, 0.94, 1]
yellow = [1.00, 0.82, 0.10, 1]
barrier = [0.72, 0.75, 0.77, 1]
dark = [0.11, 0.14, 0.18, 1]
glass = [0.28, 0.47, 0.59, 1]

def place(x,y,z):
    #puts pit box in the centre
    return SE3(x - pitbox_x, y - pitbox_y)

def box (size, x, y, z, colour):
    return sg.Cuboid(size, pose = place(x,y,z), colour = colour)

def cylinder(radius, length, x, y, z, color):
    return sg.Cylinder(radius=radius, length=length, pose=place(x, y, z), color=color)

def sphere(radius, x, y, z, color):
    return sg.Sphere(radius, pose=place(x, y, z), color=color)

def lane_and_markings():
    shapes = [
        box ([75, 7.5, .1], 18, pitbox_y, -.05, pit_asphalt),
        box ([77, 7.6, .1], 18, -44.2, -.05, pit_asphalt),
        #solid white edges
        box ([46, .12, .014], 29, pitbox_y -3.63, .014, white)
        ]
    # yellow speed limit dashes
    shapes += [box ([ 3, .13, .014], x, pitbox_y + 3.5, .016, yellow) for x in range (15, 54, 6)]
    #white dashed centre line
    shapes += [box ([ 2.5, .09, .014], x, pitbox_y, .014, white) for x in range (0, 52, 6)]
    return shapes


def pit_box():
    # yellow box painted on ground with a red mark where the car stops
    shapes = [box ([.12, 6.8, .016], pitbox_x + dx, pitbox_y, .017, yellow) for dx in (-4,4)]
    shapes += [box ([ 8, .12, .016], pitbox_x, pitbox_y + dy, .017, yellow) for dy in (-3.4,3.4)]
    shapes.append(box ([.12, 3.4, .016], pitbox_x + 3.3, pitbox_y, .018, red))
    return shapes

def pit_wall():
    return[
        box([38.0, 0.30, 0.9], 34, -30.8, 0.45, barrier),
        box([38.0, 0.13, 0.14], 34, -30.8, 0.96, dark),
    ]

def garages():
    shapes = []
    #5 garages
    for index, x in enumerate((-17.0, -3.0, 11.0, 25.0, 39.0)):
        team_color = silver if index == 3 else (red if index == 1 else [0.20, 0.37, 0.70, 1])
        shapes += [
            box([12.6, 11.0, 5.1], x, -51.8, 2.55, [0.70, 0.72, 0.74, 1]),
            box([12.9, 12.2, 0.40], x, -51.9, 5.3, dark),
            # Dark door opening, team panel above, metal door frame
            box([9.8, 0.16, 3.9], x, -46.22, 1.98, dark),
            box([10.0, 0.22, 0.70], x, -46.10, 4.55, team_color),
        ]
        shapes += [box([0.22, 0.22, 3.8], x + dx, -46.04, 1.95, white) for dx in (-4.9, 4.9)]
        
    # Pit control tower with glass windows
    shapes += [
        box([17.0, 10.0, 8.0], 53, -51.9, 4.0, [0.63, 0.66, 0.69, 1]),
        box([17.4, 10.4, 0.5], 53, -51.9, 8.4, dark),
    ]
    shapes += [box([3.2, 0.13, 1.8], x, -46.80, 6.1, glass) for x in (47, 51, 55, 59)]
    return shapes

def gantry():
    #red gantry over pit box
    shapes = [box([0.50, 0.50, 6.4], 21, y, 3.20, white) for y in (-45.8, -32.5)]
    shapes += [
        box([1.1, 14.1, 0.52], 21, -39.2, 6.35, red),
        box([1.15, 14.1, 1.0], 21, -39.2, 5.65, dark),
    ]
    return shapes

def equipment():
    shapes = []
    # Equipment carts beside the box
    for x, y in ((15.2, -43.0), (27.5, -43.0), (17.0, -31.9)):
        shapes += [
            box([1.0, 0.7, 1.0], x, y, 0.50, red),
            box([0.8, 0.15, 0.25], x, y - 0.45, 0.82, black),
        ]
    return shapes

def build_pit_lane():
    """All pit lane shapes, positioned around this scene's pit box at the origin."""
    return (lane_and_markings() + pit_box() + pit_wall()
            + garages() + gantry() + equipment())