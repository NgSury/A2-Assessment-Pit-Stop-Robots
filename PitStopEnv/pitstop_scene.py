import numpy as np
import spatialgeometry as sg
from spatialmath import SE3
from pitstop_config import (stand_height, stand_raduis, stand_offset, tyre_stand_grey, tool_grey, dark_grey, floor_grey, random_obstacle_count)
from pitlane import build_pitlane

class scene:
    def build_floor_stands(self):
        staubli = self.staubli.Robot
    #add wheel stands
        self.stands ={}
        self.stands_shapes =[]
        for name, (dx,dy) in stand_offset.items():
            pose = SE3(staubli.base.t[0] +dx, staubli.base.t[1], +dy, stand_height)
            self.stands [name] = pose
        
            self.stands_shapes.append(
                sg.Cylinder(
                    stand_raduis,
                    stand_height,
                    pose = pose * SE3(0, 0, -stand_height/2),
                    colour = tyre_stand_grey
                )
            )
    
    def build_obstacles_curtain(self):
        #collision demo object
        self.obstacle_size = np.array ([.14, .14, .18])
        self.obstacle_pose = SE3 (0, 0, -5)
        self.obstacle_active = False
        self.obstacle_drop_at = None #transit number the armed obstacle drops in
        self.transit_index = None #transits done so far, none outside sequence
        self.obstacle = sg.Cuboid(
            self.obstacle_size,
            pose = self.obstacle_pose,
            colour = [.9, .1, .1, 1]
        )
        
        #extra boxes to spawn randomly
        self.random_boxes =[]
        self.random_box_shapes = [
            sg.Cuboid(
                self.obstacle_size,
                pose=self.obstacle_pose,
                color=[0.9, 0.45, 0.1, 1.0]
            )
            for _ in range(random_obstacle_count)
        ]
        
        #light curtain
        self.light_curtain = [
            sg.Cuboid(
                [.08, .08, 1.2],
                pose = SE3 (-1.8, -2.7, .6),
                colour = [.9,.7, 0, 1]
            ),
            sg.Cuboid(
                [.08, .08, 1.2],
                pose = SE3 (1.8,-2.6,.6),
                colour = []
            ),
            sg.Cuboid(
                [3.6, 0.02, 0.03],
                pose=SE3(0, -2.7, 0.65),
                color=[1.0, 0.0, 0.0, 0.35]
            )
        ]
        def add_to_env(self):
            if self.env is None:
                return

            objects = [
                *self.pit_lane,
                self.kuka.base_mesh,
                self.abb.base_mesh,
                self.staubli.base_mesh,
                *self.stand_shapes,
                *self.props.values(),
                self.obstacle,
                *self.random_box_shapes,
                *self.light_curtain,
                self.kuka.Robot,
                self.abb.Robot,
                self.staubli.Robot
        ]

        for obj in object:
            self.env.add(obj)