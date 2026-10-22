import numpy as np
import spactialgeometry as sg
from spatialmath import SE3
from pistop_config import (stand_height, stand_raduis, stand_offset, tyre_stand_grey, tool_grey, dark_grey, floor_grey, random_obstacle_count)
from pitlane import build_pitlane

class scene:
    def build_floor_stands(self):

    #add wheel stands
        self.stands ={}
        self.stands_shapes =[]
    for name, (dx,dy) in stand_offset.items():
        pose = SE3(staubli.base.t[0] +dx, staubli.base.t[1], +dy, stand_height)