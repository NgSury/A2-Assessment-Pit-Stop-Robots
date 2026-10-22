import numpy as np
from spatialmath import SE3
from swift import Slider, Button, Label
from pitstop_util import back

class PitStopGUI:
    
    #makes selectable robot list for the GUI
    def robot_list(self):
        return [
            self.stabunli.robot,
            self.kuka.robot,
            self.abb.robot
        ]
    #makes the sliders for the GUI
    def select_robot(self, robot):
        self.selected_robot = int(value)
        robot = self.robot_list()[self.selected_robot]
        
        for i, slider in enumerate(self.joint_sliders):
            low, high = np.rad2deg(robot.links[i].qlim)
            
            slider.min = float(low)
            slider.max = float(high)
            slider.value = float(np.rad2deg(robot.q[i]))
            
    # give the user manual control of the robot joints via sliders
    def slider_joint(self, angle, joint):
        if self.safety.state != "System Ready" or self.recovery.complete:
            return
        robot = self.robot_list()[self.selected_robot]
        q = robot.q.copy()
        
        q[joint] = np.deg2rad(float(angle))  # Swift may send the value as a string
        