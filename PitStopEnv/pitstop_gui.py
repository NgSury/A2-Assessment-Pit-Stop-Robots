import numpy as np
from spatialmath import SE3
from swift import Slider, Button, Label
from pitstop_config import sequence_transits

class PitStopGUI:
    
    #makes selectable robot list for the GUI
    def robot_list(self):
        return [
            self.stabunli.robot,
            self.kuka.robot,
            self.abb.robot
        ]
    #makes the sliders for the GUI
    def select_robot(self, value):
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
        if not self.path_clear(robot, [robot.q, q]):
            self.safety.reason = "Joint Blocked"
            return
        self.recovery.begin()
        robot.q = q
        
    #incrementally moves the robot in the x, y, or z direction by a small amount
    def request_jog(self, dx=0, dy=0, dz=0):
        if self.safety.state == "System Ready" or self.recovery.complete:
            self.jog_request = (dx, dy, dz)
            
    #initial check to see if the system is ready to operate
    def request_start(self, _=None):
        if self.safety.state in ("System Ready", "Recovery Ready"):
            self.run_requested = True
    
    #create demo obstacle for the robot to avoid
    #appears randomly
    def place_obstacle(self, _=None):
        if self.safety.state != "READY" or self.recovery.complete:
            print("RESET before arming the obstacle")
            return
        self.clear_obstacle()
        # It drops onto a random transit of the next run (see MotionMixin.drop_random_obstacle).
        self.obstacle_drop_at = int(np.random.randint(1, sequence_transits + 1))
        print("Obstacle armed: it will appear in a random robot's path during the next run")
    #clear the obstacle
    def clear_obstacle(self, _=None):
        self.obstacle_drop_at = None
        self.obstacle_active = False
        self.obstacle_pose = SE3 (0, 0, -5)
        self.obstacle.T = self.obstacle_pose.A
    
    #safety
    
    def trigger_light_curtain(self, _=None):
        self.safety.stop( "FAULT",
                        "Light Curtain Triggered")
    
    #GUI Setup
    
    def add_gui(self):
        if self.env is None:
            return
        self.status = Label(desc = "State: Ready")
        self.env.add(self.status)
        
        #robot selection with buttons
        for i, name in enumerate(["Stäubli", "Kuka", "ABB"]):
            self.env.add(Button(cb=lambda _, i=i: self.select_robot(i), desc="Select " +name))
        
        #joint sliders, tells each joint where to go
        self.joint_sliders = []
        for i in range(6):
            slider = Slider(
                cb=lambda angle, j=i: self.slider_joint(angle, j),
                min=-180,
                max=180,
                step=0.1,
                value=0,
                desc=f"Joint {i + 1}",
                unit="deg"
            )
            self.joint_sliders.append(slider)
            self.env.add(slider)
        
        #add jogs
        
        jogs = {
            "X +":(0.5, 0, 0),
            "X -":(-0.5, 0, 0),
            "Y +":(0, 0.5, 0),
            "Y -":(0, -0.5, 0),
            "Z +":(0, 0, 0.5),
            "Z -":(0, 0, -0.5)
        }
        
        for label, move in jogs.items():
            self.env.add(Button(cb=lambda _, move=move: self.request_jog(*move), desc=label))
        #add start, estop, reset, obstacle and light trigger buttons
        buttons = [
            Button (cb =lambda _: self.request_start(), desc="Start"),
            Button (cb=lambda _: self.recovery.begin(), desc="Reset"),
                Button (cb=lambda _: self.safety.stop("E-Stop", "GUI Emergency Stop"), desc="E-Stop"),
            Button (cb=lambda _: self.place_obstacle(), desc="Place Obstacle"),
            Button (cb=lambda _: self.clear_obstacle(), desc="Clear Obstacle"),
            Button (cb=lambda _: self.trigger_light_curtain(), desc="Trigger Light Curtain")
        ]
        for button in buttons:
            self.env.add(button)