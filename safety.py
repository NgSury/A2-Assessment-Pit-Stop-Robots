"""Shared safety state for the pit-stop simulation."""

class Safety:
    def __init__(self, controller=None):
        self.state = "READY"
        self.reason = ""
        self.controller = controller
        self.reset_requested = False
        self.start_requested = False
        self.recovery_requested = False
        self.light_curtain_broken = False

    def stop(self, state, reason):
        self.state = state
        self.reason = reason
        print(f"{state}: {reason}")

    def break_light_curtain(self):
        self.light_curtain_broken = True
        # A real E-stop takes precedence if both conditions occur.
        if self.state != "ESTOP":
            self.stop("CURTAIN_STOP", "Light curtain interrupted - clear before recovery")

    def clear_light_curtain(self):
        self.light_curtain_broken = False
        if self.state == "CURTAIN_STOP":
            self.reason = "Light curtain clear - press RECOVER TO SAFE STATE"
        print("Light curtain clear; restart inhibited until recovery")

    def reset(self):
        if self.light_curtain_broken:
            return False
        if self.state not in ("RUNNING", "RECOVERING"):
            self.state = "READY"
            self.reason = ""
            print("System reset -> READY")
            return True
        return False

    def start(self):
        if self.state == "READY" and not self.light_curtain_broken:
            self.state = "RUNNING"
            self.reason = ""
            return True
        return False

    def poll(self):
        # X is sampled during motion as well as in the main loop.
        # Defer scene mutation until the active trajectory has unwound.
        if self.controller is not None and self.controller.pressed_once(2):  # X button
            self.reset_requested = True
        # Controller E-stop uses the same latched safety state as the GUI stop.
        # Releasing the controller button does not resume motion.
        if self.controller is not None and self.controller.pressed():
            if self.state != "ESTOP":
                self.stop("ESTOP", "Controller emergency stop")
