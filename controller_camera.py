"""Game-controller orbit camera for the Swift pit-stop simulation.

Shares the joystick opened by ControllerEstop, so the two never fight over
the device. Uses only the sticks, the triggers and the unused Y button, so
it doesn't overlap the safety buttons:

    Left stick   -> orbit (yaw / pitch)
    Right stick  -> pan the look-at target
    ZL / ZR      -> zoom out / in
    Y (3)        -> reset camera view

Untouched: A (0) RECOVERY, B (1) E-STOP, X (2) FULL RESET, Start (6) START AUTO.
"""

import numpy as np

from controller_estop import Y_button

# Axis numbers for the Switch Pro Controller under pygame 2 / SDL2.
# Check yours with: [round(js.get_axis(i), 2) for i in range(js.get_numaxes())]
LEFT_X, LEFT_Y, RIGHT_X, RIGHT_Y, ZL_AXIS, ZR_AXIS = 0, 1, 2, 3, 4, 5

CAMERA_RESET_BUTTON = Y_button


class ControllerCamera:
    def __init__(self, env, controls, target=(0.0, 0.0, 0.4),
                 yaw=0.0, pitch=0.4, radius=4.0, deadzone=0.15):
        self.env = env
        self.controls = controls          # the ControllerEstop instance
        self.deadzone = deadzone
        self._home = (np.array(target, float), yaw, pitch, radius)
        self._sent = None                 # last pose sent to Swift
        self.reset()

    def reset(self):
        target, self.yaw, self.pitch, self.radius = self._home
        self.target = target.copy()
        self._sent = None                 # always send the reset view

    def _axis(self, i, deadzone=True):
        js = self.controls.joystick
        if js is None or i >= js.get_numaxes():
            return -1.0 if not deadzone else 0.0   # triggers rest at -1
        v = js.get_axis(i)
        return 0.0 if deadzone and abs(v) < self.deadzone else v

    def update(self, dt):
        """Call once per loop, before env.step(dt). Does nothing without a
        controller. ControllerEstop.pressed()/pressed_once() already pump
        pygame events, so this doesn't pump again."""
        if self.env is None or self.controls.joystick is None:
            return

        reset = self.controls.pressed_once(CAMERA_RESET_BUTTON)
        if reset:
            self.reset()

        # Left stick: orbit
        self.yaw -= self._axis(LEFT_X) * 1.5 * dt
        self.pitch = float(np.clip(self.pitch + self._axis(LEFT_Y) * 1.5 * dt, -1.4, 1.4))

        # Right stick: pan target in the ground plane, relative to the view.
        # Pan and zoom speeds scale with distance so the whole race track is reachable.
        fwd = np.array([-np.cos(self.yaw), -np.sin(self.yaw), 0.0])
        right = np.array([-np.sin(self.yaw), np.cos(self.yaw), 0.0])
        speed = max(1.5, 0.5 * self.radius)
        self.target += (self._axis(RIGHT_X) * right - self._axis(RIGHT_Y) * fwd) * speed * dt

        # ZL / ZR: zoom. On the Pro Controller these are digital, -1 or +1.
        zl = (self._axis(ZL_AXIS, deadzone=False) + 1) / 2
        zr = (self._axis(ZR_AXIS, deadzone=False) + 1) / 2
        self.radius = float(np.clip(self.radius * (1 + (zl - zr) * 0.8 * dt), 0.5, 150.0))

        pos = self.target + self.radius * np.array([
            np.cos(self.pitch) * np.cos(self.yaw),
            np.cos(self.pitch) * np.sin(self.yaw),
            np.sin(self.pitch),
        ])

        # Only send when the view changed, so the mouse can still orbit the
        # Swift view while the sticks are left alone.
        pose = np.concatenate([pos, self.target])
        if self._sent is None and not reset:
            self._sent = pose             # first call: keep Swift's own view
            return
        if self._sent is not None and np.allclose(pose, self._sent):
            return
        self._sent = pose
        self.env.set_camera_pose(pos, self.target)
