import numpy as np 

class recovery:
    def __init__(self,cell):
        self.cell = cell
        self.history = []
        self.recording = False
        self.pending = False
        self.complete = False
        self.signature = None
    def capture(self):
        c = self.cell
        return ([r.q.copy() for r in c.robot_list()], float(c.car.pitch), dict(c.mounts))
    def record(self):
        if not self.recording or self.cell.safety.state == 'RECOVERING':
            return
        state = self.capture()
        signature = (tuple(q.tobytes() for q in state[0]), state[1], tuple((name, parent, offset.A.tobytes()) for name, (parent, offset) in state[2].items()))
        if signature != self.signature:
            self.history.append(state)
            self.signature = signature
    def begin(self):
        self.recording = True
        self.record()

    def prepare(self):
        c = self.cell
        if not c.safety.reset():
            return
        self.recording = False
        self.pending = len(self.history) > 1
        c.run_requested = False
        c.jog_request = None
        if self.pending:
            c.safety.state = 'RECOVERY_READY'
            c.safety.reason = 'Press START to retrace safely; no scene teleport'
        else:
            self.history.clear()
            self.signature = None
            self.complete = False

    def reverse(self):
        c = self.cell
        print('Recovery: retracing executed motion and attachments')
        self.recording = False
        while len(self.history) > 1:
            start = self.capture()
            goal = self.history[-2]
            delta = max(np.max(np.abs(b - a)) for a, b in zip(start[0], goal[0]))
            count = max(1, int(np.ceil(delta / np.deg2rad(1.5))))
            for fraction in np.linspace(0, 1, count + 1)[1:]:
                c.guard()
                configs = [a + fraction * (b - a) for a, b in zip(start[0], goal[0])]
                for robot, q in zip(c.robot_list(), configs):
                    if not c.path_clear(robot, [robot.q, q]):
                        c.safety.stop('COLLISION', 'Recovery blocked; remove the obstruction, RESET then START')
                        c.guard()
                for robot, q in zip(c.robot_list(), configs):
                    robot.q = q
                c.car.pitch = start[1] + fraction * (goal[1] - start[1])
                if fraction == 1:
                    c.mounts = dict(goal[2])
                c.render()
            self.history.pop()
        c.guard()  # A stop on the final frame must remain latched too.
        self.history.clear()
        self.signature = None
        self.pending = self.complete = False
        c.select_robot(c.selected_robot)
        print('Recovery complete: initial scene restored by motion')
