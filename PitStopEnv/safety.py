try:
    import serial
except ImportError:
    serial = None


class Safety:
    def __init__(self, port=None):
        self.state = "READY"
        self.reason = ""
        self.serial = None
        self.reset_requested = False
        self.start_requested = False

        if port and serial:
            try:
                self.serial = serial.Serial(port, 115200, timeout=0)
                print(f"Arduino connected on {port}")
            except Exception as exc:
                print(f"Arduino not connected: {exc}")

    def stop(self, state, reason):
        self.state = state
        self.reason = reason
        print(f"{state}: {reason}")

    def reset(self):
        if self.state not in ("RUNNING", "RECOVERING"):
            self.state = "READY"
            self.reason = ""
            print("System reset -> READY")
            return True
        return False

    def start(self):
        if self.state in ("READY", "RECOVERY_READY"):
            self.state = "RECOVERING" if self.state == "RECOVERY_READY" else "RUNNING"
            self.reason = ""
            return True
        return False

    def poll(self):
        if not self.serial:
            return

        while self.serial.in_waiting:
            msg = self.serial.readline().decode(errors="ignore").strip().upper()
            if msg == "ESTOP":
                self.stop("ESTOP", "Physical emergency stop")
            elif msg == "RESET":
                self.reset_requested = True
            elif msg == "START":
                if self.state in ("READY", "RECOVERY_READY"):
                    self.start_requested = True
