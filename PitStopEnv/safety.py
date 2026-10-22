"""Shared safety state and Arduino panel for the pit-stop demo."""

import time

try:
    import serial
except ImportError:
    serial = None


button_bits = {
    "ack": 0,
    "estop": 1,
    "abb": 2,
    "staubli": 3,
    "kuka": 4,
    "spare": 5,
}

led_bits = {
    "system": 0,
    "kuka": 3,
    "staubli": 6,
    "abb": 9,
}

colour_bits = {"green": 0, "yellow": 1, "red": 2}


class Safety:
    def __init__(self, port=None):
        self.state = "READY"
        self.reason = ""
        self.serial = None
        self.button_mask = 0
        self.press_edges = set()
        self.last_rx = 0.0
        self.last_led_mask = None
        self.reset_requested = False
        self.start_requested = False
        self.recovery_release_required = False

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

    def pressed(self, name):
        return bool(self.button_mask & (1 << button_bits[name]))

    def recovery_buttons(self):
        return [name for name in ("kuka", "staubli", "abb") if self.pressed(name)]

    def poll(self):
        if not self.serial:
            return

        while self.serial.in_waiting:
            line = self.serial.readline().decode(errors="ignore").strip()
            parts = line.split()
            if len(parts) != 3 or parts[0] != "B":
                continue

            try:
                new_mask = int(parts[2])
            except ValueError:
                continue

            pressed_now = new_mask & ~self.button_mask
            for name, bit in button_bits.items():
                if pressed_now & (1 << bit):
                    self.press_edges.add(name)

            self.button_mask = new_mask
            self.last_rx = time.monotonic()

        if self.last_rx and time.monotonic() - self.last_rx > 0.5:
            if self.state in ("RUNNING", "RECOVERY_READY", "RECOVERING"):
                self.stop("FAULT", "Arduino panel communication lost")
            return

        if self.pressed("estop"):
            if self.state != "ESTOP":
                self.stop("ESTOP", "Physical emergency stop")
            return

        if "ack" in self.press_edges:
            self.press_edges.discard("ack")
            if self.state in ("ESTOP", "FAULT", "COLLISION"):
                self.state = "RECOVERY_READY"
                self.reason = "Acknowledged - release recovery buttons"
                self.recovery_release_required = True

    def leds(self, system="off", kuka="off", staubli="off", abb="off"):
        if not self.serial:
            return

        mask = 0
        for group, colour in {
            "system": system,
            "kuka": kuka,
            "staubli": staubli,
            "abb": abb,
        }.items():
            if colour != "off":
                mask |= 1 << (led_bits[group] + colour_bits[colour])

        if mask != self.last_led_mask:
            self.serial.write(f"L {mask}\n".encode())
            self.last_led_mask = mask
