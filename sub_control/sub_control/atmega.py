import serial
from sub_control.state import State

PORT = "/dev/ttyUSB0"

class Atmega:
    """
    Utilities to interface with the Arduino on board the sub.
    """
    def __init__(self):
        """
        initialize stuff
        """
        self.state = [0, 0, 0, 0, 0, 0]
        self.serial = serial.Serial(PORT)

    def write(self, command: str):
        """
        Write command to arduino

        :param command: command to send (str)
        """
        self.serial.write(bytes(command))

    def write_state(self, state: State):
        self.relative()

    def write_depth(self, depth: float):
        self.write("z {depth}")

    def relative(self, state: State):
        """
        Send relative state command (relative to current position).
        """
        self.serial.write(bytes(
            f"s {state.x} {state.y} {state.z} {state.yaw} {state.pitch} {state.roll}\n"))

    def alive(self):
        """
        Get whether sub is alive.
        """

        # Send request
        self.serial.write("a\n")

        killed = int(self.serial.read())
        return not killed

    def depth(self):
        """
        Get current depth of sub
        """

        # Send request for depth
        self.serial.write("w\n")

        return float(self.serial.read())

    def state(self):
        """
        Get current state
        """
        # Send request for state
        self.serial.write("c\n")

        x, y, z, yaw, pitch, roll = map(float, self.serial.read().split())

        return State(x, y, z, yaw, pitch, roll)
