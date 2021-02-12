import serial
import time
import rclpy
from std_msgs.msg import String

from sub_control_interfaces.msg import State

PORT = "/dev/ttyUSB0"
BB_TO_SIM = "/tmp/bb_to_sim.txt"
SIM_TO_BB = "/tmp/sim_to_bb.txt"

class Atmega:
    """
    Utilities to interface with the Arduino on board the sub.
    """
    def __init__(self, sim=False):
        """
        initialize stuff
        """
        self._state = [0, 0, 0, 0, 0, 0]
        self.sim = sim
        if self.sim:
            self.sim_to_bb = open(SIM_TO_BB, "r+")
            self.bb_to_sim = open(BB_TO_SIM, "w+")
        else:
            self.serial = serial.Serial(PORT)

    def write(self, command: str):
        """
        Write command to arduino

        :param command: command to send (str)
        """
        if self.sim:
            self.bb_to_sim.write(command)
            self.bb_to_sim.flush()
        else:
            self.serial.write(bytes(command))

    def read(self):
        time.sleep(0.1)
        if self.sim:
            return self.sim_to_bb.readline()
        else:
            return self.serial.readline()

    def set_power(self, power: float):
        self.write(f"p {power}\n")

    def write_state(self, state: State):
        """
        Send absolute state command.
        """
        self.write(
            f"s {state.x} {state.y} {state.z} {state.yaw} {state.pitch} {state.roll}\n")

    def write_depth(self, depth: float):
        self.write(f"z {depth}\n")

    def relative(self, state: State):
        """
        Send relative state command (relative to current position).
        """
        self.write(
            f"r {state.x} {state.y} {state.z} {state.yaw} {state.pitch} {state.roll}\n")

    def alive(self):
        """
        Get whether sub is alive.
        """

        # Send request
        self.write("a\n")

        alive = bool(int(self.read()))
        return alive

    def depth(self):
        """
        Get current depth (altitude above floor) of sub
        """

        # Send request for depth
        self.write("w\n")

        return float(self.read())

    def state(self):
        """
        Get current state
        """
        # Send request for state
        self.write("c\n")

        x, y, z, yaw, pitch, roll = map(float, self.read().split())

        return State(x=x, y=y, z=z, yaw=yaw, pitch=pitch, roll=roll)
