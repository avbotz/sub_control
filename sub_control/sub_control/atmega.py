import serial
import time
import rclpy
from std_msgs.msg import String

from sub_control_interfaces.msg import State

PORT = "/dev/ttyUSB0"
SIM_PORT = "/tmp/sim_port.txt"

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
            self.serial = open(SIM_PORT, "a+")
            self.node = rclpy.create_node("atmega_node")
            self.publisher = self.node.create_publisher(String, "/nemo/commands", 1)
        else:
            self.serial = serial.Serial(PORT)

        self.write("p 0.2\n")

    def write(self, command: str):
        """
        Write command to arduino

        :param command: command to send (str)
        """
        if self.sim:
            msg = String()
            msg.data = command
            self.publisher.publish(msg)
            print(f"published to sim {command}")
        else:
            self.serial.write(bytes(command))

    def read(self):
        time.sleep(0.1)
        return self.serial.readline()

    def set_power(self, power: float):
        self.write("p 0.15\n")

    def write_state(self, state: State):
        self.relative(state)

    def write_depth(self, depth: float):
        self.write("z {depth}\n")

    def relative(self, state: State):
        """
        Send relative state command (relative to current position).
        """
        self.write(
            f"s {state.x} {state.y} {state.z} {state.yaw} {state.pitch} {state.roll}\n")

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
        Get current depth of sub
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
