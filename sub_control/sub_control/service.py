import rclpy
from rclpy.node import Node

from sub_control.atmega import Atmega
from sub_control_interfaces.srv import ControlAlive, ControlDepth, ControlState, ControlWrite, ControlWriteDepth, ControlWriteState

class ControlService(Node):
    """
    Service for controlling sub.
    """
    def __init__(self):
        """
        Register services
        """
        super().__init__('control_service')
        self.atmega = Atmega()
        self.logger = self.get_logger()
        self.control_alive_service = self.create_service(ControlAlive, "control_alive", self.alive)
        self.control_state_service = self.create_service(ControlState, "control_state", self.state)
        self.control_depth_service = self.create_service(ControlDepth, "control_depth", self.depth)
        self.control_write_service = self.create_service(ControlWrite, "control_write", self.write)
        self.control_write_state_service = \
            self.create_service(ControlWriteState, "control_write_state", self.write_state)
        self.control_write_depth_service = \
            self.create_service(ControlWriteDepth, "control_write_depth", self.write_depth)

    def alive(self, request, response):
        response.data: bool = self.atmega.alive()
        self.logger.info(f"Received alive request. Alive: {response.data}")
        return response

    def state(self, request, response):
        state: State = self.atmega.state()
        self.logger.info(f"Received request for state. State: {state}")
        response.x = state.x
        response.y = state.y
        response.z = state.z
        response.yaw = state.yaw
        response.pitch = state.pitch
        response.roll = state.roll
        return response

    def depth(self, request, response):
        response.depth: float = self.atmega.depth()
        self.logger.info(f"Received request for depth. Depth: {response.depth}")
        return response

    def write(self, request, response):
        self.logger.info(f"Received request for manual write. Command: '{request.data}'")
        self.atmega.write(request.data)
        return response

    def write_state(self, request, response):
        state: State = State(request.x,
                             request.y,
                             request.z,
                             request.yaw,
                             request.pitch,
                             request.roll)
        self.logger.info(f"Received request for state write. State: '{state}'")

        self.atmega.write_state(state)
        return response

    def write_depth(self, request, response):
        self.logger.info(f"Received request for depth write. Depth: '{request.dist}'")
        self.atmega.write_depth(request.dist)



def main(args=None):
    rclpy.init(args=args)

    control_service = ControlService()
    rclpy.spin(control_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
