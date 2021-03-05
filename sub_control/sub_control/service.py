import rclpy
from rclpy.node import Node

from sub_control_interfaces.msg import State
from sub_control.atmega import Atmega
from sub_control_interfaces.srv import ControlAlive, ControlDepth, ControlSetPower, ControlState, ControlWrite, ControlWriteDepth, ControlWriteState

class ControlService(Node):
    """
    Service for controlling sub.
    """
    def __init__(self, sim=False):
        """
        Register services
        """
        super().__init__('control_service')
        self.declare_parameter("SIM")
        self.sim_enabled = self.get_parameter("SIM").get_parameter_value().bool_value

        self.atmega = Atmega(self.sim_enabled)
        self.logger = self.get_logger()
        self.control_alive_service = self.create_service(ControlAlive, "control_alive", self.alive)
        self.control_state_service = self.create_service(ControlState, "control_state", self.state)
        self.control_depth_service = self.create_service(ControlDepth, "control_depth", self.depth)
        self.control_write_service = self.create_service(ControlWrite, "control_write", self.write)
        self.control_set_power_service = \
            self.create_service(ControlSetPower, "control_set_power", self.set_power)
        self.control_write_state_service = \
            self.create_service(ControlWriteState, "control_write_state", self.write_state)
        self.control_write_depth_service = \
            self.create_service(ControlWriteDepth, "control_write_depth", self.write_depth)

        self.param_timer = self.create_timer(2, self.param_timer_callback)

    def param_timer_callback(self):
        sim = self.get_parameter("SIM").get_parameter_value().bool_value

        sim_param = rclpy.parameter.Parameter(
            "SIM",
            rclpy.Parameter.Type.BOOL,
            sim
        )

        params = [sim_param]
        self.set_parameters(params)

    def alive(self, request, response):
        response.data: bool = self.atmega.alive()
        print(f"Received alive request. Alive: {response.data}", flush=True)
        return response

    def state(self, request, response):
        state: State = self.atmega.state()
        print("Received request for state. State: ({}, {}, {}, {}, {}, {})".format(
            round(state.x, 2),
            round(state.y, 2),
            round(state.z, 2),
            round(state.yaw, 2),
            round(state.pitch, 2),
            round(state.roll, 2)), flush=True)
        response.x = state.x
        response.y = state.y
        response.z = state.z
        response.yaw = state.yaw
        response.pitch = state.pitch
        response.roll = state.roll
        return response

    def depth(self, request, response):
        response.depth: float = self.atmega.depth()
        print(f"Received request for depth. Depth: {round(response.depth, 2)}", flush=True)
        return response

    def set_power(self, request, response):
        print(f"Received request for set power. Power: {round(request.power, 2)}", flush=True)
        self.atmega.set_power(request.power)
        return response

    def write(self, request, response):
        print(f"Received request for manual write. Command: {request.data}", flush=True)
        self.atmega.write(request.data)
        return response

    def write_state(self, request, response):
        state: State = State(x=request.x,
                             y=request.y,
                             z=request.z,
                             yaw=request.yaw,
                             pitch=request.pitch,
                             roll=request.roll)
        print("Received request for write state. State: ({}, {}, {}, {}, {}, {})".format(
            round(state.x, 2),
            round(state.y, 2),
            round(state.z, 2),
            round(state.yaw, 2),
            round(state.pitch, 2),
            round(state.roll, 2)), flush=True)
        self.atmega.write_state(state)
        return response

    def write_depth(self, request, response):
        print(f"Received request for depth write. Depth: {round(request.dist, 2)}", flush=True)
        self.atmega.write_depth(request.dist)
        return response


def main(args=None):
    rclpy.init(args=args)

    control_service = ControlService()
    rclpy.spin(control_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
