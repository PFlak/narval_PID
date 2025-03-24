import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from geometry_msgs.msg import Wrench, WrenchStamped
from sensor_msgs.msg import FluidPressure
from narval_msgs.msg import PIDOutput
from narval_pid.PID import PID

import time

class DepthPID(PID, Node):
    def __init__(self):
        PID.__init__(self)
        Node.__init__(self, 'depth_pid')

        self.__Kp = 0.
        self.__Ki = 0.
        self.__Kd = 0.
        self.__Kaw = 0.
        self.__max_rate = 0
        self.__max = 0.
        self.__min = 0.
        self.__setpoint = .0
        self.__input_wrench = ''
        self.__input_pressure = ''
        self.__input_margin = .0
        self.__status = True
        self.__output_wrench = ''
        self.__publish_statistics = True

        self.declare_parameter("kp", -0.04)
        self.declare_parameter("ki", 0.0)
        self.declare_parameter("kd", 0.0)
        self.declare_parameter("kaw", 0.0)
        self.declare_parameter("max_rate", 30)
        self.declare_parameter("max", 20.0)
        self.declare_parameter("min", -20.0)
        self.declare_parameter("setpoint", 0.0)
        self.declare_parameter("input_wrench", "joy_wrench_stmp")
        self.declare_parameter("input_margin", 2.0)
        self.declare_parameter("status", True)
        self.declare_parameter("input_pressure", "bluerov2/pressure")
        self.declare_parameter("output_wrench", "PID/depth")
        self.declare_parameter("publish_statistics", True)

        if not self.input_pressure_name:
            self._logger.warning(f"No input pressure topic. Depth_pid will remain inactive")

        self._logger.info('I\'m here')
        self.sub_input_wrench = self.create_subscription(WrenchStamped, self.input_wrench_name, self.cb_input_wrench, 10)
        self.sub_input_pressure = self.create_subscription(FluidPressure, self.input_pressure_name, self.cb_pressure, 10)

        self.pub_output_wrench = self.create_publisher(WrenchStamped, self.output_wrench_name, 0)

        if self.publish_statistics:
            self.pub_statistics = self.create_publisher(PIDOutput, 'PID/stats', 0)

        self._update_setpoint_flag = False

        self._wrench_prev = WrenchStamped()

    @property
    def Kp(self) -> float:
        self.__Kp = self.get_parameter('kp').value
        return self.__Kp

    @Kp.setter
    def Kp(self, kp: float):
        parameter = Parameter('kp', Parameter.Type.DOUBLE, kp)
        self.set_parameters([parameter])
        self.__Kp = kp
    
    @property
    def Ki(self) -> float:
        self.__Ki = self.get_parameter('ki').value
        return self.__Ki
    
    @Ki.setter
    def Ki(self, ki: float):
        parameter = Parameter('ki', Parameter.Type.DOUBLE, ki)
        self.set_parameters([parameter])
        self.__Ki = ki

    @property
    def Kd(self) -> float:
        self.__Kd = self.get_parameter('kd').value
        return self.__Kd
    
    @Kd.setter
    def Kd(self, kd):
        parameter = Parameter('kd', Parameter.Type.DOUBLE, kd)
        self.set_parameters([parameter])
        self.__Kd = kd

    @property
    def max(self):
        self.__max = self.get_parameter('max').value
        return self.__max
    
    @property
    def min(self):
        self.__min = self.get_parameter('min').value
        return self.__min
    
    @property
    def max_rate(self):
        self.__max_rate = self.get_parameter('max_rate').value
        return self.__max_rate

    @property
    def setpoint(self) -> float:
        self.__setpoint = self.get_parameter('setpoint').value
        return self.__setpoint
    
    @setpoint.setter
    def setpoint(self, setpoint):
        parameter = Parameter('setpoint', Parameter.Type.DOUBLE, setpoint)
        self.set_parameters([parameter])
        self.__setpoint = setpoint

    @property
    def status(self) -> bool:
        self.__status = self.get_parameter('status').value
        return self.__status
    
    @status.setter
    def status(self, status):
        parameter = Parameter('status', Parameter.Type.BOOL, status)
        self.set_parameters([parameter])
        self.__status = status

    @property
    def input_wrench_name(self) -> str:
        self.__input_wrench = self.get_parameter('input_wrench').value
        return self.__input_wrench
    
    @input_wrench_name.setter
    def input_wrench_name(self, input_wrench):
        parameter = Parameter('input_wrench', Parameter.Type.STRING, input_wrench)
        self.set_parameters([parameter])
        self.__input_wrench = input_wrench

    @property
    def input_wrench_margin(self) -> float:
        self.__input_margin = self.get_parameter('input_margin').value
        return self.__input_margin
    
    @input_wrench_margin.setter
    def input_wrench_margin(self, input_margin):
        parameter = Parameter('input_margin', Parameter.Type.DOUBLE, input_margin)
        self.set_parameters([parameter])
        self.__input_margin = input_margin

    @property
    def input_pressure_name(self) -> str:
        self.__input_pressure = self.get_parameter('input_pressure').value
        return self.__input_pressure
    
    @input_pressure_name.setter
    def input_pressure_name(self, input_pressure):
        parameter = Parameter('input_pressure', Parameter.Type.STRING, input_pressure)
        self.set_parameters([parameter])
        self.__input_pressure = input_pressure

    @property
    def output_wrench_name(self) -> str:
        self.__output_wrench = self.get_parameter('output_wrench').value
        return self.__output_wrench
    
    @output_wrench_name.setter
    def output_wrench_name(self, output_wrench):
        parameter = Parameter('output_wrench', Parameter.Type.STRING, output_wrench)
        self.set_parameters([parameter])
        self.__output_wrench = output_wrench

    @property
    def publish_statistics(self) -> bool:
        self.__publish_statistics = self.get_parameter('publish_statistics').value
        return self.__publish_statistics
    
    @publish_statistics.setter
    def publish_statistics(self, publish_statistics):
        parameter = Parameter('publish_statistics', Parameter.Type.BOOL, publish_statistics)
        self.set_parameters([parameter])
        self.__publish_statistics = publish_statistics
    
    def cb_input_wrench(self, msg: WrenchStamped):
        z_force = msg.wrench.force.z
        self._wrench_prev = msg
        # self._logger.info(f"{self._wrench_prev.wrench.force}")
        if abs(z_force) >= self.input_wrench_margin:
            self._update_setpoint_flag = True
        else:
            self._update_setpoint_flag = False

    def cb_pressure(self, msg: FluidPressure):

        pressure = msg.fluid_pressure / 997
        self.T = time.time()

        # self._logger.info(f"Pressure: {pressure}, flag: {self._update_setpoint_flag}")

        if self._update_setpoint_flag:
            self.setpoint = pressure

        command, err = self.step(pressure)
        # self._logger.info(f"{self.integral}; {self.deriv_prev}")

        now = self.get_clock().now()

        wrench = self._wrench_prev
        wrench.header.stamp = now.to_msg()
        
        if not self._update_setpoint_flag:
            wrench.wrench.force.z += command

        self.pub_output_wrench.publish(wrench)

        stats = PIDOutput()
        stats.command = command
        stats.error = err
        stats.setpoint = self.setpoint

        self.pub_statistics.publish(stats)


if __name__ == '__main__':
    rclpy.init()
    
    depth = DepthPID()

    
    rclpy.spin(depth)

    depth.destroy_node()
    rclpy.shutdown()
