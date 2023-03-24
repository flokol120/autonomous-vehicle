from cmath import sqrt
from typing import Dict
from xmlrpc.client import Boolean
from simple_pid import PID
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

from sensor_msgs.msg import Range

RANGE = 3.0
MAX_RANGE = 5.0
SPEED = 0.5
p, i, d = 100, .001, 1

class SensorController(Node):
    
    def __init__(self):
        super().__init__('sensor_controller')
        self.auto_speed = self.create_publisher(Float64, '/speed', 1)
        self.auto_steer = self.create_publisher(Float64, '/steering', 1)
        self.tof = self.create_subscription(Range, '/tof_0', self.roundDriver, 1)
        self.pid = PID(p, i, d, setpoint=RANGE)
        self.pid.output_limits = (-45.0, 45.0)
        self.pid.sample_time = 1/60

    def roundDriver(self, msg):
        output = self.pid(msg.range)
        print(f"PID: {output}")
        msg = Float64()
        msg.data = output
        self.auto_steer.publish(msg)
        msg.data = SPEED
        self.auto_speed.publish(msg)


    def inRange(self, value, lower, upper):
        return min(max(value, lower), upper)
    

def main(args=None):
    rclpy.init(args=args)

    controller = SensorController()

    rclpy.spin(controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()