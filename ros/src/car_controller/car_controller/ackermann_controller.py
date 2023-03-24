import math
import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Float64MultiArray, Float64


class AckermannController(Node):
    axle_distance = 0.254
    axle_width = 0.2
    wheel_circumference = 0.2

    def __init__(self):
        super().__init__("ackermann_controller")
        self.steering = 0
        self.speed = 0

        self.steering_pub = self.create_publisher(Float64MultiArray, "/steering_controller/commands", 1)
        self.velocity_pub = self.create_publisher(Float64MultiArray, "/velocity_controller/commands", 1)

        self.steering_sub = self.create_subscription(Float64, "/steering", self.steering_callback, 1)
        self.velocity_sub = self.create_subscription(Float64, "/speed", self.speed_callback, 1)

    def steering_callback(self, msg):
        # invert sign for intuitiveness
        steering_rad = -math.radians(msg.data)
        self.steering = steering_rad

        steering_left = math.atan2(2 * self.axle_distance * math.sin(steering_rad),
                               2 * self.axle_distance * math.cos(steering_rad) - self.axle_width * math.sin(steering_rad))
        steering_right = math.atan2(2 * self.axle_distance * math.sin(steering_rad),
                               2 * self.axle_distance * math.cos(steering_rad) + self.axle_width * math.sin(steering_rad))

        steering_msg = Float64MultiArray()
        steering_msg.data = [ steering_left, steering_right ]
        self.steering_pub.publish(steering_msg)
        self.update_wheel_speeds()

    def update_wheel_speeds(self):
        if self.steering == 0:
            speed_msg = Float64MultiArray()
            speed_msg.data = [ float(self.speed), float(self.speed) ]
            self.velocity_pub.publish(speed_msg)
            return

        center_radius = self.axle_distance / math.atan(self.steering)
        left_radius = self.axle_distance / math.atan(self.steering) - self.axle_width / 2
        right_radius = self.axle_distance / math.atan(self.steering) + self.axle_width / 2

        left_speed_fraction = left_radius / center_radius
        right_speed_fraction = right_radius / center_radius

        speed_msg = Float64MultiArray()
        speed_msg.data = [ left_speed_fraction * self.speed, right_speed_fraction * self.speed ]
        self.velocity_pub.publish(speed_msg)

    def speed_callback(self, msg):
        self.speed = msg.data / self.wheel_circumference * math.pi * 2
        self.update_wheel_speeds()


def main(args=None):
    rclpy.init(args=args)
    ackermann = AckermannController()

    rclpy.spin(ackermann)

    ackermann.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

