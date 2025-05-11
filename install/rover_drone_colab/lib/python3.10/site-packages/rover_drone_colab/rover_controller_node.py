#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from enum import Enum
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class RoverState(Enum):
    STATIC = 0
    MOVING = 1

class RoverController(Node):
    def __init__(self):
        super().__init__('rover_controller_node')

        self.state = RoverState.STATIC
        self.state_counter = 0
        self.rover_x = 0.0  # Track rover’s X position manually

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/rover/setpoint_velocity/cmd_vel_unstamped', 10)
        self.x_pub = self.create_publisher(Float64, '/rover/x_position', 10)

        # Timer at 20 Hz
        self.timer = self.create_timer(0.05, self.timer_callback)

        self.get_logger().info("🚗 Rover Controller Initialized (velocity + x_position publisher)")

    def timer_callback(self):
        self.state_counter += 1

        if self.state == RoverState.STATIC:
            self.publish_velocity(0.0, 0.0, 0.0)
            self.publish_x_position()

            if self.state_counter > 30:
                self.get_logger().info("🟢 Switching to MOVING state")
                self.state = RoverState.MOVING
                self.state_counter = 0

        elif self.state == RoverState.MOVING:
            velocity = 0.1  # m/s
            dt = 0.05       # 20 Hz → 0.05s per tick
            self.rover_x += velocity * dt

            self.publish_velocity(velocity, 0.0, 0.0)
            self.publish_x_position()

    def publish_velocity(self, vx, vy, vz):
        twist = Twist()
        twist.linear.x = vx
        twist.linear.y = vy
        twist.linear.z = vz
        self.cmd_pub.publish(twist)

    def publish_x_position(self):
        msg = Float64()
        msg.data = self.rover_x
        self.x_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RoverController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
