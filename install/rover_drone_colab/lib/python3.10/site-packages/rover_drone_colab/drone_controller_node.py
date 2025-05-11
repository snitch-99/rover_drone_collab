#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from enum import Enum
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, MountControl
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from cv_bridge import CvBridge
import cv2
import math

class DroneState(Enum):
    INIT = 0
    MOVING_TO_CAPTURE = 1
    CAPTURE_IMAGE = 2
    WAIT_FOR_ROVER = 3
    DESCEND = 4
    DONE = 5

class DroneController(Node):
    def __init__(self):
        super().__init__('drone_controller_node')

        # Publishers
        self.setpoint_pub = self.create_publisher(PoseStamped, '/drone/setpoint_position/local', 10)
        self.gimbal_pub = self.create_publisher(MountControl, '/drone/mount_control/command', 10)

        # Subscriptions
        self.state_sub = self.create_subscription(State, '/drone/state', self.state_callback, 10)

        best_effort_qos = QoSProfile(depth=10)
        best_effort_qos.reliability = QoSReliabilityPolicy.BEST_EFFORT

        self.pose_sub = self.create_subscription(
            PoseStamped, '/drone/local_position/pose', self.pose_callback, qos_profile=best_effort_qos)

        self.camera_sub = self.create_subscription(Image, '/camera', self.camera_callback, 10)

        # ✅ Simplified X-only rover tracking
        self.rover_x = None
        self.rover_x_sub = self.create_subscription(Float64, '/rover/x_position', self.rover_x_callback, 10)

        self.bridge = CvBridge()

        self.current_state = DroneState.INIT
        self.state_counter = 0
        self.image_saved = False
        self.current_mode = None
        self.current_position = None

        self.capture_target = (2.0, 2.0, 4.0)  # x, y, z
        self.fixed_y = 2.0
        self.initial_z = 4.0
        self.final_z = 0.2
        self.position_threshold = 0.3

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('FSM started. Awaiting OFFBOARD mode...')

    def state_callback(self, msg: State):
        self.current_mode = msg.mode

    def pose_callback(self, msg: PoseStamped):
        self.current_position = (
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        )

    def rover_x_callback(self, msg: Float64):
        self.rover_x = msg.data

    def camera_callback(self, msg: Image):
        if self.current_state != DroneState.CAPTURE_IMAGE or self.image_saved:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            save_path = '/home/kanav/project_ws/src/rover_drone_colab/captured_image.png'
            cv2.imwrite(save_path, cv_image)
            self.get_logger().info(f'📸 Image saved to: {save_path}')
            self.image_saved = True
        except Exception as e:
            self.get_logger().error(f'Image save failed: {e}')

    def timer_callback(self):
        self.state_counter += 1

        match self.current_state:

            case DroneState.INIT:
                if self.state_counter == 1:
                    self.get_logger().info('Entered INIT state.')
                self.publish_setpoint(0.0, 0.0, 0.5)

                gimbal_cmd = MountControl()
                gimbal_cmd.mode = 2
                gimbal_cmd.pitch = -90.0
                self.gimbal_pub.publish(gimbal_cmd)

                if self.current_mode == 'OFFBOARD':
                    self.get_logger().info('OFFBOARD detected → MOVING_TO_CAPTURE')
                    self.current_state = DroneState.MOVING_TO_CAPTURE
                    self.state_counter = 0

            case DroneState.MOVING_TO_CAPTURE:
                if self.state_counter == 1:
                    self.get_logger().info('Entered MOVING_TO_CAPTURE state.')

                x, y, z = self.capture_target
                self.publish_setpoint(x, y, z)

                if self.current_position:
                    dist = math.dist(self.current_position, (x, y, z))
                    if dist < self.position_threshold and self.state_counter > 30:
                        self.get_logger().info('📸 Reached capture target → CAPTURE_IMAGE')
                        self.current_state = DroneState.CAPTURE_IMAGE
                        self.state_counter = 0

            case DroneState.CAPTURE_IMAGE:
                if self.state_counter == 1:
                    self.get_logger().info('Entered CAPTURE_IMAGE state.')

                self.publish_setpoint(*self.capture_target)

                if self.state_counter > 5:
                    self.get_logger().info('✅ Timed capture complete → WAIT_FOR_ROVER')
                    self.current_state = DroneState.WAIT_FOR_ROVER
                    self.state_counter = 0

            case DroneState.WAIT_FOR_ROVER:
                if self.state_counter == 1:
                    self.get_logger().info('Entered WAIT_FOR_ROVER state.')

                if self.state_counter <= 20:
                    self.publish_setpoint(*self.capture_target)

                elif 20 < self.state_counter <= 250:
                    if self.rover_x is not None:
                        self.publish_setpoint(self.rover_x, self.fixed_y, self.initial_z)
                        self.get_logger().info(f'📍 Tracking rover at X={self.rover_x:.2f}')
                    else:
                        self.publish_setpoint(*self.capture_target)

                else:
                    if self.rover_x is None:
                        self.rover_x = 0.0
                        self.get_logger().warn('⚠️ No rover X received. Using fallback.')

                    self.get_logger().info('🕳️ Done following → DESCEND')
                    self.current_state = DroneState.DESCEND
                    self.state_counter = 0

            case DroneState.DESCEND:
                if self.state_counter == 1:
                    self.get_logger().info('Entered DESCEND state.')

                if self.rover_x is not None:
                    x = self.rover_x
                    z = self.initial_z + (self.final_z - self.initial_z) * min(self.state_counter / 60.0, 1.0)
                    self.publish_setpoint(x, self.fixed_y, z)

                    if self.current_position:
                        dist = math.dist(self.current_position, (x, self.fixed_y, z))
                        if dist < self.position_threshold and self.state_counter > 60:
                            self.get_logger().info('🛬 Reached target → DONE')
                            self.current_state = DroneState.DONE
                            self.state_counter = 0

            case DroneState.DONE:
                if self.state_counter == 1:
                    self.get_logger().info('✅ FSM complete. Drone is landed and idle.')

    def publish_setpoint(self, x, y, z):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "map"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.w = 1.0
        self.setpoint_pub.publish(pose)

def main(args=None):
    rclpy.init(args=args)
    node = DroneController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
