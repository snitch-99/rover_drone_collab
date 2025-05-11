#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State

class ArmRoverNode(Node):
    def __init__(self):
        super().__init__('arm_rover_node')

        self.current_state = None

        # Subscriptions
        self.state_sub = self.create_subscription(
            State,
            '/rover/state',
            self.state_cb,
            10
        )

        # Clients
        self.arm_client = self.create_client(CommandBool, '/rover/cmd/arming')
        self.mode_client = self.create_client(SetMode, '/rover/set_mode')

        self.create_timer(1.0, self.arm_and_mode_timer_callback)

    def state_cb(self, msg):
        self.current_state = msg

    def arm_and_mode_timer_callback(self):
        if self.current_state is None:
            self.get_logger().info('Waiting for rover state...')
            return

        if not self.current_state.connected:
            self.get_logger().info('Waiting for rover FCU connection...')
            return

        if not self.arm_client.service_is_ready() or not self.mode_client.service_is_ready():
            self.get_logger().info('Waiting for MAVROS services for rover...')
            return

        if not self.current_state.armed:
            self.get_logger().info('Sending arming command to rover...')
            req = CommandBool.Request()
            req.value = True
            future = self.arm_client.call_async(req)

            def arm_callback(fut):
                try:
                    response = fut.result()
                    if response.success:
                        self.get_logger().info('✅ Rover armed successfully!')
                    else:
                        self.get_logger().warn('❌ Failed to arm the rover.')
                except Exception as e:
                    self.get_logger().error(f'Arming service call failed: {e}')

            future.add_done_callback(arm_callback)

        elif self.current_state.mode != 'OFFBOARD':
            self.get_logger().info('Setting rover mode to OFFBOARD...')
            mode_req = SetMode.Request()
            mode_req.custom_mode = 'OFFBOARD'
            mode_future = self.mode_client.call_async(mode_req)

            def mode_callback(fut):
                try:
                    response = fut.result()
                    if response.mode_sent:
                        self.get_logger().info('✅ OFFBOARD mode set successfully for rover.')
                    else:
                        self.get_logger().warn('❌ Failed to set OFFBOARD mode for rover.')
                except Exception as e:
                    self.get_logger().error(f'SetMode service call failed: {e}')

            mode_future.add_done_callback(mode_callback)

        else:
            self.get_logger().info('Rover is already armed and in OFFBOARD mode.')

def main(args=None):
    rclpy.init(args=args)
    node = ArmRoverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

