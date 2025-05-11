#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([

        # MAVROS for Drone (namespaced under /drone)
        ExecuteProcess(
            cmd=[
                'bash', '-c',
                'source /opt/ros/humble/setup.bash && '
                'source ~/project_ws/install/setup.bash && '
                'ros2 launch mavros px4.launch '
                'fcu_url:=udp://:14540@127.0.0.1:14557 '
                'gcs_url:=udp://@127.0.0.1:14550 '
                'tgt_system:=1 tgt_component:=1 '
                'namespace:=drone'
            ],
            output='screen'
        ),

        ## MAVROS for Rover
        ExecuteProcess(
            cmd=[
                'bash', '-c',
                'source /opt/ros/humble/setup.bash && '
                'source ~/project_ws/install/setup.bash && '
                'ros2 launch mavros px4.launch '
                'fcu_url:=udp://:14541@127.0.0.1:14558 '
                'gcs_url:=udp://@127.0.0.1:14560 '
                'tgt_system:=2 tgt_component:=1 '
                'namespace:=rover'
            ],
            output='screen'
        ),

        # Drone Controller
        ExecuteProcess(
            cmd=[
                'gnome-terminal', '--title=Drone Controller', '--', 'bash', '-c',
                'source /opt/ros/humble/setup.bash && '
                'source ~/project_ws/install/setup.bash && '
                'ros2 run rover_drone_colab drone_controller_node; exec bash'
            ],
            output='screen'
        ),

        ExecuteProcess(
            cmd=[
                'gnome-terminal', '--title=Drone Controller', '--', 'bash', '-c',
                'source /opt/ros/humble/setup.bash && '
                'source ~/project_ws/install/setup.bash && '
                'ros2 run rover_drone_colab rover_controller_node; exec bash'
            ],
            output='screen'
        ),

        # Drone Arming Node
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'rover_drone_colab', 'arm_node'
            ],
            output='screen'
        ),

        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'rover_drone_colab', 'arm_rover'
            ],
            output='screen'
        ),
    ])