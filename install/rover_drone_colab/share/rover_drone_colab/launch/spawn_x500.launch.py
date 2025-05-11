#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, IncludeLaunchDescription, LogInfo
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('rover_drone_colab')
    model_path = os.path.join(pkg_share, 'models')
    launch_path = os.path.join(pkg_share, 'launch')

    # Set GZ_SIM_MODEL_PATH
    os.environ['GZ_SIM_MODEL_PATH'] = os.environ.get('GZ_SIM_MODEL_PATH', '') + os.pathsep + model_path

    # Launch PX4 drone (instance 0)
    px4_sitl = ExecuteProcess(
        cmd=[
            'gnome-terminal', '--title=PX4 SITL', '--', 'bash', '-c',
            'cd ~/PX4-Autopilot && PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL=x500_gimbal PX4_GZ_MODEL_POSE="1,2,0.5,0,0,0" ./build/px4_sitl_default/bin/px4; exec bash'
        ],
        output='screen'
    )

    # Launch PX4 rover (instance 1)
    rover_px4 = ExecuteProcess(
        cmd=[
            'gnome-terminal', '--title=PX4 Rover', '--', 'bash', '-c',
            'cd ~/PX4-Autopilot &&  PX4_SIM_MODEL=gz_rover_ackermann PX4_GZ_MODEL_POSE="2,2,0.5,0,0,0" ./build/px4_sitl_default/bin/px4 -i 1; exec bash'
        ],
        output='screen'
    )

    # QGroundControl
    qgc_launch = ExecuteProcess(
        cmd=[
            'gnome-terminal', '--', os.path.expanduser('~/QGroundControl.AppImage')
        ],
        output='screen'
    )

    # Bridge camera topic to /camera
    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='camera_bridge',
        arguments=[
            '/world/default/model/x500_gimbal_0/link/camera_link/sensor/camera/image@sensor_msgs/msg/Image[gz.msgs.Image'
        ],
        remappings=[
            ('/world/default/model/x500_gimbal_0/link/camera_link/sensor/camera/image', '/camera')
        ],
        output='screen'
    )

    # Include fly_hover_setup (MAVROS, FSM, nodes)
    fly_hover_setup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_path, 'fly_hover_setup.launch.py')
        )
    )



    return LaunchDescription([
        px4_sitl,
        TimerAction(period=5.0, actions=[rover_px4]),
        TimerAction(period=15.0, actions=[qgc_launch]),
        TimerAction(period=18.0, actions=[
            LogInfo(msg="Launching MAVROS2 and Hover Setpoints..."),
            fly_hover_setup
        ])
    ])
