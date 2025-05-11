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

    px4_rover = ExecuteProcess(
        cmd=[
            'env',
            'PX4_SIM_MODEL=gz_r1_rover',
            'PX4_GZ_MODEL_POSE=2,2,4,0,0,-1.57',
            './build/px4_sitl_default/bin/px4',
            '-i', '1'
        ],
        cwd=os.path.expanduser('~/PX4-Autopilot'),  # ✅ Set working directory
        output='screen'
    )

    # Include fly_hover_setup (MAVROS, FSM, nodes)
    fly_hover_setup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_path, 'fly_hover_setup_rover.launch.py')
        )
    )

    return LaunchDescription([
        px4_rover,
        #TimerAction(period=18.0, actions=[
        #    LogInfo(msg="Launching MAVROS2 and Hover Setpoints..."),
        #    fly_hover_setup
        #])
    ])
