import math
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    meca500_world_dir = get_package_share_directory('meca500_world')

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(meca500_world_dir, 'launch', 'sim.launch.py')
        )
    )

    meca500_control = Node(
        package="meca500_control",
        executable="meca500_sine_joint1",
        output="screen",
        parameters=[{
        "cycle_frequency_hz" : 1000,
        "sine_wave_period_s" : 3.0,
        "sine_wave_amplitude" : 0.5
    }]
    )

    return LaunchDescription([
        meca500_control,
        sim_launch,
        ])