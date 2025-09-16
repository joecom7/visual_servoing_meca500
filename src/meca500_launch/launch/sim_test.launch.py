import math
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    aipr_2507_support_dir = get_package_share_directory('aipr_2507_support')

    included_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(aipr_2507_support_dir, 'launch', 'sim.launch.py')
        )
    )

    main = Node(
        package="esame",
        executable="Main",
        output="screen",
        parameters=[{
        "panda_joint_movement_duration" : PANDA_JOINT_MOVEMENT_DURATION,
        "meca_joint_movement_duration" : MECA_JOINT_MOVEMENT_DURATION,
    }]
    )

    panda_joint_traj = Node(
        package="esame",
        executable="PandaJointTraj",
        output="screen",
        parameters=[
            {"cycle_frequency_hz":JOINT_TRAJ_CYCLE_FREQUENCY_HZ,},
        ]
    )

    meca_joint_traj = Node(
        package="esame",
        executable="MecaJointTraj",
        output="screen",
        parameters=[
            {"cycle_frequency_hz":JOINT_TRAJ_CYCLE_FREQUENCY_HZ,},
        ]
    )

    distance_calculator = Node(
        package="esame",
        executable="DistanceCalculator",
        output="screen",
        parameters=[
            {"distance_topic" : DISTANCE_TOPIC,
            "ee_frames" : EE_FRAMES,
            "cycle_time_micros" : DISTANCE_CYCLE_TIME_MICROS}
        ]
    )

    return LaunchDescription([
        included_launch,
        panda_joint_traj,
        meca_joint_traj,
        distance_calculator,
        main,
        ])