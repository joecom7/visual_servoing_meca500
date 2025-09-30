import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    meca500_world_dir = get_package_share_directory("meca500_world")

    # Declare the camera_update_rate argument so it can be overridden from CLI
    declare_camera_update_rate = DeclareLaunchArgument(
        "camera_update_rate",
        default_value="60.0",
        description="Update rate of the camera in Hz",
    )

    # Declare the performance_mode argument for YOLO
    declare_performance_mode = DeclareLaunchArgument(
        "performance_mode",
        default_value="high",
        description="YOLO performance mode: low, medium, high",
    )

    # Pass the launch argument to the sim.launch.py file
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(meca500_world_dir, "launch", "sim.launch.py")
        ),
        launch_arguments={
            "camera_update_rate": LaunchConfiguration("camera_update_rate")
        }.items(),
    )

    meca500_moveit_config_dir = get_package_share_directory("meca500_moveit_config")

    # Pass the launch argument to the sim.launch.py file
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(meca500_moveit_config_dir, "launch", "move_group.launch.py")
        )
    )

    meca500_control = Node(
        package="meca500_control",
        executable="ibvs_dumb",
        output="screen",
        parameters=[
            {
                "cycle_frequency_hz": 100,
                "k_p": 2e0,
                "k_roll": 1e+1,
            }
        ],
    )

    frame_publisher = Node(
        package="meca500_utils",
        executable="frame_publisher",
        output="screen",
        parameters=[
            {
                "cycle_frequency_hz": 1000,
            }
        ],
    )

    move_joint_pose = Node(
        package="meca500_control",
        executable="move_joint_pose",
        output="screen",
        parameters=[
            {
                "cycle_frequency_hz": 1000,
                "k_p": 1e+0,
                "joint_norm_tolerance" : 0.05,
            }
        ],
    )

    meca500_vision = Node(
        package="meca500_vision",
        executable="image_listener",
        output="screen",
        parameters=[
            {
                "performance_mode": LaunchConfiguration("performance_mode"),
                "image_width_pixels": 1280,
                "image_height_pixels": 720,
            }
        ],
    )

    jacobian_calculator = Node(
        package="meca500_utils", executable="jacobian_calculator", output="screen"
    )

    image_jacobian_calculator = Node(
        package="meca500_utils",
        executable="image_jacobian_calculator",
        output="screen",
        parameters=[
            {
                "horizontal_fov_degrees": 85.2,
                "image_width_pixels": 1280,
            }
        ],
    )

    home_and_follow = Node(
        package="meca500_launch",
        executable="home_and_follow",
        output="screen",
        parameters=[
            {
                "cycle_frequency_hz": 1000,
                "state_delay_sec": 0.0,
                "initial_position": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                "home_position": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            }
        ],
    )

    return LaunchDescription(
        [
            declare_camera_update_rate,
            declare_performance_mode,
            meca500_control,
            sim_launch,
            meca500_vision,
            jacobian_calculator,
            move_group_launch,
            image_jacobian_calculator,
            frame_publisher,
            home_and_follow,
            move_joint_pose,
        ]
    )
