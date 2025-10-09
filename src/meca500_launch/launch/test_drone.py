import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

CYCLE_FREQUENCY_HZ = 100


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
            "camera_update_rate": LaunchConfiguration("camera_update_rate"),
            "table_size_x": str(0.2),
            "table_size_y": str(0.2),
            "table_height": str(0.5),
            "randomize": "true",
            "randomize_z": "false",
            "x_min": str(-1.0),
            "x_max": str(5.0),
            "y_min": str(-2.0),
            "y_max": str(2.0),
            "z_min": str(0.5),
            "z_max": str(1.5),
            "actor_speed": str(0.8),
            "actor_mesh": "drone",
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
                "cycle_frequency_hz": CYCLE_FREQUENCY_HZ,
                "k_p": 2e0,
                "k_roll": 1e1,
                "k_limit": 0.01,
                "limit_threshold_deg": 10.0,
                "joint_limits_lower": [-175.0, -70.0, -135.0, -170.0, -115.0, -360.0],
                "joint_limits_upper": [175.0, 90.0, 70.0, 170.0, 115.0, 360.0],
                # "joint_limits_lower": [-30.0, -70.0, -135.0, -170.0, -115.0, -36000.0],
                # "joint_limits_upper": [30.0, 90.0, 70.0, 170.0, 115.0, 36000.0],
            }
        ],
    )

    frame_publisher = Node(
        package="meca500_utils",
        executable="frame_publisher",
        output="screen",
        parameters=[
            {
                "cycle_frequency_hz": CYCLE_FREQUENCY_HZ,
            }
        ],
    )

    move_joint_pose = Node(
        package="meca500_control",
        executable="move_joint_pose",
        output="screen",
        parameters=[
            {
                "cycle_frequency_hz": CYCLE_FREQUENCY_HZ,
                "k_p": 5e-1,
                "joint_norm_tolerance": 0.05,
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

    target_pose_filter = Node(
        package="meca500_filtering",
        executable="target_pose_filter",
        output="screen",
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
                "cycle_frequency_hz": CYCLE_FREQUENCY_HZ,
                "state_delay_sec": 0.0,
                "initial_position": [1.57, 0.0, 0.0, 0.0, 0.0, 0.0],
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
            target_pose_filter,
        ]
    )
