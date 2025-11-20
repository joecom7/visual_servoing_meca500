# Copyright 2024 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import math
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import FileContent, LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
import xacro
import random


def generate_fixed_trajectory():
    """Generate the fixed predefined trajectory."""
    waypoints = [
        {"x": 3.5, "y": -1, "z": 1.0, "roll": 0, "pitch": 0, "yaw": 0.0},
        {"x": 5.5, "y": -1, "z": 1.0, "roll": 0, "pitch": 0, "yaw": 0.0},
        {"x": 5.5, "y": -1, "z": 1.0, "roll": 0, "pitch": 0, "yaw": 1.57},
        {"x": 5.5, "y": 1, "z": 1.0, "roll": 0, "pitch": 0, "yaw": 1.57},
        {"x": 5.5, "y": 1, "z": 1.0, "roll": 0, "pitch": 0, "yaw": 3.142},
        {"x": 3.5, "y": 1, "z": 1.0, "roll": 0, "pitch": 0, "yaw": 3.142},
        {"x": 3.5, "y": 1, "z": 1.0, "roll": 0, "pitch": 0, "yaw": -1.57},
        {"x": 3.5, "y": -1, "z": 1.0, "roll": 0, "pitch": 0, "yaw": -1.57},
        {"x": 3.5, "y": -1, "z": 1.0, "roll": 0, "pitch": 0, "yaw": 0.0},
    ]
    times = [0, 2, 2.5, 4, 4.5, 6, 6.5, 8, 8.5]
    return waypoints, times


def generate_random_waypoints(
    n=9, 
    x_min=1.0, 
    x_max=5.0, 
    y_min=-5.0, 
    y_max=5.0, 
    z_min=0.5, 
    z_max=1.5,
    randomize_z=False,
    speed=1.0
):
    """Generate n random waypoints with timestamps and yaw along the path.
    
    Args:
        n: number of waypoints
        x_min, x_max: bounds for x coordinate
        y_min, y_max: bounds for y coordinate
        z_min, z_max: bounds for z coordinate (used only if randomize_z=True)
        randomize_z: if True, randomize z values; if False, use fixed z=1.0
        speed: constant speed in m/s for trajectory
    """
    waypoints = []
    
    # First pass: generate all waypoint positions
    for i in range(n):
        if i == 0:
            x = (x_min + x_max) / 2  # Start at middle of x range
            y = y_min
            z = 1.0 if not randomize_z else round(random.uniform(z_min, z_max), 2)
        elif i == n - 1:
            # last waypoint = first (to close the loop)
            x, y, z = waypoints[0]["x"], waypoints[0]["y"], waypoints[0]["z"]
        else:
            x = round(random.uniform(x_min, x_max), 2)
            y = round(random.uniform(y_min, y_max), 2)
            z = 1.0 if not randomize_z else round(random.uniform(z_min, z_max), 2)

        waypoints.append({"x": x, "y": y, "z": z, "roll": 0, "pitch": 0, "yaw": 0.0})

    # Second pass: calculate yaw angles based on actual next waypoint direction
    for i in range(len(waypoints)):
        if i < len(waypoints) - 1:
            dx = waypoints[i + 1]["x"] - waypoints[i]["x"]
            dy = waypoints[i + 1]["y"] - waypoints[i]["y"]
            yaw = math.atan2(dy, dx)
            waypoints[i]["yaw"] = yaw
        else:
            # Last waypoint points towards first
            waypoints[i]["yaw"] = waypoints[0]["yaw"]

    # Third pass: calculate times based on distance and constant speed
    times = []
    total_time = 0.0
    
    for i in range(len(waypoints)):
        times.append(round(total_time, 2))
        
        if i < len(waypoints) - 1:
            dx = waypoints[i + 1]["x"] - waypoints[i]["x"]
            dy = waypoints[i + 1]["y"] - waypoints[i]["y"]
            dz = waypoints[i + 1]["z"] - waypoints[i]["z"]
            dist = math.sqrt(dx * dx + dy * dy + dz * dz)
            dt = dist / speed  # time = distance / speed
            total_time += dt

    return waypoints, times


def launch_sim(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    pkg_meca500_world = get_package_share_directory("meca500_world")

    # Path del file xacro
    xacro_file = os.path.join(pkg_meca500_world, "models", "meca500", "meca500.xacro")
    # Path del file URDF generato
    urdf_file = os.path.join(pkg_meca500_world, "models", "meca500", "meca500.urdf")

    # Parametri da launch
    camera_update_rate = float(
        LaunchConfiguration("camera_update_rate").perform(context)
    )
    table_height = float(LaunchConfiguration("table_height").perform(context))
    table_thickness = float(LaunchConfiguration("table_thickness").perform(context))
    meca_offset_x = float(LaunchConfiguration("meca_offset_x").perform(context))
    meca_offset_y = float(LaunchConfiguration("meca_offset_y").perform(context))

    # Trajectory parameters
    randomize = LaunchConfiguration("randomize").perform(context).lower() == "true"
    randomize_z = LaunchConfiguration("randomize_z").perform(context).lower() == "true"
    actor_mesh = LaunchConfiguration("actor_mesh").perform(context)
    actor_speed = float(LaunchConfiguration("actor_speed").perform(context))
    x_min = float(LaunchConfiguration("x_min").perform(context))
    x_max = float(LaunchConfiguration("x_max").perform(context))
    y_min = float(LaunchConfiguration("y_min").perform(context))
    y_max = float(LaunchConfiguration("y_max").perform(context))
    z_min = float(LaunchConfiguration("z_min").perform(context))
    z_max = float(LaunchConfiguration("z_max").perform(context))

    # Genero l'URDF a runtime e lo scrivo su file
    urdf_string = xacro.process_file(
        xacro_file,
        mappings={
            "camera_update_rate": str(camera_update_rate),
            "use_package_reference": str(False),
            "table_height": str(table_height),
            "table_thickness": str(table_thickness),
            "meca_offset_x": str(meca_offset_x),
            "meca_offset_y": str(meca_offset_y),
        },
    ).toxml()

    with open(urdf_file, "w") as f:
        f.write(urdf_string)
    print(f"[INFO] URDF scritto su disco: {urdf_file}")

    urdf = FileContent(urdf_file)

    xacro_file = os.path.join(pkg_meca500_world, "worlds", "empty_world.xacro")
    sdf_file = os.path.join(pkg_meca500_world, "worlds", "empty_world.sdf")

    # Generate waypoints based on randomize flag
    if randomize:
        waypoints, times = generate_random_waypoints(
            n=9,
            x_min=x_min,
            x_max=x_max,
            y_min=y_min,
            y_max=y_max,
            z_min=z_min,
            z_max=z_max,
            randomize_z=randomize_z,
            speed=actor_speed
        )
        print(f"[INFO] Generated random trajectory with {len(waypoints)} waypoints at {actor_speed} m/s")
    else:
        waypoints, times = generate_fixed_trajectory()
        print(f"[INFO] Using fixed predefined trajectory")

    # Prepare mapping for xacro args
    mapping = {}
    for i, wp in enumerate(waypoints):
        idx = i + 1
        mapping[f"wp{idx}"] = (
            f"{wp['x']} {wp['y']} {wp['z']} {wp['roll']} {wp['pitch']} {wp['yaw']}"
        )
        mapping[f"t{idx}"] = str(times[i])
    mapping["actor_mesh"] = actor_mesh

    sdf_string = xacro.process_file(xacro_file, mappings=mapping).toxml()
    with open(sdf_file, "w") as f:
        f.write(sdf_string)
    print(f"[INFO] SDF generated at {sdf_file}")

    DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )

    robot_state = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time, "robot_description": urdf}],
        arguments=[urdf],
    )

    meca500_fp3_model = PathJoinSubstitution(
        [pkg_meca500_world, "models", "meca500", "meca500.urdf"]
    )

    # Setup to launch the simulator and Gazebo world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": PathJoinSubstitution(
                [pkg_meca500_world, "worlds", "empty_world.sdf"]
            )
        }.items(),
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[
            {
                "config_file": os.path.join(pkg_meca500_world, "config", "bridge.yaml"),
                "qos_overrides./tf_static.publisher.durability": "transient_local",
            }
        ],
        output="screen",
    )

    world = LaunchConfiguration("world")
    file = LaunchConfiguration("file")
    model_string = LaunchConfiguration("model_string")
    topic = LaunchConfiguration("topic")
    entity_name = LaunchConfiguration("entity_name")
    allow_renaming = LaunchConfiguration("allow_renaming")
    x = LaunchConfiguration("x", default="0.0")
    y = LaunchConfiguration("y", default="0.0")
    z = LaunchConfiguration("z", default="0.0")
    roll = LaunchConfiguration("R", default="0.0")
    pitch = LaunchConfiguration("P", default="0.0")
    yaw = LaunchConfiguration("Y", default="0.0")

    declare_world_cmd = DeclareLaunchArgument(
        "world",
        default_value=TextSubstitution(text="meca500_world"),
        description="World name",
    )

    declare_file_cmd = DeclareLaunchArgument(
        "file",
        default_value=meca500_fp3_model,
        description="SDF/URDF filename of model",
    )

    declare_model_string_cmd = DeclareLaunchArgument(
        "model_string", default_value="", description="XML(SDF) string"
    )

    declare_topic_cmd = DeclareLaunchArgument(
        "topic",
        default_value=TextSubstitution(text=""),
        description="Get XML from this topic",
    )

    declare_entity_name_cmd = DeclareLaunchArgument(
        "entity_name",
        default_value=TextSubstitution(text="meca500"),
        description="Name of the entity",
    )

    declare_allow_renaming_cmd = DeclareLaunchArgument(
        "allow_renaming",
        default_value="False",
        description="Whether the entity allows renaming or not",
    )

    load_nodes = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        parameters=[
            {
                "world": world,
                "file": file,
                "string": model_string,
                "topic": topic,
                "name": entity_name,
                "allow_renaming": allow_renaming,
                "x": x,
                "y": y,
                "z": z,
                "R": roll,
                "P": pitch,
                "Y": yaw,
            }
        ],
    )

    joint_velocity_bridge = Node(
        package="meca500_world", executable="joint_velocity_bridge", output="screen"
    )

    return [
        declare_world_cmd,
        declare_file_cmd,
        declare_model_string_cmd,
        declare_topic_cmd,
        declare_entity_name_cmd,
        declare_allow_renaming_cmd,
        gz_sim,
        load_nodes,
        robot_state,
        bridge,
        joint_velocity_bridge,
    ]


def generate_launch_description():
    # Declare launch arguments
    declare_camera_update_rate = DeclareLaunchArgument(
        "camera_update_rate", default_value="60.0"
    )

    declare_table_height = DeclareLaunchArgument(
        "table_height", default_value="2.0", description="Altezza del tavolo"
    )

    declare_table_thickness = DeclareLaunchArgument(
        "table_thickness",
        default_value="0.05",
        description="Spessore del piano del tavolo",
    )

    declare_meca_offset_x = DeclareLaunchArgument(
        "meca_offset_x",
        default_value="0.0",
        description="Offset X del Meca sul tavolo",
    )

    declare_meca_offset_y = DeclareLaunchArgument(
        "meca_offset_y",
        default_value="0.0",
        description="Offset Y del Meca sul tavolo",
    )

    # Trajectory configuration arguments
    declare_randomize = DeclareLaunchArgument(
        "randomize",
        default_value="false",
        description="Use random trajectory (true) or fixed predefined trajectory (false)",
    )

    declare_randomize_z = DeclareLaunchArgument(
        "randomize_z",
        default_value="false",
        description="Randomize z coordinate for random trajectory (only used if randomize=true)",
    )

    declare_actor_mesh = DeclareLaunchArgument(
        "actor_mesh",
        default_value="drone",
        description="Mesh name for the actor",
    )

    declare_actor_speed = DeclareLaunchArgument(
        "actor_speed",
        default_value="1.0",
        description="Actor movement speed in m/s (only used for random trajectories)",
    )

    declare_x_min = DeclareLaunchArgument(
        "x_min",
        default_value="1.0",
        description="Minimum x coordinate for random trajectory",
    )

    declare_x_max = DeclareLaunchArgument(
        "x_max",
        default_value="5.0",
        description="Maximum x coordinate for random trajectory",
    )

    declare_y_min = DeclareLaunchArgument(
        "y_min",
        default_value="-5.0",
        description="Minimum y coordinate for random trajectory",
    )

    declare_y_max = DeclareLaunchArgument(
        "y_max",
        default_value="5.0",
        description="Maximum y coordinate for random trajectory",
    )

    declare_z_min = DeclareLaunchArgument(
        "z_min",
        default_value="0.5",
        description="Minimum z coordinate for random trajectory (only used if randomize_z=true)",
    )

    declare_z_max = DeclareLaunchArgument(
        "z_max",
        default_value="1.5",
        description="Maximum z coordinate for random trajectory (only used if randomize_z=true)",
    )

    return LaunchDescription(
        [
            declare_camera_update_rate,
            declare_table_height,
            declare_table_thickness,
            declare_meca_offset_x,
            declare_meca_offset_y,
            declare_randomize,
            declare_randomize_z,
            declare_actor_mesh,
            declare_actor_speed,
            declare_x_min,
            declare_x_max,
            declare_y_min,
            declare_y_max,
            declare_z_min,
            declare_z_max,
            OpaqueFunction(function=launch_sim),
        ]
    )