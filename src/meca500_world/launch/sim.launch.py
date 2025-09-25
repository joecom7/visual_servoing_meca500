# Copyright 2024 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing,
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

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


def launch_sim(context, *args, **kwargs):

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    pkg_meca500_world = get_package_share_directory("meca500_world")

    # Path del file xacro
    xacro_file = os.path.join(pkg_meca500_world, "models", "meca500", "meca500.xacro")
    # Path del file URDF generato
    urdf_file = os.path.join(pkg_meca500_world, "models", "meca500", "meca500.urdf")

    # Parametri da launch
    camera_update_rate = float(LaunchConfiguration('camera_update_rate').perform(context))
    table_height = float(LaunchConfiguration("table_height").perform(context))
    table_thickness = float(LaunchConfiguration("table_thickness").perform(context))
    meca_offset_x = float(LaunchConfiguration("meca_offset_x").perform(context))
    meca_offset_y = float(LaunchConfiguration("meca_offset_y").perform(context))

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
        }
    ).toxml()

    with open(urdf_file, "w") as f:
        f.write(urdf_string)
        print(f"[INFO] URDF scritto su disco: {urdf_file}")

    urdf = FileContent(urdf_file)

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
        "model_string",
        default_value="",
        description="XML(SDF) string",
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
        package="meca500_world",
        executable="joint_velocity_bridge",
        output="screen"
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
        joint_velocity_bridge
    ]


def generate_launch_description():

    # Declare launch arguments
    declare_camera_update_rate = DeclareLaunchArgument(
        "camera_update_rate", default_value="60.0"
    )
    declare_table_height = DeclareLaunchArgument(
        "table_height", default_value="0.75", description="Altezza del tavolo"
    )
    declare_table_thickness = DeclareLaunchArgument(
        "table_thickness", default_value="0.05", description="Spessore del piano del tavolo"
    )
    declare_meca_offset_x = DeclareLaunchArgument(
        "meca_offset_x", default_value="0.0", description="Offset X del Meca sul tavolo"
    )
    declare_meca_offset_y = DeclareLaunchArgument(
        "meca_offset_y", default_value="0.0", description="Offset Y del Meca sul tavolo"
    )

    return LaunchDescription(
        [
            declare_camera_update_rate,
            declare_table_height,
            declare_table_thickness,
            declare_meca_offset_x,
            declare_meca_offset_y,
            OpaqueFunction(function=launch_sim),
        ]
    )
