import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_meca500_world = get_package_share_directory('meca500_world')

    # Parameters
    world = LaunchConfiguration('world')
    ball_name = LaunchConfiguration('ball_name')
    x = LaunchConfiguration('x', default='0.5')  # in front of robot
    y = LaunchConfiguration('y', default='0.0')
    z = LaunchConfiguration('z', default='0.05')  # radius of the ball

    # Declare launch arguments
    declare_world_cmd = DeclareLaunchArgument(
        'world', default_value=TextSubstitution(text='meca500_world'),
        description='World name')
    declare_ball_name_cmd = DeclareLaunchArgument(
        'ball_name', default_value=TextSubstitution(text='red_ball'),
        description='Name of the ball entity')
    declare_x_cmd = DeclareLaunchArgument('x', default_value='0.5')
    declare_y_cmd = DeclareLaunchArgument('y', default_value='0.0')
    declare_z_cmd = DeclareLaunchArgument('z', default_value='0.05')

    # Spawn node (ros_gz_sim create)
    spawn_ball_node = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        parameters=[{
            'world': world,
            'name': ball_name,
            'allow_renaming': False,
            'x': x,
            'y': y,
            'z': z,
            'R': 0.0,
            'P': 0.0,
            'Y': 0.0,
            'file': '',
            'string': f"""
<sdf version='1.7'>
  <model name='target'>
  <static>true</static>
    <link name='ball_link'>
      <pose>0 0 0 0 0 0</pose>
      <collision name='collision'>
        <geometry>
          <sphere>
            <radius>0.05</radius>
          </sphere>
        </geometry>
      </collision>
      <visual name='visual'>
        <geometry>
          <sphere>
            <radius>0.05</radius>
          </sphere>
        </geometry>
        <material>
          <ambient>1 0 0 1</ambient>
          <diffuse>1 0 0 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""
        }]
    )

    # Build launch description
    ld = LaunchDescription()
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_ball_name_cmd)
    ld.add_action(declare_x_cmd)
    ld.add_action(declare_y_cmd)
    ld.add_action(declare_z_cmd)
    ld.add_action(spawn_ball_node)

    return ld
