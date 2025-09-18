from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def spawn_table(context, *args, **kwargs):
    # Get args from launch
    world = LaunchConfiguration('world').perform(context)
    table_name = LaunchConfiguration('table_name').perform(context)
    x = float(LaunchConfiguration('x').perform(context))
    y = float(LaunchConfiguration('y').perform(context))
    height = float(LaunchConfiguration('height').perform(context))
    width = float(LaunchConfiguration('width').perform(context))
    depth = float(LaunchConfiguration('depth').perform(context))
    top_thickness = float(LaunchConfiguration('top_thickness').perform(context))
    leg_thickness = float(LaunchConfiguration('leg_thickness').perform(context))

    # Derived values
    top_z = height - top_thickness / 2.0
    leg_height = height - top_thickness
    leg_z = leg_height / 2.0
    dx = width / 2.0 - leg_thickness / 2.0
    dy = depth / 2.0 - leg_thickness / 2.0

    # Build SDF with numbers already substituted
    sdf = f"""
<sdf version='1.7'>
  <model name='{table_name}'>
    <static>true</static>
    <link name='table_link'>

      <!-- Table top -->
      <visual name='top_visual'>
        <pose>0 0 {top_z} 0 0 0</pose>
        <geometry><box><size>{width} {depth} {top_thickness}</size></box></geometry>
        <material><ambient>0.6 0.3 0.1 1</ambient><diffuse>0.6 0.3 0.1 1</diffuse></material>
      </visual>
      <collision name='top_collision'>
        <pose>0 0 {top_z} 0 0 0</pose>
        <geometry><box><size>{width} {depth} {top_thickness}</size></box></geometry>
      </collision>

      <!-- Legs -->
      <visual name='leg1_visual'>
        <pose>{dx} {dy} {leg_z} 0 0 0</pose>
        <geometry><box><size>{leg_thickness} {leg_thickness} {leg_height}</size></box></geometry>
      </visual>
      <collision name='leg1_collision'>
        <pose>{dx} {dy} {leg_z} 0 0 0</pose>
        <geometry><box><size>{leg_thickness} {leg_thickness} {leg_height}</size></box></geometry>
      </collision>

      <visual name='leg2_visual'>
        <pose>{-dx} {dy} {leg_z} 0 0 0</pose>
        <geometry><box><size>{leg_thickness} {leg_thickness} {leg_height}</size></box></geometry>
      </visual>
      <collision name='leg2_collision'>
        <pose>{-dx} {dy} {leg_z} 0 0 0</pose>
        <geometry><box><size>{leg_thickness} {leg_thickness} {leg_height}</size></box></geometry>
      </collision>

      <visual name='leg3_visual'>
        <pose>{dx} {-dy} {leg_z} 0 0 0</pose>
        <geometry><box><size>{leg_thickness} {leg_thickness} {leg_height}</size></box></geometry>
      </visual>
      <collision name='leg3_collision'>
        <pose>{dx} {-dy} {leg_z} 0 0 0</pose>
        <geometry><box><size>{leg_thickness} {leg_thickness} {leg_height}</size></box></geometry>
      </collision>

      <visual name='leg4_visual'>
        <pose>{-dx} {-dy} {leg_z} 0 0 0</pose>
        <geometry><box><size>{leg_thickness} {leg_thickness} {leg_height}</size></box></geometry>
      </visual>
      <collision name='leg4_collision'>
        <pose>{-dx} {-dy} {leg_z} 0 0 0</pose>
        <geometry><box><size>{leg_thickness} {leg_thickness} {leg_height}</size></box></geometry>
      </collision>

    </link>
  </model>
</sdf>
"""

    # Node to spawn the table
    return [
        Node(
            package='ros_gz_sim',
            executable='create',
            output='screen',
            parameters=[{
                'world': world,
                'name': table_name,
                'allow_renaming': False,
                'x': x,
                'y': y,
                'z': 0.0,  # place so table sits on ground
                'R': 0.0,
                'P': 0.0,
                'Y': 0.0,
                'file': '',
                'string': sdf
            }]
        )
    ]


def generate_launch_description():
    # Declare launch arguments
    declare_world_cmd = DeclareLaunchArgument(
        'world', default_value=TextSubstitution(text='meca500_world'))
    declare_table_name_cmd = DeclareLaunchArgument(
        'table_name', default_value=TextSubstitution(text='table'))
    declare_x_cmd = DeclareLaunchArgument('x', default_value='0.0')
    declare_y_cmd = DeclareLaunchArgument('y', default_value='0.0')
    declare_h_cmd = DeclareLaunchArgument('height', default_value='0.75')
    declare_w_cmd = DeclareLaunchArgument('width', default_value='1.0')
    declare_d_cmd = DeclareLaunchArgument('depth', default_value='0.6')
    declare_top_cmd = DeclareLaunchArgument('top_thickness', default_value='0.05')
    declare_leg_cmd = DeclareLaunchArgument('leg_thickness', default_value='0.05')

    return LaunchDescription([
        declare_world_cmd,
        declare_table_name_cmd,
        declare_x_cmd,
        declare_y_cmd,
        declare_h_cmd,
        declare_w_cmd,
        declare_d_cmd,
        declare_top_cmd,
        declare_leg_cmd,
        OpaqueFunction(function=spawn_table)
    ])
