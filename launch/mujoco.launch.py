from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    goal_x_arg = DeclareLaunchArgument('goal_x', default_value='8.0')
    goal_y_arg = DeclareLaunchArgument('goal_y', default_value='8.0')
    start_x_arg = DeclareLaunchArgument('start_x', default_value='5.54')
    start_y_arg = DeclareLaunchArgument('start_y', default_value='5.54')
    use_wp_arg = DeclareLaunchArgument('use_waypoints', default_value='false')
    loop_arg = DeclareLaunchArgument('loop_waypoints', default_value='false')

    bridge = Node(
        package='turtle_pid',
        executable='mujoco_bridge',
        name='mujoco_bridge',
        output='screen',
        parameters=[{
            'start_x': ParameterValue(LaunchConfiguration('start_x'), value_type=float),
            'start_y': ParameterValue(LaunchConfiguration('start_y'), value_type=float),
        }],
    )

    pid_node = Node(
        package='turtle_pid',
        executable='pid_controller',
        name='pid_controller',
        output='screen',
        parameters=[{
            'goal_x': ParameterValue(LaunchConfiguration('goal_x'), value_type=float),
            'goal_y': ParameterValue(LaunchConfiguration('goal_y'), value_type=float),
            'use_waypoints': ParameterValue(LaunchConfiguration('use_waypoints'), value_type=bool),
            'loop_waypoints': ParameterValue(LaunchConfiguration('loop_waypoints'), value_type=bool),
        }],
    )

    return LaunchDescription([goal_x_arg, goal_y_arg, start_x_arg, start_y_arg, use_wp_arg, loop_arg, bridge, pid_node])