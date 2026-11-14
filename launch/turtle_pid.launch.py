from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # 1. Declare the arguments people can set, with defaults
    goal_x_arg = DeclareLaunchArgument(
        'goal_x', default_value='8.0', description='Goal x position (0.5 to 10.58)')
    goal_y_arg = DeclareLaunchArgument(
        'goal_y', default_value='8.0', description='Goal y position (0.5 to 10.58)')

    # 2. Start turtlesim
    turtlesim_node = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtlesim',
    )

    # 3. Start the controller, passing the arguments in as parameters
    pid_node = Node(
        package='turtle_pid',
        executable='pid_controller',
        name='pid_controller',
        output='screen',
        parameters=[{
            'goal_x': ParameterValue(LaunchConfiguration('goal_x'), value_type=float),
            'goal_y': ParameterValue(LaunchConfiguration('goal_y'), value_type=float),
            'kp_angular': 6.0,
        }],
    )

    return LaunchDescription([goal_x_arg, goal_y_arg, turtlesim_node, pid_node])