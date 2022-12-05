from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression

from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()

    robot_name_arg = DeclareLaunchArgument('robot_name',
                                           default_value='flying_arm_2',
                                           description='Name of the robot controlled by the ODRI master board.')
    ld.add_action(robot_name_arg)

    default_yaml_path = PathJoinSubstitution([
        FindPackageShare('odri_interface'), 'config',
        PythonExpression(["'", LaunchConfiguration('robot_name'), "'+ '.yaml'"])
    ])

    yaml_path_arg = DeclareLaunchArgument('yaml_path',
                                          default_value=default_yaml_path,
                                          description='Path of the yaml file with state publisher node parameters')
    ld.add_action(yaml_path_arg)

    remappings = [('robot_state', '/odri/robot_state')]

    node = Node(package='odri_interface',
                name='robot_interface',
                executable='robot_interface',
                output='screen',
                emulate_tty=True,
                parameters=[LaunchConfiguration('yaml_path')],
                remappings=remappings)

    ld.add_action(node)

    return ld