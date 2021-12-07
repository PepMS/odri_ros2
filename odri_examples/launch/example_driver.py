import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    odri_iface_config = os.path.join(get_package_share_directory('odri_interface'), 'config', 'params.yaml')

    odri_iface_node = Node(package='odri_interface',
                           name='master_board_wrapper',
                           executable='master_board_wrapper',
                           parameters=[odri_iface_config],
                           remappings=[("master_board_state", "/odri/master_board_state"),
                                       ("motor_commands", "/odri/motor_commands")])

    odri_example_node = Node(package='odri_examples',
                             name='example_driver',
                             executable='example_driver',
                             remappings=[("master_board_state", "/odri/master_board_state"),
                                         ("motor_commands", "/odri/motor_commands")])

    ld.add_action(odri_iface_node)
    ld.add_action(odri_example_node)

    return ld