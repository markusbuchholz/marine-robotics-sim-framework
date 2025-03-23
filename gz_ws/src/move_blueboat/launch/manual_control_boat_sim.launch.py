# Copyright 2024, Markus Buchholz

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    joy_dev_argument = DeclareLaunchArgument(
        'joy_dev',
        default_value='/dev/input/js0',
        description='Joystick device'
    )

    joystick_connected_argument = DeclareLaunchArgument(
        'joystick_connected',
        default_value='true',
        description='Is the joystick physically connected?'
    )

    joy_node_linux = Node(
        package='joy_linux',  
        executable='joy_linux_node',  
        name='joystick',
        parameters=[{'dev': LaunchConfiguration('joy_dev'), 'autorepeat_rate': 10.0}],
        condition=IfCondition(LaunchConfiguration('joystick_connected'))
    )

    joy_node = Node(
        package="move_blueboat",
        executable="joy_sim_controler",
        name="joy_node",
        output="screen",
    )
    

    return LaunchDescription([

        joy_dev_argument,
        joystick_connected_argument,
        joy_node_linux,
        joy_node,
     

    ])