from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch Gazebo simulation with environment variable
        ExecuteProcess(
            #cmd=['env', 'LIBGL_ALWAYS_SOFTWARE=1', 'gz', 'sim', 'waves_tether.sdf'],
            #cmd=['env', 'LIBGL_ALWAYS_SOFTWARE=1', 'gz', 'sim', 'waves_tether_dev.sdf'],
            #cmd=['env', 'LIBGL_ALWAYS_SOFTWARE=1', 'gz', 'sim', 'waves_tether_mission.sdf'],
            #cmd=['env', 'LIBGL_ALWAYS_SOFTWARE=1', 'gz', 'sim', 'waves_tether_mission_bop3.sdf'],
            cmd=['env', 'LIBGL_ALWAYS_SOFTWARE=1', 'gz', 'sim', 'rov_alpha.sdf'],
            #cmd=['env', 'LIBGL_ALWAYS_SOFTWARE=1', 'gz', 'sim', 'waves_tether_mission_bop10.sdf'],
            #cmd=['env', 'LIBGL_ALWAYS_SOFTWARE=1', 'gz', 'sim', 'waves_tether_mission_bop10_free.sdf'],
            #cmd=['env', 'LIBGL_ALWAYS_SOFTWARE=1', 'gz', 'sim', 'test.sdf'],
            output='screen'
        ),
        
        TimerAction(
            period=5.0,  # Wait for 5 seconds before running wmctrl
            actions=[
                ExecuteProcess(
                    cmd=['wmctrl', '-r', 'Gazebo', '-b', 'add,above'],
                    output='screen'
                )
            ]
        ),

        # Launch the Gazebo-ROS bridge
        Node(
            package='ros_ign_bridge',
            executable='parameter_bridge',
            arguments=[
                '/model/blueboat/joint/motor_port_joint/cmd_thrust@std_msgs/msg/Float64@ignition.msgs.Double',
                '/model/blueboat/joint/motor_stbd_joint/cmd_thrust@std_msgs/msg/Float64@ignition.msgs.Double',
                '/model/blueboat/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
                '/navsat@sensor_msgs/msg/NavSatFix@ignition.msgs.NavSat'
            ],
            output='screen'
        ),
        
        

        # Optionally, launch your ROS 2 node if you have a custom node for additional logic
        # Node(
        #     package='move_blueboat',
        #     executable='robot_controller',
        #     output='screen',
        #     name='robot_controller'
        # ),
    ])

