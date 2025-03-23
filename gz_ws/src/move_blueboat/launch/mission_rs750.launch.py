from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch Gazebo simulation with environment variable
        ExecuteProcess(
            #cmd=['env', 'LIBGL_ALWAYS_SOFTWARE=1', 'gz', 'sim', 'wavetank_tether_5.sdf'],
           # cmd=['env', 'LIBGL_ALWAYS_SOFTWARE=1', 'gz', 'sim', 'wavetank_tether_3.sdf'], ##for line
            cmd=['env', 'LIBGL_ALWAYS_SOFTWARE=1', 'gz', 'sim', 'waves_mission_rs750.sdf'], ##for virtual obstacle
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
#/world/waves/model/rs750/link/base_link/sensor/navsat_sensor/navsat
#blueboat_sitl@markus:~/ardupilot$ gz topic -i -t /world/waves/model/rs750/link/base_link/sensor/navsat_sensor/navsat

 # tcp://127.0.0.1:33665, gz.msgs.NavSat
        # Launch the Gazebo-ROS bridge
        Node(
            package='ros_ign_bridge',
            executable='parameter_bridge',
            arguments=[
                '/main_sail_joint/cmd_pos@std_msgs/msg/Float64@ignition.msgs.Double',
                '/fore_sail_joint/cmd_pos@std_msgs/msg/Float64@ignition.msgs.Double',
                '/rudder_joint/cmd_pos@std_msgs/msg/Float64@ignition.msgs.Double',
                '/world/waves/model/rs750/link/base_link/sensor/navsat_sensor/navsat@sensor_msgs/msg/NavSatFix@ignition.msgs.NavSat',
                '/world/waves/model/rs750/link/base_link/sensor/anemometer/anemometer@geometry_msgs/msg/Vector3@ignition.msgs.Vector3d',
                '/world/waves/model/rs750/link/base_link/sensor/imu_sensor/imu@sensor_msgs/msg/Imu@ignition.msgs.IMU'
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

