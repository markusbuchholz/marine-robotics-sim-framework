import os
import subprocess
from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration

def generate_launch_description():
    sdf_file_path = '/home/blueboat_sitl/SITL_Models/Gazebo/models/blueboat/model.sdf'
    
    # Convert SDF to URDF
    urdf_content = subprocess.check_output(['gz', 'sdf', '-p', sdf_file_path]).decode('utf-8')
    
    return LaunchDescription([
        # Log information
        LogInfo(msg="Starting Gazebo simulation with environment variable"),
        
        # Launch Gazebo simulation with environment variable
        ExecuteProcess(
            cmd=['env', 'LIBGL_ALWAYS_SOFTWARE=1', 'gz', 'sim', 'waves.sdf'],
            output='screen'
        ),
        
        LogInfo(msg="Launching Gazebo-ROS bridge"),
        
        # Launch the Gazebo-ROS bridge
        Node(
            package='ros_ign_bridge',
            executable='parameter_bridge',
            arguments=[
                '/model/blueboat/joint/motor_port_joint/cmd_thrust@std_msgs/msg/Float64@ignition.msgs.Double',
                '/model/blueboat/joint/motor_stbd_joint/cmd_thrust@std_msgs/msg/Float64@ignition.msgs.Double',
                '/model/blueboat/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
                '/navsat@sensor_msgs/msg/NavSatFix@ignition.msgs.NavSat',
                '/world/waves/model/blueboat/joint_state@sensor_msgs/msg/JointState@ignition.msgs.Model'
            ],
            output='screen'
        ),
        
        LogInfo(msg="Publishing URDF to /robot_description topic"),
        
        # Publish the URDF to the /robot_description topic
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': urdf_content}]
        ),
        
        LogInfo(msg="Launching joint_state_publisher"),
        
        # Launch the joint_state_publisher if needed
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        ),
        
        LogInfo(msg="Launching RViz with default configuration"),
        
        # Launch RViz with default configuration
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        ),

        LogInfo(msg="Launching TF broadcaster for blueboat"),
        
        # Launch TF broadcaster
        Node(
            package='move_blueboat',
            executable='blueboat_tf_broadcaster',
            name='blueboat_tf_broadcaster',
            output='screen'
        ),
    ])
