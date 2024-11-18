from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Declare the launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')


    pkg_share = get_package_share_directory('vipar_bringup')
    urdf_file = os.path.join(pkg_share, 'description', 'box.urdf')
    
    world_file = PathJoinSubstitution((pkg_share), "worlds", "test.sdf")
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': ParameterValue(
            Command(['xacro ', urdf_file]), value_type=str)}]
    )

    # Joint State Publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}]
    )    

    mpu6050_node = Node(
        package='vipar_driver',
        executable='mpu6050',
        name='mpu6050_node'
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_share, 'config', 'config.rviz')]
    )

    # Gazebo Sim
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        # launch_arguments={'gz_args': world_file}.items(),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # Bridge between Gazebo and ROS
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
            '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
        ],
        output='screen'
    )

    
    return LaunchDescription([
        robot_state_publisher,
        mpu6050_node,
        rviz_node
    ])