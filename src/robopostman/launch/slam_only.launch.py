import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
 
 
def generate_launch_description():
    pkg = get_package_share_directory('robopostman')
    slam_pkg = get_package_share_directory('slam_toolbox')
 
    urdf_path = os.path.join(pkg, 'urdf', 'robopostman.urdf')
    world_path = os.path.join(pkg, 'worlds', 'neighborhood.sdf')
    slam_params = os.path.join(pkg, 'config', 'slam_toolbox_params.yaml')
    rviz_config = os.path.join(pkg, 'rviz', 'robopostman.rviz')
 
    with open(urdf_path, 'r') as f:
        robot_description = f.read()
 
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_path],
        output='screen',
    )
 
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }]
    )
 
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'robopostman', '-string', robot_description,
                   '-x', '0.0', '-y', '0.0', '-z', '0.1'],
        output='screen',
    )
 
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        ],
        output='screen',
    )
 
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_pkg, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'slam_params_file': slam_params,
            'use_sim_time': 'true',
        }.items(),
    )
 
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )
 
    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        TimerAction(period=3.0, actions=[spawn_robot]),
        TimerAction(period=4.0, actions=[gz_bridge]),
        TimerAction(period=5.0, actions=[slam]),
        TimerAction(period=8.0, actions=[rviz]),
    ])
