import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command

def generate_launch_description():
    pkg_path = get_package_share_directory("dagger_description")
    xacro_file = os.path.join(pkg_path, 'urdf', 'dagger.urdf.xacro')
    rviz_config = os.path.join(pkg_path, 'rviz', 'rviz.rviz')
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
        'robot_description': Command(['xacro ', xacro_file])
        }]
    )
    
    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="screen"
    )
   
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz_config],
        output="screen",        
    )
    
    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz2,
    ])
