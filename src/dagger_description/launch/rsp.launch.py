import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    pkg_path = get_package_share_directory("dagger_description")
    xacro_file = os.path.join(pkg_path, 'urdf', 'robot.urdf.xacro')

    robot_description = xacro.process_file(
        xacro_file,
        mappings={'use_sim': 'False'}
    ).toxml()
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
        'robot_description': robot_description
        }]
    )
    
    # Only the two wheel joints move, and nothing but the Foxglove robot model
    # uses their frames (all sensors hang off fixed joints): 2 Hz, not 20 Hz
    # (measured 9.5 % CPU at 20 Hz, 2026-09-29). This is the file drive_bringup
    # (T1) includes.
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        output="screen",
        parameters=[{"rate": 2.0}],
    )
    
    static_tf_lidar = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="lidar_to_rslidar_tf",
        arguments=[
            "0", "0", "0",        # x y z
            "0", "0", "0",        # roll pitch yaw (radians)
            "lidar_link",         # parent frame
            "rslidar"             # child frame
        ],
        output="screen"
    )
    
    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher,
        static_tf_lidar
    ])
