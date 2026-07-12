from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup = FindPackageShare("dagger_bringup")

    use_camera = LaunchConfiguration("use_camera")
    use_camera_arg = DeclareLaunchArgument("use_camera", default_value="false")

    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup, "launch", "description.launch.py"])
        )
    )

    sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup, "launch", "sensors.launch.py"])
        ),
        launch_arguments={"use_camera": use_camera}.items(),
    )

    safety = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup, "launch", "safety.launch.py"])
        )
    )

    return LaunchDescription([use_camera_arg, description, sensors, safety])
