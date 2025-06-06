from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Path to the move_group.launch.py file
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('parm_moveit_config'),
                'launch',
                'move_group.launch.py'
            ])
        )
    )

    # Path to the rsp.launch.py file
    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('parm_moveit_config'),
                'launch',
                'rsp.launch.py'
            ])
        )
    )

    # Return the combined launch description
    return LaunchDescription([
        move_group_launch,
        rsp_launch
    ])