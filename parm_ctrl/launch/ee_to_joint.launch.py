from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory
import yaml

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except Exception as e:
        return {}

def generate_launch_description():
    # Get paths to config files
    moveit_config_pkg = get_package_share_directory('parm_moveit_config')
    
    # Load YAML configs
    planning_pipelines = load_yaml('parm_moveit_config', 'config/moveit_planning_pipelines.yaml')
    kinematics = load_yaml('parm_moveit_config', 'config/kinematics.yaml')
    
    # Combine parameters
    parameters = {
        'use_sim_time': False,
        'planning_pipelines': {
            'pipeline_names': ['ompl'],
            'ompl': {
                'planning_plugin': 'ompl_interface/OMPLPlanner',
                'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints',
                'start_state_max_bounds_error': 0.1
            }
        }
    }
    parameters.update(planning_pipelines)
    parameters.update(kinematics)
    
    return LaunchDescription([
        Node(
            package='parm_ctrl',
            executable='ee_to_joint.py',
            name='ee_to_joint_node',
            output='screen',
            parameters=[parameters]
        )
    ])