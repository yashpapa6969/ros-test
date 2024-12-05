from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    package_name = 'maverick'
    default_model_path = 'urdf/maverick.urdf'
    default_rviz_config_path = PathJoinSubstitution([
        FindPackageShare(package_name), 'rviz', 'urdf.rviz'
    ])

    # Common node parameters
    node_params = {
        'shoulderFlexion': [('left_shoulder_flexion', 'left'), ('right_shoulder_flexion', 'right')],
        'shoulderAdduction': [('left_shoulder_adduction', 'left'), ('right_shoulder_adduction', 'right')],
        'elbowFlexion': [('left_elbow_flexion', 'left'), ('right_elbow_flexion', 'right')],
        'gripperControl': [('left_gripper_control', 'left'), ('right_gripper_control', 'right')]
    }

    ld = LaunchDescription([
        # Declare parameters
        DeclareLaunchArgument('maverick', default_value=package_name, description='Package name'),
        DeclareLaunchArgument('model', default_value=default_model_path, description='Path to the robot description'),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz_config_path, description='Path to RViz config'),

        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': ParameterValue(
                Command(['xacro ', PathJoinSubstitution([FindPackageShare(package_name), LaunchConfiguration('model')])]),
                value_type=str
            )}]
        ),

        # RViz
        Node(package='rviz2', executable='rviz2', output='screen', arguments=['-d', LaunchConfiguration('rviz_config')])
    ])

    # Only add gripper control nodes
    for name, side in node_params['gripperControl']:
        ld.add_action(Node(
            package=package_name,
            executable='gripperControl',
            name=name,
            parameters=[{'side': side}],
            output='screen'
        ))

    return ld
