from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_path

def generate_launch_description():
    descr_path = get_package_share_path('dexhand_description')
    default_model_path = descr_path / 'urdf/dexhand-right.xacro'
    default_rviz_config_path = descr_path / 'rviz/urdf.rviz'

    # Model argument
    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=str(default_model_path),
        description='Path to robot URDF/XACRO file'
    )

    # Camera parameters
    camera_arg = DeclareLaunchArgument(
        name='camera_index',
        default_value='0',
        description='Camera device index'
    )

    # Robot description
    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]),
        value_type=str
    )

    # Hand tracking node
    hand_tracking_node = Node(
        package='dexhand_description',
        executable='hand_tracking_node',
        name='hand_tracking',
        parameters=[{
            'camera_index': LaunchConfiguration('camera_index'),
            'robot_description': robot_description
        }],
        output='screen'
    )

    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'publish_frequency': 30.0
        }]
    )

    # RViz visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', str(default_rviz_config_path)],
        output='screen'
    )

    return LaunchDescription([
        model_arg,
        camera_arg,
        hand_tracking_node,
        robot_state_publisher_node,
        rviz_node
    ])
