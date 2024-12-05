from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    descr_path = get_package_share_path('dexhand_description')
    default_model_path = descr_path / 'urdf/dexhand-right.xacro'
    default_rviz_config_path = descr_path / 'rviz/urdf.rviz'

    # Camera parameters
    camera_arg = DeclareLaunchArgument(
        name='camera_index',
        default_value='0',
        description='Camera device index'
    )

    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='true',
        choices=['true', 'false'],
        description='Flag to enable joint_state_publisher_gui'
    )

    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=str(default_model_path),
        description='Absolute path to robot urdf file'
    )

    rviz_arg = DeclareLaunchArgument(
        name='rvizconfig',
        default_value=str(default_rviz_config_path),
        description='Absolute path to rviz config file'
    )

    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    # Hand tracking node using OpenCV and MediaPipe
    hand_tracking_node = Node(
        package='dexhand_description',
        executable='hand_tracking_node.py',
        name='hand_tracking',
        parameters=[{
            'camera_index': LaunchConfiguration('camera_index'),
        }],
        output='screen'
    )

    # Joint state publisher node that subscribes to hand tracking data
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('gui')),
        parameters=[{
            'use_hand_tracking': True,
            'source_topic': '/hand_tracking/joint_states'
        }]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    return LaunchDescription([
        camera_arg,
        gui_arg,
        model_arg,
        rviz_arg,
        hand_tracking_node,
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node
    ])
