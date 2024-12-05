from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Camera parameters
    camera_arg = DeclareLaunchArgument(
        name='camera_index',
        default_value='0',
        description='Camera device index'
    )

    # Hand tracking node
    hand_tracking_node = Node(
        package='dexhand_description',  # Your package name
        executable='hand_tracking_node.py',
        name='hand_tracking',
        parameters=[{
            'camera_index': LaunchConfiguration('camera_index'),
        }],
        output='screen'
    )

    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher'
    )

    # Joint state publisher
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'use_hand_tracking': True,
            'source_topic': '/hand_tracking/joint_states'
        }]
    )

    # RViz visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', 'path/to/your/config.rviz'],  # Update with your RViz config path
        output='screen'
    )

    return LaunchDescription([
        camera_arg,
        hand_tracking_node,
        robot_state_publisher_node, 
        joint_state_publisher_node,
        rviz_node
    ])
