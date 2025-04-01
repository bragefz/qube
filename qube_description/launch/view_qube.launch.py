from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get path to the URDF file
    urdf_file = os.path.join(get_package_share_directory('qube_description'), 'urdf', 'qube.urdf.xacro')

    # Process URDF file using Command substitution
    robot_description_content = Command(['xacro ', urdf_file])


    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}


    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[robot_description],
        output='screen'
    )

    # Joint State Publisher GUI node
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'map']
    )


    try:
        rviz_config = os.path.join(get_package_share_directory('qube_bringup'), 'config', 'qube_rviz_config.rviz')
        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        )
    except:

        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        )


    return LaunchDescription([
        static_tf_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node,
    ])