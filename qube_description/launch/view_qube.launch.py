from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import xacro
import os

def generate_launch_description():
    urdf_file=os.path.join(get_package_share_directory('qube_description'), 'urdf', 'qube.urdf.xacro')

    # Process URDF file using Command substitution instead of direct xacro processing
    robot_description_content = Command(['xacro ', urdf_file])

    # Use processed URDF content for robot_description parameter
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen'
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[],
        output='screen'
    )

    return LaunchDescription([
        #joint_state_publisher_gui,
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz2_node,
    ])



