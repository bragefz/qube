from sys import executable

from fontTools.misc.cython import returns
from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share_dir=get_package_share_directory('qube_description')
    urdf_file=os.path.join(pkg_share_dir, 'urdf', 'qube.urdf.xacro')

    robot_description=parameterValue(
        command(['xacro',urdf_file]),
        value_type=str
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        output='screen'

    ),

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],

    joint_state_publisher_gui = Node(
        packaging='joint_state_publisher_gui',
        executable='joint'
    )

    return LaunchDescription([


])



