from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os
import xacro


def generate_launch_description():
    # Declare launch arguments with default values
    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate', default_value='115200',
        description='Communication frequency'
    )

    device_arg = DeclareLaunchArgument(
        'device', default_value='/dev/ttyUSB0',
        description='USB device'
    )

    simulation_arg = DeclareLaunchArgument(
        'simulation', default_value='False',
        description='Set whether the program should control real (False) or simulated hardware (True)'
    )

    qube_driver_launch = os.path.join(
        get_package_share_directory('qube_driver'),
        'launch',
        'qube_driver.launch.py'
    )

    xacro_file = os.path.join(
        get_package_share_directory("joint_description"),
        "urdf",
        "controlled_qube.urdf.xacro"
    )

    # Process the xacro file with parameters
    mappings = {
        'baud_rate': LaunchConfiguration('baud_rate'),
        'device': LaunchConfiguration('device'),
        'simulation': LaunchConfiguration('simulation')
    }
    robot_description_content = xacro.process_file(xacro_file, mappings=mappings).toxml()

    # Define the qube_driver launch include
    qube_driver_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(qube_driver_launch),
        launch_arguments=mappings.items()
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        # arguments=['-d', os.path.join(get_package_share_directory("joint_description"), 'config', 'your_config.rviz')]
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}]
    )

    args = [
        baud_rate_arg,
        device_arg,
        simulation_arg
    ]

    nodes_to_start = [
        qube_driver_include,
        rviz_node,
        robot_state_publisher_node
    ]


    return LaunchDescription([
        *args,
        *nodes_to_start
    ])