from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os
import xacro
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Declare arguments for robot description and control xacro files
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
        description='Set whether the program should control simulated hardware (True) or real hardware (False)'
    )

    # Declare arguments for the pid controller
    kp_arg = DeclareLaunchArgument(
        'kp', default_value='3.0',
        description='Proportional gain'
    )

    ki_arg = DeclareLaunchArgument(
        'ki', default_value='0.1',
        description='Integral gain'
    )

    kd_arg = DeclareLaunchArgument(
        'kd', default_value='0.2',
        description='Derivative gain'
    )

    setpoint_arg = DeclareLaunchArgument(
        'setpoint', default_value='0.0',
        description='Target position'
    )

    qube_driver_launch = os.path.join(
        get_package_share_directory('qube_driver'),
        'launch',
        'qube_driver.launch.py'
    )

    controlled_cube_xacro = os.path.join(
        get_package_share_directory('qube_bringup'),
        "urdf",
        "controlled_qube.urdf.xacro"
    )

    # Process the xacro file with parameters
    mappings = {
        'baud_rate': LaunchConfiguration('baud_rate'),
        'device': LaunchConfiguration('device'),
        'simulation': LaunchConfiguration('simulation')
    }

    robot_description_content = Command([
        'xacro ', controlled_cube_xacro,
        ' baud_rate:=', mappings['baud_rate'],
        ' device:=', mappings['device'],
        ' simulation:=', mappings['simulation']
    ])

    robot_description = {'robot_description': ParameterValue(
        robot_description_content,
        value_type=str
    )}

    #Define the qube_driver launch include
    qube_driver_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(qube_driver_launch),
        launch_arguments=mappings.items()
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(get_package_share_directory("qube_bringup"), 'config', 'qube_rviz_config.rviz')]
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
        # parameters = [{'robot_description': ParameterValue(robot_description_content, value_type=str)}]
    )

    rqt_reconfigure_node = Node(
        package='rqt_reconfigure',
        executable='rqt_reconfigure',
        name='rqt_reconfigure',
    )

    qube_controller_node = Node(
        package="qube_controller",
        executable="qube_controller_node",
        name="qube_controller_node",
        parameters=[{
            'kp': LaunchConfiguration('kp'),
            'ki': LaunchConfiguration('ki'),
            'kd': LaunchConfiguration('kd'),
            'setpoint': LaunchConfiguration('setpoint'),
        }]
    )

    return LaunchDescription([
        # Args
        baud_rate_arg,
        device_arg,
        simulation_arg,
        kp_arg,
        ki_arg,
        kd_arg,
        setpoint_arg,
        # Launch file from qube driver
        qube_driver_include,
        # Nodes
        rviz_node,
        robot_state_publisher_node,
        qube_controller_node,
        rqt_reconfigure_node
    ])