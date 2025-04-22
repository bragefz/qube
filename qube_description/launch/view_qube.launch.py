from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # finner path for qube.urdf.xacro filen
    urdf_file = os.path.join(get_package_share_directory('qube_description'), 'urdf', 'qube.urdf.xacro')

    #lager robot_desciption_content som konverterer xacro til urdf
    robot_description_content = Command(['xacro ', urdf_file])

    #robot_description parameter som inneholder den konverterte urdf modellen
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    #Lager robot_state_publisher_node som er noden som skal publisere tilstandene til robotens ledd
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

    #Lager en statisk transform publisher som definerer forholdet mellom "world" og "map" koordinatsystemene
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'map'] #Argumentene ['0', '0', '0', '0', '0', '0'] gir posisjon (x,y,z) og (r,p,y)
    )

    #prøver å finne en RViz config fil
    try:
        #finner stien til config filen og setter opp RViz med denne configen
        rviz_config = os.path.join(get_package_share_directory('qube_bringup'), 'config', 'qube_rviz_config.rviz')
        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        )
    except:
        #starter RViz standard hvis den ikke finner config fil eller om den ikke finnes
        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        )

    #returnerer en "LaunchDescription" med rekkefølgen noden skal startes på
    return LaunchDescription([
        static_tf_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node,
    ])