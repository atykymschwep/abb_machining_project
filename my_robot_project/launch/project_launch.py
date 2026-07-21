import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():
    abb_model_path = get_package_share_directory('abb_model')
    my_pkg_path = get_package_share_directory('my_robot_project')

    # Robot State Publisher
    urdf_file = os.path.join(abb_model_path, 'urdf', 'abb_irb4600_60_205.xacro')
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', urdf_file])}]
    )

    # RViz
    rviz_config_file = os.path.join(my_pkg_path, 'rviz', 'config.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file]
    )

    # truck
    spawner_node = Node(
        package='my_robot_project',
        executable='model_spawner'
    )

    # trajectory
    planner_node = Node(
        package='my_robot_project',
        executable='trajectory_planner',
        output='screen'
    )

    return LaunchDescription([
        rsp_node,
        rviz_node,
        spawner_node,
        planner_node
    ])