"""
Launch-файл для отображения URDF модели трактора в RViz2.
Запускает: robot_state_publisher, joint_state_publisher_gui, rviz2.
Использование: ros2 launch agv_description display.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Путь к пакету
    pkg_share = get_package_share_directory('agv_description')

    # Путь к URDF/Xacro файлу
    urdf_file = os.path.join(pkg_share, 'urdf', 'tractor.urdf.xacro')

    # Путь к конфигурации RViz2
    rviz_config = os.path.join(pkg_share, 'rviz', 'tractor.rviz')

    # Аргумент: использовать ли GUI для joint_state_publisher
    use_joint_state_publisher_gui_arg = DeclareLaunchArgument(
        'use_joint_state_publisher_gui',
        default_value='true',
        description='Запускать joint_state_publisher с GUI (true) или без (false)'
    )
    use_gui = LaunchConfiguration('use_joint_state_publisher_gui')

    # Преобразуем Xacro в URDF строку; ParameterValue(str) нужен в ROS 2 Humble,
    # чтобы launch не пытался интерпретировать XML как YAML
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )

    # Узел: публикует URDF в топик /robot_description и TF-дерево
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # Узел: публикует состояния суставов (колёс), нужен для TF
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        condition=UnlessCondition(use_gui)
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        condition=IfCondition(use_gui)
    )

    # Узел: RViz2 визуализация
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
    )

    return LaunchDescription([
        use_joint_state_publisher_gui_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
    ])
