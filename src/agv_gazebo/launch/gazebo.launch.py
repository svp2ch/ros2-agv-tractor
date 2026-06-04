"""
Launch-файл для запуска Gazebo Classic в headless режиме с трактором.

Запускает:
  1. gzserver (Gazebo без GUI) с миром agricultural_field
  2. robot_state_publisher с полной моделью (URDF + плагины)
  3. spawn_entity — помещает трактор в мир Gazebo

Использование:
  ros2 launch agv_gazebo gazebo.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Пути к пакетам
    pkg_agv_gazebo = get_package_share_directory('agv_gazebo')
    pkg_gazebo_ros  = get_package_share_directory('gazebo_ros')

    # Аргумент: начальная позиция трактора в мире
    x_arg = DeclareLaunchArgument('x', default_value='0.0',  description='Начальная X позиция')
    y_arg = DeclareLaunchArgument('y', default_value='0.0',  description='Начальная Y позиция')
    z_arg = DeclareLaunchArgument('z', default_value='0.65', description='Начальная Z позиция (высота над землёй)')
    yaw_arg = DeclareLaunchArgument('yaw', default_value='0.0', description='Начальный курс (радианы)')

    # Путь к файлу мира
    world_file = os.path.join(pkg_agv_gazebo, 'worlds', 'agricultural_field.world')

    # Путь к полной модели трактора (URDF + Gazebo-плагины)
    urdf_with_gazebo = os.path.join(pkg_agv_gazebo, 'urdf', 'tractor_gazebo.urdf.xacro')

    # URDF строка, завёрнутая в ParameterValue как того требует ROS 2 Humble
    robot_description = ParameterValue(
        Command(['xacro ', urdf_with_gazebo]),
        value_type=str
    )

    # Запуск gzserver (headless: gui:=false — обязательно, VM не тянет рендер)
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={
            'world':   world_file,
            'verbose': 'false',
            'pause':   'false',
        }.items()
    )

    # Публикуем URDF в /robot_description и TF-дерево
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description,
                     'publish_frequency': 50.0}]
    )

    # Помещаем модель трактора в мир Gazebo
    spawn_tractor = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_tractor',
        output='screen',
        arguments=[
            '-topic', '/robot_description',
            '-entity', 'agv_tractor',
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z'),
            '-Y', LaunchConfiguration('yaw'),
        ]
    )

    return LaunchDescription([
        x_arg,
        y_arg,
        z_arg,
        yaw_arg,
        gazebo_server,
        robot_state_publisher,
        spawn_tractor,
    ])
