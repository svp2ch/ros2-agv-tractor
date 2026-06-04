"""
Launch-файл системы навигации трактора.

Запускает:
  1. Nav2 (navigation2) с нашими параметрами
  2. slam_toolbox в режиме онлайн-SLAM (создаёт карту на лету)
  3. Планировщик покрытия территории

Требует запущенного agv_gazebo и agv_sensor_processing!
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    # Директории пакетов
    nav_pkg_dir = get_package_share_directory('agv_navigation')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # Аргументы запуска
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Использовать симулированное время'
    )

    slam_arg = DeclareLaunchArgument(
        'slam',
        default_value='true',
        description='Включить SLAM (иначе — AMCL с готовой картой)'
    )

    map_yaml_arg = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Путь к YAML файлу карты (только если slam:=false)'
    )

    coverage_arg = DeclareLaunchArgument(
        'coverage',
        default_value='false',
        description='Запустить планировщик покрытия территории'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    slam = LaunchConfiguration('slam')
    map_yaml = LaunchConfiguration('map')
    start_coverage = LaunchConfiguration('coverage')

    # Путь к конфигурации Nav2
    nav2_params_file = os.path.join(nav_pkg_dir, 'config', 'nav2_params.yaml')

    # --- Глобальный параметр use_sim_time ---
    set_sim_time = SetParameter(name='use_sim_time', value=use_sim_time)

    # --- SLAM Toolbox (онлайн SLAM) ---
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'odom_frame': 'odom',
                'map_frame': 'map',
                'base_frame': 'base_link',
                'scan_topic': '/scan',
                'mode': 'mapping',
                'max_laser_range': 20.0,
                'minimum_travel_distance': 0.3,
                'minimum_travel_heading': 0.3,
                'map_update_interval': 2.0,     # Реже обновляем карту
                'resolution': 0.05,             # Более точная карта (вдвое мельче)
                'max_iterations': 5,            # Меньше итераций = меньше CPU
                'enable_interactive_mode': False,
            }
        ],
        condition=IfCondition(slam),
    )

    # --- Nav2 bringup ---
    # Используем стандартный launch Nav2, передаём наши параметры
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file,
            'autostart': 'True',
            'use_composition': 'False',   # Отдельные процессы — легче для VM
        }.items()
    )

    # --- Планировщик покрытия (запускается через 10с после Nav2) ---
    coverage_planner = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='agv_navigation',
                executable='coverage_planner',
                name='coverage_planner',
                output='screen',
                parameters=[
                    {
                        'use_sim_time': use_sim_time,
                        'working_width': 2.0,       # Ширина захвата трактора, м
                        'robot_radius': 1.5,
                        'field_x_min': -18.0,       # Границы поля (Gazebo мир)
                        'field_x_max': 18.0,
                        'field_y_min': -18.0,
                        'field_y_max': 18.0,
                        'use_map': False,            # Использовать фиксированные границы
                    }
                ],
                condition=IfCondition(start_coverage),
            )
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        slam_arg,
        map_yaml_arg,
        coverage_arg,
        set_sim_time,
        slam_toolbox_node,
        nav2_launch,
        coverage_planner,
    ])
