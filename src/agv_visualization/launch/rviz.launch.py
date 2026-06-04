"""
Launch-файл для запуска RViz2 с конфигурацией трактора.

Отображает: модель трактора, лидар, одометрию EKF, карту SLAM,
costmap'ы Nav2, маршрут покрытия, состояние FSM.

Запуск:
    ros2 launch agv_visualization rviz.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('agv_visualization')

    # Путь к конфигурационному файлу RViz2
    rviz_config = os.path.join(pkg_dir, 'config', 'agv_rviz.rviz')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Использовать симулированное время'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    return LaunchDescription([
        use_sim_time_arg,
        rviz_node,
    ])
