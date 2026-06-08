"""
Launch-файл пакета agv_sensor_processing.

Запускает пять узлов:
  1. ekf_local          — robot_localization EKF: odom + IMU  -> /odometry/filtered
  2. navsat_transform   — robot_localization:  GPS NavSatFix  -> /odometry/gps
  3. ekf_global         — robot_localization EKF: filtered + GPS -> /odometry/filtered_map
  4. sensor_fusion      — наш мониторинг здоровья сенсоров
  5. lidar_processor    — фильтрация лидара + OccupancyGrid

Использование:
  ros2 launch agv_sensor_processing sensor_processing.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('agv_sensor_processing')

    # Пути к конфигурационным файлам
    ekf_local_cfg      = os.path.join(pkg, 'config', 'ekf_local.yaml')
    ekf_global_cfg     = os.path.join(pkg, 'config', 'ekf_global.yaml')
    navsat_cfg         = os.path.join(pkg, 'config', 'navsat_transform.yaml')

    # ------------------------------------------------------------------ #
    # 1. EKF локальный: odom + IMU -> /odometry/filtered                  #
    #    Фрейм: odom -> base_link                                         #
    # ------------------------------------------------------------------ #
    ekf_local_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_local',
        output='screen',
        parameters=[ekf_local_cfg,
                    {'use_sim_time': True}],
        remappings=[
            # Стандартный выходной топик robot_localization
            ('odometry/filtered', '/odometry/filtered'),
        ]
    )

    # ------------------------------------------------------------------ #
    # 2. navsat_transform: GPS NavSatFix -> /odometry/gps                 #
    #    Конвертирует широту/долготу в декартовы координаты map-фрейма   #
    # ------------------------------------------------------------------ #
    navsat_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        output='screen',
        parameters=[navsat_cfg,
                    {'use_sim_time': True}],
        remappings=[
            ('imu/data',          '/imu/data'),
            ('gps/fix',           '/gps/fix'),
            # ВАЖНО: navsat должен читать ГЛОБАЛЬНУЮ одометрию (фрейм map),
            # а не локальную (фрейм odom). Тогда /odometry/gps публикуется во
            # фрейме map, и глобальный EKF использует GPS как абсолютное
            # измерение напрямую. Если кормить локальной одометрией, GPS выходит
            # во фрейме odom, EKF преобразует его через свой же map->odom
            # (циклическая зависимость) и оценка расходится, игнорируя GPS.
            ('odometry/filtered', '/odometry/filtered_map'),
            ('odometry/gps',      '/odometry/gps'),
        ]
    )

    # ------------------------------------------------------------------ #
    # 3. EKF глобальный: /odometry/filtered + /odometry/gps              #
    #    -> /odometry/filtered_map                                        #
    #    Фрейм: map -> odom (корректирует накопленный дрейф)             #
    # ------------------------------------------------------------------ #
    ekf_global_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_global',
        output='screen',
        parameters=[ekf_global_cfg,
                    {'use_sim_time': True}],
        remappings=[
            # Выход глобального EKF -> /odometry/filtered_map.
            # Входы (odom0=/odom, imu0=/imu/data, odom1=/odometry/gps) заданы
            # абсолютными именами в yaml и под это правило не попадают.
            ('odometry/filtered', '/odometry/filtered_map'),
        ]
    )

    # ------------------------------------------------------------------ #
    # 4. Монитор слияния сенсоров (наш Python-узел)                      #
    #    Публикует: /sensor_fusion/diagnostics                            #
    # ------------------------------------------------------------------ #
    sensor_fusion_node = Node(
        package='agv_sensor_processing',
        executable='sensor_fusion.py',
        name='sensor_fusion_monitor',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # ------------------------------------------------------------------ #
    # 5. Обработка лидара (наш Python-узел)                              #
    #    Публикует: /scan_filtered, /local_map                           #
    # ------------------------------------------------------------------ #
    lidar_processor_node = Node(
        package='agv_sensor_processing',
        executable='lidar_processing.py',
        name='lidar_processor',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'min_range':       0.15,
            'max_range':       15.0,
            'grid_resolution': 0.1,
            'grid_cells':      200,
        }],
        remappings=[
            ('/scan', '/scan'),
        ]
    )

    return LaunchDescription([
        ekf_local_node,
        navsat_node,
        ekf_global_node,
        sensor_fusion_node,
        lidar_processor_node,
    ])
