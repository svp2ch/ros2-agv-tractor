"""
Единый launch-файл для запуска всей системы беспилотного трактора.

Последовательность запуска (TimerAction):
  t=0с   — Gazebo (gzserver + robot_state_publisher + spawn)
  t=8с   — sensor_processing (EKF, лидар, NAVSAT) — ждём стабилизации Gazebo
  t=15с  — navigation (Nav2 + SLAM + планировщик покрытия) — ждём TF odom от EKF
  t=20с  — decision_maker (FSM) — ждём готовности Nav2
  t=22с  — RViz2 (только если rviz:=true) — вся система уже поднята

Использование:
  ros2 launch agv_bringup full_system.launch.py
  ros2 launch agv_bringup full_system.launch.py rviz:=true
  ros2 launch agv_bringup full_system.launch.py coverage:=false rviz:=true
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    LogInfo,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # --- Директории пакетов ---
    gazebo_pkg      = get_package_share_directory('agv_gazebo')
    sensor_pkg      = get_package_share_directory('agv_sensor_processing')
    nav_pkg         = get_package_share_directory('agv_navigation')
    decision_pkg    = get_package_share_directory('agv_decision')
    viz_pkg         = get_package_share_directory('agv_visualization')

    # --- Аргументы командной строки ---
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Запустить RViz2 (true/false)'
    )

    coverage_arg = DeclareLaunchArgument(
        'coverage',
        default_value='true',
        description='Запустить планировщик покрытия территории'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Использовать симулированное время Gazebo'
    )

    slam_arg = DeclareLaunchArgument(
        'slam',
        default_value='true',
        description='Включить SLAM (slam_toolbox)'
    )

    use_sim_time  = LaunchConfiguration('use_sim_time')
    start_rviz    = LaunchConfiguration('rviz')
    start_coverage = LaunchConfiguration('coverage')
    use_slam      = LaunchConfiguration('slam')

    # =========================================================
    # t = 0с  —  GAZEBO (headless: gzserver + RSP + spawn)
    #
    # robot_state_publisher запускается внутри gazebo.launch.py —
    # дублировать его здесь не нужно (вызывает конфликт имён в ROS 2).
    # RViz2 стартует в t=13с, к этому моменту RSP уже 13с работает.
    # =========================================================
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg, 'launch', 'gazebo.launch.py')
        ),
    )

    # =========================================================
    # t = 8с  —  SENSOR PROCESSING (EKF + лидар + NAVSAT)
    # Увеличено с 5с: Gazebo нужно время чтобы прогрузить мир и
    # начать публиковать данные сенсоров до старта EKF.
    # =========================================================
    sensor_launch = TimerAction(
        period=8.0,
        actions=[
            LogInfo(msg='[agv_bringup] Запуск обработки сенсоров...'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(sensor_pkg, 'launch', 'sensor_processing.launch.py')
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                }.items()
            ),
        ]
    )

    # =========================================================
    # t = 15с  —  NAVIGATION (Nav2 + SLAM + coverage planner)
    # Увеличено с 10с: Nav2 требует активный TF odom→base_link от EKF.
    # EKF стартует в t=8с, нужно ~7с чтобы он прогрелся и опубликовал TF.
    # =========================================================
    nav_launch = TimerAction(
        period=15.0,
        actions=[
            LogInfo(msg='[agv_bringup] Запуск навигации...'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav_pkg, 'launch', 'navigation.launch.py')
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'slam': use_slam,
                    'coverage': start_coverage,
                }.items()
            ),
        ]
    )

    # =========================================================
    # t = 20с  —  DECISION MAKER (FSM)
    # Nav2 стартует в t=15с, нужно ~5с на инициализацию lifecycle-нодов.
    # =========================================================
    decision_launch = TimerAction(
        period=20.0,
        actions=[
            LogInfo(msg='[agv_bringup] Запуск модуля принятия решений (FSM)...'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(decision_pkg, 'launch', 'decision.launch.py')
                ),
            ),
        ]
    )

    # =========================================================
    # t = 22с  —  RVIZ2 (только если rviz:=true)
    # Запускаем последней: вся система уже активна, RViz2 сразу
    # получит /robot_description, TF, /map и /odometry/filtered.
    # =========================================================
    rviz_launch = TimerAction(
        period=22.0,
        actions=[
            LogInfo(msg='[agv_bringup] Запуск RViz2...'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(viz_pkg, 'launch', 'rviz.launch.py')
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                }.items(),
                condition=IfCondition(start_rviz),
            ),
        ]
    )

    return LaunchDescription([
        # Аргументы
        rviz_arg,
        coverage_arg,
        use_sim_time_arg,
        slam_arg,

        # Последовательный запуск системы
        LogInfo(msg='[agv_bringup] === Запуск системы беспилотного трактора ==='),
        LogInfo(msg='[agv_bringup] t=0с  — Gazebo (headless) + robot_state_publisher'),
        gazebo_launch,

        LogInfo(msg='[agv_bringup] t=8с  — Обработка сенсоров (EKF)'),
        sensor_launch,

        LogInfo(msg='[agv_bringup] t=15с — Навигация (Nav2 + SLAM)'),
        nav_launch,

        LogInfo(msg='[agv_bringup] t=20с — Модуль решений (FSM)'),
        decision_launch,

        rviz_launch,
    ])
