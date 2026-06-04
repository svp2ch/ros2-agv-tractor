"""
Launch-файл для запуска узла принятия решений (FSM).

Запуск:
    ros2 launch agv_decision decision.launch.py

Параметры командной строки:
    emergency_distance:=0.3   — критическая дистанция до препятствия (м)
    warning_distance:=1.0     — дистанция предупреждения (м)
    sensor_timeout:=5.0       — таймаут сенсоров (с)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # --- Аргументы командной строки ---
    args = [
        DeclareLaunchArgument(
            'emergency_distance', default_value='0.3',
            description='Критическая дистанция до препятствия (м) — аварийная остановка',
        ),
        DeclareLaunchArgument(
            'warning_distance', default_value='1.0',
            description='Дистанция предупреждения (м) — замедление/объезд',
        ),
        DeclareLaunchArgument(
            'sensor_timeout', default_value='5.0',
            description='Таймаут сенсора в секундах — переход в SENSOR_FAILURE',
        ),
        DeclareLaunchArgument(
            'turn_radius', default_value='2.0',
            description='Радиус обнаружения точки разворота (м)',
        ),
        DeclareLaunchArgument(
            'turn_duration', default_value='4.0',
            description='Длительность фазы разворота (с)',
        ),
        DeclareLaunchArgument(
            'emergency_reset_time', default_value='10.0',
            description='Время авто-сброса EMERGENCY_STOP (с)',
        ),
    ]

    decision_node = Node(
        package='agv_decision',
        executable='decision_node',
        name='decision_node',
        output='screen',
        parameters=[{
            'emergency_distance': LaunchConfiguration('emergency_distance'),
            'warning_distance': LaunchConfiguration('warning_distance'),
            'sensor_timeout': LaunchConfiguration('sensor_timeout'),
            'turn_radius': LaunchConfiguration('turn_radius'),
            'turn_duration': LaunchConfiguration('turn_duration'),
            'emergency_reset_time': LaunchConfiguration('emergency_reset_time'),
            'fsm_rate': 5.0,
        }],
    )

    return LaunchDescription(args + [decision_node])
