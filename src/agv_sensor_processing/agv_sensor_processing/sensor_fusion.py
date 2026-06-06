#!/usr/bin/env python3
"""
Узел мониторинга слияния сенсорных данных (sensor_fusion).

Что делает:
  - Следит за тем, что все сенсоры (GPS, IMU, одометрия) публикуют данные.
  - Публикует диагностику в /sensor_fusion/diagnostics каждую секунду.
  - Логирует предупреждения, если сенсор замолчал дольше порога.

Само слияние данных выполняет robot_localization ekf_node (C++ узел,
запускается в том же launch-файле). Этот узел — диагностический монитор.

Публикует:
  /sensor_fusion/diagnostics  (diagnostic_msgs/DiagnosticArray)

Подписывается:
  /gps/fix   (sensor_msgs/NavSatFix)
  /imu/data  (sensor_msgs/Imu)
  /odom      (nav_msgs/Odometry)
  /odometry/filtered  (nav_msgs/Odometry) — выход EKF
"""

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


class SensorFusionMonitor(Node):
    """Мониторит здоровье сенсоров и качество EKF-слияния."""

    # Максимальное время молчания сенсора до предупреждения (секунды)
    SENSOR_TIMEOUT = 2.0

    def __init__(self):
        super().__init__('sensor_fusion_monitor')

        # Время последнего сообщения от каждого сенсора
        self._last_msg_time: dict[str, float] = {
            'gps':      0.0,
            'imu':      0.0,
            'odom':     0.0,
            'ekf_out':  0.0,
        }

        # Счётчики принятых сообщений (для оценки частоты)
        self._msg_count: dict[str, int] = {k: 0 for k in self._last_msg_time}

        # Подписки на сенсоры
        self.create_subscription(NavSatFix, '/gps/fix',
                                 self._gps_callback, 10)
        self.create_subscription(Imu, '/imu/data',
                                 self._imu_callback, 10)
        self.create_subscription(Odometry, '/odom',
                                 self._odom_callback, 10)
        self.create_subscription(Odometry, '/odometry/filtered',
                                 self._ekf_callback, 10)

        # Публикатор диагностики
        self._diag_pub = self.create_publisher(
            DiagnosticArray, '/sensor_fusion/diagnostics', 10)

        # Таймер: публикуем диагностику раз в секунду
        self.create_timer(1.0, self._publish_diagnostics)

        self.get_logger().info('Монитор слияния сенсоров запущен.')
        self.get_logger().info(
            'Ожидаемые топики: /gps/fix, /imu/data, /odom, /odometry/filtered')

    # ------------------------------------------------------------------ #
    #  Callbacks сенсоров                                                  #
    # ------------------------------------------------------------------ #

    def _gps_callback(self, msg: NavSatFix) -> None:
        self._record('gps')
        # Предупреждаем о потере фиксации (STATUS_NO_FIX = -1)
        if msg.status.status < 0:
            self.get_logger().warn('GPS: нет спутникового фикса (STATUS_NO_FIX).')

    def _imu_callback(self, _msg: Imu) -> None:
        self._record('imu')

    def _odom_callback(self, _msg: Odometry) -> None:
        self._record('odom')

    def _ekf_callback(self, _msg: Odometry) -> None:
        self._record('ekf_out')

    def _record(self, sensor: str) -> None:
        """Запоминаем время последнего сообщения от сенсора."""
        self._last_msg_time[sensor] = self.get_clock().now().nanoseconds * 1e-9
        self._msg_count[sensor] += 1

    # ------------------------------------------------------------------ #
    #  Диагностика                                                         #
    # ------------------------------------------------------------------ #

    def _publish_diagnostics(self) -> None:
        now_sec = self.get_clock().now().nanoseconds * 1e-9

        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()

        sensor_names = {
            'gps':     '/gps/fix',
            'imu':     '/imu/data',
            'odom':    '/odom',
            'ekf_out': '/odometry/filtered',
        }

        all_ok = True
        for key, topic in sensor_names.items():
            status = DiagnosticStatus()
            status.name = f'sensor_fusion/{key}'
            status.hardware_id = topic

            last = self._last_msg_time[key]
            age = now_sec - last if last > 0.0 else float('inf')

            if last == 0.0:
                # Ещё ни одного сообщения с момента запуска
                status.level = DiagnosticStatus.WARN
                status.message = 'Ожидание данных...'
                all_ok = False
            elif age > self.SENSOR_TIMEOUT:
                status.level = DiagnosticStatus.ERROR
                status.message = f'Нет данных {age:.1f} с — сенсор недоступен!'
                all_ok = False
                self.get_logger().error(
                    f'{topic}: нет данных уже {age:.1f} с!')
            else:
                status.level = DiagnosticStatus.OK
                status.message = f'OK, последнее сообщение {age:.2f} с назад'

            status.values = [
                KeyValue(key='msgs_total', value=str(self._msg_count[key])),
                KeyValue(key='age_sec',    value=f'{age:.2f}'),
            ]
            array.status.append(status)

        if all_ok:
            self.get_logger().debug('Все сенсоры в норме, EKF работает.')

        self._diag_pub.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = SensorFusionMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
