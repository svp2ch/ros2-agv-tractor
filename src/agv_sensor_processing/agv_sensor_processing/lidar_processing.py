#!/usr/bin/env python3
"""
Узел обработки данных 2D лидара (lidar_processing).

Этапы обработки:
  1. Фильтрация LaserScan: убираем inf/nan и лучи вне рабочего диапазона.
  2. Построение OccupancyGrid: локальная карта занятости вокруг трактора.
     Используем трассировку луча (алгоритм Брезенхема) — свободные ячейки
     вдоль луча помечаются как 0, конечная точка (препятствие) — как 100.

Подписывается:
  /scan               (sensor_msgs/LaserScan)

Публикует:
  /scan_filtered      (sensor_msgs/LaserScan)   — отфильтрованный скан
  /local_map          (nav_msgs/OccupancyGrid)  — локальная карта занятости
"""

import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, MapMetaData
from builtin_interfaces.msg import Time


class LidarProcessor(Node):
    """Фильтрует LaserScan и строит локальную OccupancyGrid."""

    def __init__(self):
        super().__init__('lidar_processor')

        # ---- Параметры (можно переопределить через launch-аргументы) ---- #
        self.declare_parameter('min_range', 0.15)
        self.declare_parameter('max_range', 15.0)
        self.declare_parameter('grid_resolution', 0.1)   # м/ячейка
        self.declare_parameter('grid_cells', 200)         # сторона сетки в ячейках (200*0.1=20 м)
        self.declare_parameter('input_topic',  '/scan')
        self.declare_parameter('output_scan',  '/scan_filtered')
        self.declare_parameter('output_grid',  '/local_map')

        self._min_range   = self.get_parameter('min_range').value
        self._max_range   = self.get_parameter('max_range').value
        self._resolution  = self.get_parameter('grid_resolution').value
        self._grid_cells  = self.get_parameter('grid_cells').value
        self._input_topic = self.get_parameter('input_topic').value

        # Размер сетки в метрах
        self._grid_size_m = self._grid_cells * self._resolution

        # Подписка на сырой скан
        self._scan_sub = self.create_subscription(
            LaserScan,
            self._input_topic,
            self._scan_callback,
            rclpy.qos.qos_profile_sensor_data  # latch=False, best-effort
        )

        # Публикаторы
        self._filtered_pub = self.create_publisher(
            LaserScan, self.get_parameter('output_scan').value, 10)
        self._grid_pub = self.create_publisher(
            OccupancyGrid, self.get_parameter('output_grid').value, 10)

        self.get_logger().info(
            f'lidar_processor запущен: '
            f'диапазон [{self._min_range}, {self._max_range}] м, '
            f'сетка {self._grid_cells}x{self._grid_cells} ячеек '
            f'({self._grid_size_m:.0f}x{self._grid_size_m:.0f} м)')

    # ------------------------------------------------------------------ #
    #  Основной callback                                                   #
    # ------------------------------------------------------------------ #

    def _scan_callback(self, msg: LaserScan) -> None:
        filtered = self._filter_scan(msg)
        self._filtered_pub.publish(filtered)
        grid = self._build_occupancy_grid(filtered)
        self._grid_pub.publish(grid)

    # ------------------------------------------------------------------ #
    #  Шаг 1: Фильтрация скана                                            #
    # ------------------------------------------------------------------ #

    def _filter_scan(self, msg: LaserScan) -> LaserScan:
        """
        Возвращает новый LaserScan, где:
          - nan/inf заменены на range_max (признак «нет измерения»)
          - лучи вне [min_range, max_range] также заменены на range_max
        """
        out = LaserScan()
        out.header        = msg.header
        out.angle_min     = msg.angle_min
        out.angle_max     = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment  = msg.time_increment
        out.scan_time     = msg.scan_time
        out.range_min     = self._min_range
        out.range_max     = self._max_range

        filtered_ranges = []
        for r in msg.ranges:
            if math.isfinite(r) and self._min_range <= r <= self._max_range:
                filtered_ranges.append(r)
            else:
                # Помечаем как «нет объекта» — стандартный признак в ROS
                filtered_ranges.append(self._max_range)

        out.ranges = filtered_ranges
        out.intensities = msg.intensities if msg.intensities else []
        return out

    # ------------------------------------------------------------------ #
    #  Шаг 2: Построение OccupancyGrid                                    #
    # ------------------------------------------------------------------ #

    def _build_occupancy_grid(self, scan: LaserScan) -> OccupancyGrid:
        """
        Строит локальную карту занятости, центрированную на лидаре.

        Алгоритм:
          - Сетка N x N ячеек, центр сетки = позиция лидара.
          - Для каждого луча: трассировка ячеек от центра до препятствия
            (ячейки вдоль луча = свободны = 0), крайняя ячейка = занята = 100.
          - Ячейки без луча = неизвестны = -1.
        """
        N = self._grid_cells
        half = N // 2
        res  = self._resolution

        # Инициализируем все ячейки как «неизвестно» (-1)
        data = [-1] * (N * N)

        angle = scan.angle_min
        for r in scan.ranges:
            if r < self._max_range:  # только реальные препятствия
                # Координаты конца луча в метрах (система координат лидара)
                end_x = r * math.cos(angle)
                end_y = r * math.sin(angle)

                # Координаты в ячейках сетки (центр = (half, half))
                end_cx = int(end_x / res) + half
                end_cy = int(end_y / res) + half

                # Трассировка луча по алгоритму Брезенхема:
                # помечаем ячейки от центра до препятствия как свободные
                for cx, cy in self._bresenham(half, half, end_cx, end_cy):
                    if 0 <= cx < N and 0 <= cy < N:
                        idx = cy * N + cx
                        # Не перезаписываем занятую ячейку свободной
                        if data[idx] != 100:
                            data[idx] = 0  # свободно

                # Конечная ячейка — занята (препятствие)
                if 0 <= end_cx < N and 0 <= end_cy < N:
                    data[end_cy * N + end_cx] = 100

            angle += scan.angle_increment

        # Формируем сообщение OccupancyGrid
        grid = OccupancyGrid()
        grid.header.stamp    = scan.header.stamp
        grid.header.frame_id = scan.header.frame_id  # lidar_link

        grid.info = MapMetaData()
        grid.info.resolution = res
        grid.info.width      = N
        grid.info.height     = N
        # Начало координат сетки — нижний левый угол (центр = лидар)
        grid.info.origin.position.x = -half * res
        grid.info.origin.position.y = -half * res
        grid.info.origin.position.z = 0.0
        grid.info.origin.orientation.w = 1.0

        grid.data = data
        return grid

    # ------------------------------------------------------------------ #
    #  Вспомогательные методы                                             #
    # ------------------------------------------------------------------ #

    @staticmethod
    def _bresenham(x0: int, y0: int, x1: int, y1: int):
        """
        Генератор ячеек целочисленной сетки вдоль отрезка (x0,y0)-(x1,y1).
        Классический алгоритм Брезенхема для растеризации линии.
        Возвращает ячейки от (x0,y0) до (x1,y1) ИСКЛЮЧАЯ конечную точку.
        """
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        while True:
            # Останавливаемся до конечной точки (её помечаем отдельно)
            if x0 == x1 and y0 == y1:
                break
            yield (x0, y0)
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0  += sx
            if e2 < dx:
                err += dx
                y0  += sy


def main(args=None):
    rclpy.init(args=args)
    node = LidarProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
