"""
Узел конечного автомата состояний (FSM) беспилотного трактора.

Реализует логику принятия решений на основе данных от Nav2 и сенсоров:
- Данные лидара → обнаружение препятствий, аварийная остановка
- Одометрия EKF → текущее положение, определение точек разворота
- Маршрут покрытия → активация навигации и последовательная отправка
  точек маршрута в Nav2 через action /navigate_to_pose
- Сигнал завершения → переход в TASK_COMPLETE

Граф переходов:
    IDLE ──────────────── (маршрут получен) ──────────────► NAVIGATING
    NAVIGATING ─────────── (препятствие < warning) ────────► AVOIDING_OBSTACLE
    NAVIGATING ─────────── (у точки разворота) ────────────► TURNING
    NAVIGATING ─────────── (coverage_complete) ────────────► TASK_COMPLETE
    AVOIDING_OBSTACLE ──── (путь свободен) ────────────────► NAVIGATING
    TURNING ────────────── (разворот завершён) ─────────────► NAVIGATING
    Любое состояние ─────── (препятствие < emergency) ──────► EMERGENCY_STOP
    Любое состояние ─────── (сенсор молчит > timeout) ──────► SENSOR_FAILURE
    EMERGENCY_STOP ─────── (таймер сброса) ─────────────────► IDLE
    SENSOR_FAILURE ─────── (сенсоры восстановлены) ─────────► IDLE

Дополнительные публикации для визуализации:
    /actual_trajectory  (nav_msgs/Path)            — пройденная траектория
    /robot_status_text  (visualization_msgs/Marker) — текст статуса над трактором
"""

import math
from enum import Enum

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSDurabilityPolicy,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
)

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point, PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray


# Максимум точек в накопленной траектории — старые отбрасываются
_TRAJECTORY_MAX_POINTS = 500


class RobotState(Enum):
    """Состояния конечного автомата трактора."""

    IDLE = 'IDLE'                            # Ожидание задания
    NAVIGATING = 'NAVIGATING'                # Движение по маршруту покрытия
    AVOIDING_OBSTACLE = 'AVOIDING_OBSTACLE'  # Обнаружено препятствие, замедление/объезд
    TURNING = 'TURNING'                      # Разворот на конце полосы бустрофедона
    EMERGENCY_STOP = 'EMERGENCY_STOP'        # Критическое препятствие — полная остановка
    SENSOR_FAILURE = 'SENSOR_FAILURE'        # Отказ одометрии или лидара
    TASK_COMPLETE = 'TASK_COMPLETE'          # Всё поле обработано


# QoS transient_local — совпадает с QoS coverage_planner, чтобы получить
# сохранённое сообщение даже если подписчик запустился позже публикатора
_LATCHING_QOS = QoSProfile(
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)


class DecisionNode(Node):
    """
    Конечный автомат состояний беспилотного трактора.

    Подписки:
        /odometry/filtered (Odometry): фильтрованная одометрия от EKF
        /scan (LaserScan): данные лидара
        /coverage_path (Path): маршрут покрытия, transient_local QoS
        /coverage_complete (Bool): сигнал завершения покрытия

    Публикации:
        /robot_state (String): имя текущего состояния FSM (5 Гц)
        /cmd_vel (Twist): нулевая скорость при EMERGENCY_STOP / SENSOR_FAILURE
        /actual_trajectory (Path): пройденная траектория трактора
        /robot_status_text (Marker): текстовый маркер статуса над трактором

    Экшен-клиент:
        /navigate_to_pose (Nav2): последовательная отправка waypoint-ов маршрута
    """

    def __init__(self):
        super().__init__('decision_node')

        # --- Параметры ---
        self.declare_parameter('emergency_distance', 0.3)
        self.declare_parameter('warning_distance', 1.0)
        self.declare_parameter('sensor_timeout', 5.0)
        self.declare_parameter('turn_radius', 2.0)
        self.declare_parameter('turn_duration', 4.0)
        self.declare_parameter('emergency_reset_time', 10.0)
        self.declare_parameter('fsm_rate', 5.0)

        self._emergency_dist = self.get_parameter('emergency_distance').value
        self._warning_dist = self.get_parameter('warning_distance').value
        self._sensor_timeout = self.get_parameter('sensor_timeout').value
        self._turn_radius = self.get_parameter('turn_radius').value
        self._turn_duration = self.get_parameter('turn_duration').value
        self._emergency_reset = self.get_parameter('emergency_reset_time').value

        # --- Состояние FSM ---
        self._state = RobotState.IDLE
        self._prev_state = RobotState.IDLE

        # --- Кэш последних данных сенсоров ---
        self._odom: Odometry | None = None
        self._scan: LaserScan | None = None
        self._coverage_path: Path | None = None
        self._task_complete = False

        # --- Временные метки (секунды, float) ---
        self._last_odom_t: float = 0.0
        self._last_scan_t: float = 0.0

        # --- Вспомогательные переменные ---
        self._min_dist: float = float('inf')   # минимальное расстояние по лидару
        self._turn_start_t: float = 0.0        # момент начала разворота
        self._emergency_start_t: float = 0.0   # момент входа в EMERGENCY_STOP

        # --- Накопленная траектория (для визуализации) ---
        self._actual_trajectory = Path()
        self._actual_trajectory.header.frame_id = 'odom'

        # --- Управление Nav2 ---
        self._current_wp_idx: int = 0          # индекс текущего waypoint
        self._nav_goal_active: bool = False    # есть ли активный goal в Nav2
        self._nav_goal_handle = None           # handle текущего goal (для отмены)
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # --- Подписки ---
        self.create_subscription(Odometry, '/odometry/filtered',
                                 self._cb_odom, 10)
        self.create_subscription(LaserScan, '/scan',
                                 self._cb_scan, 10)
        self.create_subscription(Path, '/coverage_path',
                                 self._cb_path, _LATCHING_QOS)
        self.create_subscription(Bool, '/coverage_complete',
                                 self._cb_complete, 10)

        # --- Публикации ---
        self._state_pub = self.create_publisher(String, '/robot_state', 10)
        self._cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._trajectory_pub = self.create_publisher(Path, '/actual_trajectory', 10)
        self._status_marker_pub = self.create_publisher(
            Marker, '/robot_status_text', 10
        )
        self._field_pub = self.create_publisher(MarkerArray, '/field_boundary', 10)

        # --- Главный таймер FSM ---
        rate = self.get_parameter('fsm_rate').value
        self.create_timer(1.0 / rate, self._tick)

        # --- Таймер 1 Гц для публикации границ поля (статика, но поддерживаем
        # повторную публикацию для подписчиков, которые подключились позже) ---
        self.create_timer(1.0, self._publish_field_boundary)

        self.get_logger().info('FSM запущен')
        self.get_logger().info(
            f'Аварийная дистанция={self._emergency_dist}м  '
            f'Предупреждение={self._warning_dist}м  '
            f'Таймаут сенсора={self._sensor_timeout}с'
        )

    # ------------------------------------------------------------------ #
    #  Callbacks — обновляют кэш данных                                   #
    # ------------------------------------------------------------------ #

    def _cb_odom(self, msg: Odometry):
        self._odom = msg
        self._last_odom_t = self._now()
        self._append_trajectory(msg)

    def _cb_scan(self, msg: LaserScan):
        self._scan = msg
        self._last_scan_t = self._now()

        # Программный фильтр самозасветки: всё ближе self_clip_dist считается
        # отражением от собственного корпуса (например, GPS-антенны) и игнорируется.
        # Дублирует <min> в URDF лидара — страховка на случай его изменения.
        self_clip_dist = 0.4

        valid = [
            r for r in msg.ranges
            if max(msg.range_min, self_clip_dist) < r < msg.range_max
            and not math.isnan(r) and not math.isinf(r)
        ]
        self._min_dist = min(valid) if valid else float('inf')

    def _cb_path(self, msg: Path):
        self._coverage_path = msg
        self._current_wp_idx = 0
        self.get_logger().info(f'Маршрут покрытия получен: {len(msg.poses)} точек')

    def _cb_complete(self, msg: Bool):
        if msg.data:
            self._task_complete = True
            self.get_logger().info('Получен сигнал завершения покрытия')

    # ------------------------------------------------------------------ #
    #  Накопление пройденной траектории                                   #
    # ------------------------------------------------------------------ #

    def _append_trajectory(self, odom: Odometry):
        """Добавляем позицию из odom в пройденную траекторию (с лимитом по точкам)."""
        ps = PoseStamped()
        ps.header.stamp = odom.header.stamp
        ps.header.frame_id = self._actual_trajectory.header.frame_id
        ps.pose = odom.pose.pose
        self._actual_trajectory.poses.append(ps)

        # Лимит — отбрасываем самые старые
        if len(self._actual_trajectory.poses) > _TRAJECTORY_MAX_POINTS:
            self._actual_trajectory.poses = (
                self._actual_trajectory.poses[-_TRAJECTORY_MAX_POINTS:]
            )

        self._actual_trajectory.header.stamp = odom.header.stamp
        self._trajectory_pub.publish(self._actual_trajectory)

    # ------------------------------------------------------------------ #
    #  Главный цикл FSM                                                   #
    # ------------------------------------------------------------------ #

    def _tick(self):
        """Одна итерация FSM: вычислить переход → выполнить действие → опубликовать."""
        now = self._now()
        next_state = self._next_state(now)
        self._transition(next_state)
        self._act()
        self._publish_state()
        self._publish_status_marker()

    def _next_state(self, now: float) -> RobotState:
        """
        Вычисляем следующее состояние по приоритетам.

        Приоритеты (1 = высший):
          1. Отказ сенсора
          2. Аварийный стоп
          3. Задание выполнено
          4. Нет маршрута → IDLE
          5. Некритическое препятствие → AVOIDING_OBSTACLE
          6. Точка разворота → TURNING
          7. Есть маршрут и путь свободен → NAVIGATING
        """
        # 1. Отказ сенсора
        if self._sensor_failed(now):
            return RobotState.SENSOR_FAILURE

        # Из SENSOR_FAILURE выходим только когда сенсоры снова отвечают
        if self._state == RobotState.SENSOR_FAILURE:
            return RobotState.IDLE

        # 2. Аварийный стоп — всегда имеет высший приоритет после отказа сенсора
        if self._min_dist < self._emergency_dist:
            if self._state != RobotState.EMERGENCY_STOP:
                self._emergency_start_t = now
            return RobotState.EMERGENCY_STOP

        # Из EMERGENCY_STOP выходим только по таймеру
        if self._state == RobotState.EMERGENCY_STOP:
            if (now - self._emergency_start_t) >= self._emergency_reset:
                self.get_logger().info('EMERGENCY_STOP сброшен автоматически')
                return RobotState.IDLE
            return RobotState.EMERGENCY_STOP

        # 3. Задание выполнено — фиксируем навсегда
        if self._task_complete:
            return RobotState.TASK_COMPLETE

        if self._state == RobotState.TASK_COMPLETE:
            return RobotState.TASK_COMPLETE

        # 4. Нет маршрута
        has_path = (
            self._coverage_path is not None
            and len(self._coverage_path.poses) > 0
        )
        if not has_path:
            return RobotState.IDLE

        # 5. Некритическое препятствие
        if self._min_dist < self._warning_dist:
            return RobotState.AVOIDING_OBSTACLE

        # Из AVOIDING_OBSTACLE возвращаемся только когда путь полностью свободен
        if self._state == RobotState.AVOIDING_OBSTACLE:
            self.get_logger().info('Препятствие устранено, возобновляю навигацию')

        # 6. Фаза разворота
        near_turn = self._near_turn_point()
        if near_turn:
            if self._state != RobotState.TURNING:
                self._turn_start_t = now
            if (now - self._turn_start_t) < self._turn_duration:
                return RobotState.TURNING
            if self._state == RobotState.TURNING:
                self.get_logger().info('Разворот завершён')

        # 7. Нормальное движение
        return RobotState.NAVIGATING

    def _sensor_failed(self, now: float) -> bool:
        """Сенсор считается отказавшим если молчит дольше timeout после первого сообщения."""
        odom_fail = self._last_odom_t > 0.0 and (now - self._last_odom_t) > self._sensor_timeout
        scan_fail = self._last_scan_t > 0.0 and (now - self._last_scan_t) > self._sensor_timeout
        return odom_fail or scan_fail

    def _near_turn_point(self) -> bool:
        """
        Проверяем близость к точке разворота.

        В бустрофедонном маршруте точки разворота — нечётные индексы
        (1, 3, 5, ...) — конец каждой горизонтальной полосы.
        """
        if self._odom is None or self._coverage_path is None:
            return False

        rx = self._odom.pose.pose.position.x
        ry = self._odom.pose.pose.position.y

        for i in range(1, len(self._coverage_path.poses), 2):
            wp = self._coverage_path.poses[i].pose.position
            dist = math.hypot(rx - wp.x, ry - wp.y)
            if dist < self._turn_radius:
                return True

        return False

    # ------------------------------------------------------------------ #
    #  Действия и публикации                                              #
    # ------------------------------------------------------------------ #

    def _transition(self, next_state: RobotState):
        """Регистрируем переход состояния с логом + действия на входе в состояние."""
        if next_state == self._state:
            return

        self.get_logger().info(
            f'[FSM] {self._state.value} → {next_state.value}'
            + (f'  (препятствие {self._min_dist:.2f}м)'
               if next_state in (RobotState.EMERGENCY_STOP,
                                 RobotState.AVOIDING_OBSTACLE) else '')
        )
        self._prev_state = self._state
        self._state = next_state

        # Действия на входе в состояние
        if next_state == RobotState.NAVIGATING:
            # Запускаем отправку текущего waypoint в Nav2 (если ещё не активен)
            self._send_current_waypoint()
        elif next_state in (RobotState.EMERGENCY_STOP,
                            RobotState.SENSOR_FAILURE,
                            RobotState.TASK_COMPLETE):
            # Отменяем активную цель Nav2
            self._cancel_active_goal()

    def _act(self):
        """Выполняем действие текущего состояния."""
        # При аварийной остановке или отказе сенсора — гасим скорость
        if self._state in (RobotState.EMERGENCY_STOP, RobotState.SENSOR_FAILURE):
            self._cmd_vel_pub.publish(Twist())  # все поля = 0.0
        elif self._state == RobotState.NAVIGATING and not self._nav_goal_active:
            # Waypoint завершён но состояние не сменилось → отправляем следующий
            self._send_current_waypoint()

    def _publish_state(self):
        """Публикуем имя текущего состояния в /robot_state."""
        msg = String()
        msg.data = self._state.value
        self._state_pub.publish(msg)

    # ------------------------------------------------------------------ #
    #  Управление Nav2: отправка/отмена waypoint                          #
    # ------------------------------------------------------------------ #

    def _send_current_waypoint(self):
        """Отправляем текущий waypoint в Nav2 (если есть путь и нет активной цели)."""
        if self._nav_goal_active:
            return  # уже едем
        if self._coverage_path is None:
            return
        if self._current_wp_idx >= len(self._coverage_path.poses):
            self.get_logger().info('Все waypoint пройдены — задание выполнено')
            self._task_complete = True
            return

        if not self._nav_client.server_is_ready():
            self.get_logger().warn(
                'Nav2 action server не готов, ожидаю...', throttle_duration_sec=5.0
            )
            if not self._nav_client.wait_for_server(timeout_sec=1.0):
                return  # попробуем на следующем тике через повторный вход

        target = self._coverage_path.poses[self._current_wp_idx]

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = (
            target.header.frame_id if target.header.frame_id else 'map'
        )
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose = target.pose

        self.get_logger().info(
            f'Отправляю waypoint {self._current_wp_idx + 1}/'
            f'{len(self._coverage_path.poses)}: '
            f'({target.pose.position.x:.2f}, {target.pose.position.y:.2f})'
        )

        self._nav_goal_active = True
        send_future = self._nav_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        """Колбэк: Nav2 принял или отклонил цель."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Nav2 отклонил цель')
            self._nav_goal_active = False
            self._nav_goal_handle = None
            return

        self._nav_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future):
        """Колбэк: Nav2 завершил выполнение цели (успех/неудача/отмена)."""
        status = future.result().status
        self._nav_goal_active = False
        self._nav_goal_handle = None

        # 4 = SUCCEEDED по action_msgs/GoalStatus
        if status == 4:
            self._current_wp_idx += 1
            self.get_logger().info(
                f'Waypoint достигнут. Прогресс: '
                f'{self._current_wp_idx}/{len(self._coverage_path.poses)}'
            )
            if self._current_wp_idx >= len(self._coverage_path.poses):
                self._task_complete = True
                self.get_logger().info('Все waypoint пройдены — задание выполнено')
        else:
            self.get_logger().warn(
                f'Waypoint {self._current_wp_idx + 1} не достигнут (status={status})'
            )

    def _cancel_active_goal(self):
        """Отменяем активную цель Nav2 (вызывается при EMERGENCY_STOP и др.)."""
        if self._nav_goal_handle is not None:
            self.get_logger().info('Отменяю активную цель Nav2')
            self._nav_goal_handle.cancel_goal_async()
        self._nav_goal_active = False
        self._nav_goal_handle = None

    # ------------------------------------------------------------------ #
    #  Текстовый маркер статуса над трактором                             #
    # ------------------------------------------------------------------ #

    def _publish_status_marker(self):
        """Публикуем TEXT_VIEW_FACING маркер с информацией о состоянии."""
        if self._odom is not None:
            x = self._odom.pose.pose.position.x
            y = self._odom.pose.pose.position.y
            v = math.hypot(
                self._odom.twist.twist.linear.x,
                self._odom.twist.twist.linear.y,
            )
        else:
            x, y, v = 0.0, 0.0, 0.0

        total_wp = (
            len(self._coverage_path.poses) if self._coverage_path is not None else 0
        )

        marker = Marker()
        marker.header.frame_id = 'base_link'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'robot_status'
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD

        # Над трактором (z = 2.5 м над base_link)
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 2.5
        marker.pose.orientation.w = 1.0

        marker.scale.z = 0.8  # размер текста

        marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)  # белый

        marker.lifetime = Duration(sec=1, nanosec=0)

        marker.text = (
            f'FSM: {self._state.value}\n'
            f'Pos: ({x:.1f}, {y:.1f})\n'
            f'WP: {self._current_wp_idx} / {total_wp}\n'
            f'v: {v:.2f} m/s'
        )

        self._status_marker_pub.publish(marker)

    # ------------------------------------------------------------------ #
    #  Граница поля и зона обработки (MarkerArray, статика)               #
    # ------------------------------------------------------------------ #

    def _publish_field_boundary(self):
        """
        Публикуем два LINE_STRIP-маркера: внешняя граница поля 36×36 м
        и внутренняя зона обработки 32×32 м (с отступом 2 м от края).
        """
        stamp = self.get_clock().now().to_msg()

        # --- Внешний контур: 36×36 м (жёлтый) ---
        outer = Marker()
        outer.header.frame_id = 'map'
        outer.header.stamp = stamp
        outer.ns = 'field_boundary'
        outer.id = 0
        outer.type = Marker.LINE_STRIP
        outer.action = Marker.ADD
        outer.scale.x = 0.1
        outer.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.8)
        outer.pose.orientation.w = 1.0
        outer.points = [
            Point(x=-18.0, y=-18.0, z=0.0),
            Point(x= 18.0, y=-18.0, z=0.0),
            Point(x= 18.0, y= 18.0, z=0.0),
            Point(x=-18.0, y= 18.0, z=0.0),
            Point(x=-18.0, y=-18.0, z=0.0),
        ]

        # --- Внутренний контур: 32×32 м (оранжевый) ---
        inner = Marker()
        inner.header.frame_id = 'map'
        inner.header.stamp = stamp
        inner.ns = 'field_boundary'
        inner.id = 1
        inner.type = Marker.LINE_STRIP
        inner.action = Marker.ADD
        inner.scale.x = 0.05
        inner.color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.8)
        inner.pose.orientation.w = 1.0
        inner.points = [
            Point(x=-16.0, y=-16.0, z=0.0),
            Point(x= 16.0, y=-16.0, z=0.0),
            Point(x= 16.0, y= 16.0, z=0.0),
            Point(x=-16.0, y= 16.0, z=0.0),
            Point(x=-16.0, y=-16.0, z=0.0),
        ]

        self._field_pub.publish(MarkerArray(markers=[outer, inner]))

    # ------------------------------------------------------------------ #
    #  Утилиты                                                            #
    # ------------------------------------------------------------------ #

    def _now(self) -> float:
        """Текущее время в секундах (float)."""
        return self.get_clock().now().nanoseconds * 1e-9


def main(args=None):
    rclpy.init(args=args)
    node = DecisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
