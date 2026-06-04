"""
Планировщик покрытия территории методом бустрофедонного разбиения.

Бустрофедон (от греч. «как вол пашет») — паттерн движения в виде
параллельных рядов с разворотами на концах, как при вспашке поля.
"""

import math
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Point
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from std_msgs.msg import Header, Bool
from nav2_msgs.action import NavigateThroughPoses
from visualization_msgs.msg import Marker, MarkerArray


class CoverageState(Enum):
    """Состояния планировщика покрытия."""
    IDLE = 'idle'
    PLANNING = 'planning'
    EXECUTING = 'executing'
    COMPLETED = 'completed'
    FAILED = 'failed'


# QoS с латчингом — новый подписчик получит последнее опубликованное сообщение
_LATCHING_QOS = QoSProfile(
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)


class CoveragePlanner(Node):
    """
    Узел планировщика покрытия территории (Coverage Path Planning).

    Реализует бустрофедонный алгоритм: поле разбивается на параллельные
    полосы шириной, равной ширине рабочего органа трактора. Трактор
    последовательно проезжает каждую полосу, разворачиваясь на краях.

    Подписки:
        /map (OccupancyGrid): карта поля от slam_toolbox
        /odometry/filtered (Odometry): текущая поза трактора от EKF

    Публикации (transient_local — сообщение сохраняется для поздних подписчиков):
        /coverage_path (Path): вычисленный маршрут покрытия
        /coverage_markers (MarkerArray): визуализация маршрута в RViz2
        /coverage_complete (Bool): сигнал завершения покрытия

    Экшены:
        /navigate_through_poses: отправка маршрута в Nav2
    """

    def __init__(self):
        super().__init__('coverage_planner')

        # --- Параметры ---
        self.declare_parameter('working_width', 2.0)
        self.declare_parameter('robot_radius', 1.5)
        self.declare_parameter('field_x_min', -20.0)
        self.declare_parameter('field_x_max', 20.0)
        self.declare_parameter('field_y_min', -20.0)
        self.declare_parameter('field_y_max', 20.0)
        self.declare_parameter('use_map', False)

        self.working_width = self.get_parameter('working_width').value
        self.robot_radius = self.get_parameter('robot_radius').value
        self.field_x_min = self.get_parameter('field_x_min').value
        self.field_x_max = self.get_parameter('field_x_max').value
        self.field_y_min = self.get_parameter('field_y_min').value
        self.field_y_max = self.get_parameter('field_y_max').value
        self.use_map = self.get_parameter('use_map').value

        self.state = CoverageState.IDLE
        self.current_pose = None
        self.coverage_map = None
        self.waypoints: list[PoseStamped] = []

        # Callback groups:
        # - action_cb_group: ReentrantCallbackGroup для ActionClient (action-колбэки)
        # - timer_cb_group: MutuallyExclusiveCallbackGroup для таймеров
        # Разделение нужно чтобы action-колбэки не блокировали таймеры
        self._action_cb = ReentrantCallbackGroup()
        self._timer_cb = MutuallyExclusiveCallbackGroup()

        # Подписки
        # /odometry/filtered от EKF — работает и при SLAM, и без него
        self.pose_sub = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self._pose_callback,
            10,
        )

        if self.use_map:
            self.map_sub = self.create_subscription(
                OccupancyGrid,
                '/map',
                self._map_callback,
                10,
            )

        # Публикации с transient_local — поздние подписчики получат сохранённое сообщение
        self.path_pub = self.create_publisher(Path, '/coverage_path', _LATCHING_QOS)
        self.marker_pub = self.create_publisher(
            MarkerArray, '/coverage_markers', _LATCHING_QOS
        )
        self.complete_pub = self.create_publisher(Bool, '/coverage_complete', 10)

        # Action client для Nav2
        self._nav_client = ActionClient(
            self,
            NavigateThroughPoses,
            'navigate_through_poses',
            callback_group=self._action_cb,
        )

        # Таймер планирования: срабатывает один раз через 3 секунды — строим маршрут
        self._plan_timer = self.create_timer(
            3.0, self._on_plan_timer, callback_group=self._timer_cb
        )

        # Таймер ожидания Nav2: каждую секунду проверяем готовность action server
        # НЕ блокируем executor — используем server_is_ready() вместо wait_for_server()
        self._nav2_poll_timer = self.create_timer(
            1.0, self._on_nav2_poll_timer, callback_group=self._timer_cb
        )
        self._nav2_poll_timer.cancel()  # Запустим только после построения маршрута

        # Таймер переодической публикации пути для RViz2
        self.create_timer(
            5.0, self._publish_path_visualization, callback_group=self._timer_cb
        )

        self.get_logger().info('Планировщик покрытия инициализирован')
        self.get_logger().info(
            f'Поле: X=[{self.field_x_min}, {self.field_x_max}], '
            f'Y=[{self.field_y_min}, {self.field_y_max}], '
            f'ширина захвата={self.working_width}м'
        )

    # ------------------------------------------------------------------ #
    #  Callbacks подписок                                                  #
    # ------------------------------------------------------------------ #

    def _pose_callback(self, msg: Odometry):
        """Обновляем текущую позу трактора из фильтрованной одометрии."""
        self.current_pose = msg.pose.pose

    def _map_callback(self, msg: OccupancyGrid):
        """Сохраняем карту для учёта препятствий."""
        self.coverage_map = msg

    # ------------------------------------------------------------------ #
    #  Таймеры                                                             #
    # ------------------------------------------------------------------ #

    def _on_plan_timer(self):
        """Однократный таймер: строим маршрут и публикуем его."""
        self._plan_timer.cancel()
        if self.state != CoverageState.IDLE:
            return
        self._build_coverage_path()
        # Публикуем сразу — transient_local гарантирует доставку поздним подписчикам
        self._publish_path_visualization()
        # ВАЖНО: цели в Nav2 НЕ отправляем. Исполнением маршрута занимается
        # decision_node (FSM) через action navigate_to_pose — по одной точке.
        # Если бы coverage_planner тоже слал navigate_through_poses, два узла
        # перебивали бы команды друг друга в одном controller_server.
        # Здесь только строим и публикуем маршрут в /coverage_path.
        # self._nav2_poll_timer.reset()  # отключено намеренно

    def _on_nav2_poll_timer(self):
        """
        Каждую секунду проверяет готовность Nav2 без блокировки executor.

        Вместо blocking wait_for_server() используем server_is_ready() — она
        возвращает результат мгновенно, не блокируя поток.
        """
        if self.state != CoverageState.PLANNING:
            self._nav2_poll_timer.cancel()
            return

        if not self._nav_client.server_is_ready():
            self.get_logger().info(
                'Жду Nav2 (navigate_through_poses)...',
                throttle_duration_sec=5.0,
            )
            return

        # Nav2 готов — отправляем маршрут и останавливаем опрос
        self._nav2_poll_timer.cancel()
        self._dispatch_waypoints()

    # ------------------------------------------------------------------ #
    #  Построение маршрута                                                 #
    # ------------------------------------------------------------------ #

    def _build_coverage_path(self):
        """
        Строим бустрофедонный маршрут покрытия поля.

        Алгоритм:
        1. Разбиваем поле на горизонтальные полосы шириной working_width
        2. Нечётные полосы — движение слева направо (+X)
        3. Чётные полосы — движение справа налево (-X)
        """
        self.get_logger().info('Вычисляю маршрут покрытия (бустрофедон)...')
        self.state = CoverageState.PLANNING

        margin = self.robot_radius + 0.5
        x_min = self.field_x_min + margin
        x_max = self.field_x_max - margin
        y_min = self.field_y_min + margin
        y_max = self.field_y_max - margin

        strip_centers = self._compute_strip_centers(y_min, y_max)

        self.waypoints = []
        for i, y in enumerate(strip_centers):
            if i % 2 == 0:
                x_start, x_end = x_min, x_max
                yaw = 0.0
            else:
                x_start, x_end = x_max, x_min
                yaw = math.pi

            self.waypoints.append(self._make_pose(x_start, y, yaw))
            self.waypoints.append(self._make_pose(x_end, y, yaw))

        total_dist = self._estimate_total_distance()
        self.get_logger().info(
            f'Маршрут готов: {len(self.waypoints)} точек, '
            f'{len(strip_centers)} полос, ~{total_dist:.1f}м'
        )

    def _compute_strip_centers(self, y_min: float, y_max: float) -> list:
        """Вычисляем координаты Y центров обрабатываемых полос."""
        centers = []
        y = y_min + self.working_width / 2.0
        while y <= y_max:
            centers.append(y)
            y += self.working_width
        if centers and centers[-1] < y_max - self.working_width * 0.1:
            centers.append(y_max - self.working_width / 2.0)
        return centers

    def _make_pose(self, x: float, y: float, yaw: float) -> PoseStamped:
        """Создаём PoseStamped из координат и угла поворота."""
        pose = PoseStamped()
        pose.header = Header()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def _estimate_total_distance(self) -> float:
        """Оцениваем общую длину маршрута."""
        if len(self.waypoints) < 2:
            return 0.0
        total = 0.0
        for i in range(1, len(self.waypoints)):
            dx = (self.waypoints[i].pose.position.x
                  - self.waypoints[i - 1].pose.position.x)
            dy = (self.waypoints[i].pose.position.y
                  - self.waypoints[i - 1].pose.position.y)
            total += math.sqrt(dx * dx + dy * dy)
        return total

    # ------------------------------------------------------------------ #
    #  Отправка маршрута в Nav2                                            #
    # ------------------------------------------------------------------ #

    def _dispatch_waypoints(self):
        """Отправляем маршрут в Nav2 через action navigate_through_poses."""
        goal = NavigateThroughPoses.Goal()
        goal.poses = self.waypoints
        goal.behavior_tree = ''

        self.get_logger().info(f'Отправляю {len(self.waypoints)} точек в Nav2...')
        self.state = CoverageState.EXECUTING

        future = self._nav_client.send_goal_async(
            goal,
            feedback_callback=self._nav_feedback_callback,
        )
        # add_done_callback выполнится в action_cb_group,
        # не блокируя таймеры в timer_cb_group
        future.add_done_callback(self._nav_goal_response_callback)

    def _nav_goal_response_callback(self, future):
        """Обрабатываем ответ Nav2 на принятие цели."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Nav2 отклонил маршрут!')
            self.state = CoverageState.FAILED
            return
        self.get_logger().info('Nav2 принял маршрут, трактор начинает движение')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._nav_result_callback)

    def _nav_feedback_callback(self, feedback_msg):
        """Логируем прогресс навигации."""
        feedback = feedback_msg.feedback
        if hasattr(feedback, 'distance_remaining'):
            self.get_logger().info(
                f'Навигация: осталось {feedback.distance_remaining:.1f}м',
                throttle_duration_sec=10.0,
            )

    def _nav_result_callback(self, future):
        """Обрабатываем результат завершения навигации."""
        status = future.result().status
        if status == 4:  # SUCCEEDED
            self.get_logger().info('Покрытие территории ЗАВЕРШЕНО успешно!')
            self.state = CoverageState.COMPLETED
            msg = Bool()
            msg.data = True
            self.complete_pub.publish(msg)
        else:
            self.get_logger().warn(f'Навигация завершилась со статусом: {status}')
            self.state = CoverageState.FAILED

    # ------------------------------------------------------------------ #
    #  Визуализация                                                        #
    # ------------------------------------------------------------------ #

    def _publish_path_visualization(self):
        """Публикуем маршрут для отображения в RViz2."""
        if not self.waypoints:
            return

        # Path — линия маршрута
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.poses = self.waypoints
        self.path_pub.publish(path_msg)

        # MarkerArray — линия + точки разворота
        markers = MarkerArray()

        line = Marker()
        line.header.frame_id = 'map'
        line.header.stamp = self.get_clock().now().to_msg()
        line.ns = 'coverage_path'
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = 0.1
        line.color.r = 0.0
        line.color.g = 0.8
        line.color.b = 0.2
        line.color.a = 0.8
        for wp in self.waypoints:
            p = Point()
            p.x = wp.pose.position.x
            p.y = wp.pose.position.y
            p.z = 0.0
            line.points.append(p)
        markers.markers.append(line)

        for i in range(1, len(self.waypoints), 2):
            wp = self.waypoints[i]
            sphere = Marker()
            sphere.header.frame_id = 'map'
            sphere.header.stamp = self.get_clock().now().to_msg()
            sphere.ns = 'turn_points'
            sphere.id = i
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose = wp.pose
            sphere.scale.x = 0.4
            sphere.scale.y = 0.4
            sphere.scale.z = 0.4
            sphere.color.r = 1.0
            sphere.color.g = 0.5
            sphere.color.b = 0.0
            sphere.color.a = 0.9
            markers.markers.append(sphere)

        self.marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    # MultiThreadedExecutor нужен, чтобы action-колбэки (ReentrantCallbackGroup)
    # и таймеры (MutuallyExclusiveCallbackGroup) работали параллельно
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=2)
    node = CoveragePlanner()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
