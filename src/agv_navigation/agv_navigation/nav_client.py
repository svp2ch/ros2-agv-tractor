"""
Клиент навигации — вспомогательный узел для управления Nav2.

Позволяет отправлять одиночные цели навигации и получать статус.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from nav2_msgs.action import NavigateToPose

import math


class NavClient(Node):
    """
    Клиент для отправки одиночных навигационных целей в Nav2.

    Полезен для тестирования навигации перед запуском полного
    алгоритма покрытия территории.

    Публикации:
        /navigation_status (std_msgs/String): статус навигации

    Экшены:
        /navigate_to_pose: отправка одиночной цели в Nav2
    """

    def __init__(self):
        super().__init__('nav_client')

        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.get_logger().info('Nav client готов. Ожидаю Nav2...')

        # Для теста — отправим трактор в центр поля через 5 секунд
        self.test_timer = self.create_timer(5.0, self._send_test_goal)

    def _send_test_goal(self):
        """Отправляем тестовую цель навигации (центр поля)."""
        self.test_timer.cancel()

        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().warn('Nav2 ещё не готов, пропускаю тест')
            return

        goal = NavigateToPose.Goal()
        goal.pose = self._make_pose(5.0, 5.0, 0.0)

        self.get_logger().info('Отправляю тестовую цель: (5.0, 5.0)')
        future = self._action_client.send_goal_async(
            goal,
            feedback_callback=self._feedback_callback
        )
        future.add_done_callback(self._goal_response_callback)

    def _make_pose(self, x: float, y: float, yaw: float) -> PoseStamped:
        """Создаём PoseStamped из координат."""
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

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Цель отклонена Nav2')
            return
        self.get_logger().info('Цель принята, начинаю навигацию...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        dist = feedback.distance_remaining
        self.get_logger().info(f'Осталось до цели: {dist:.2f}м', throttle_duration_sec=2.0)

    def _result_callback(self, future):
        status = future.result().status
        if status == 4:
            self.get_logger().info('Цель достигнута!')
        else:
            self.get_logger().warn(f'Навигация завершилась со статусом: {status}')


def main(args=None):
    rclpy.init(args=args)
    node = NavClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
