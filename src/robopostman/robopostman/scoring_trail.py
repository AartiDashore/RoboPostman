#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Int32, Float32, Bool
import math


class ScoringTrail(Node):
    def __init__(self):
        super().__init__('scoring_trail')

        self.score = 0
        self.deliveries = 0
        self.coins_collected = 0
        self.fuels_collected = 0
        self.dog_penalties = 0

        self.DELIVERY_SCORE = 100
        self.COIN_SCORE = 20
        self.FUEL_SCORE = 30
        self.DOG_PENALTY = -50
        self.COVERAGE_BONUS = 2

        self.trail_path = Path()
        self.trail_path.header.frame_id = 'map'
        self.visited_cells = set()
        self.map_info = None
        self.last_pos = None

        self.trail_pub = self.create_publisher(Path, '/robot_trail', 10)
        self.score_pub = self.create_publisher(Int32, '/total_score', 10)
        self.coverage_pub = self.create_publisher(Float32, '/coverage_percent', 10)
        self.score_detail_pub = self.create_publisher(String, '/score_detail', 10)
        # Red flash signal for Gazebo overlay or RQT
        self.red_flash_pub = self.create_publisher(Bool, '/red_flash', 10)

        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self.create_subscription(String, '/score_event', self._score_event_cb, 10)
        self.create_subscription(Int32, '/deliveries_completed', self._delivery_cb, 10)
        self.create_subscription(OccupancyGrid, '/map', self._map_cb, 10)

        self.create_timer(0.5, self._publish_trail)
        self.create_timer(2.0, self._publish_score)
        self._flash_active = False
        self._flash_timer = None

        self.get_logger().info('Scoring and Trail node started')

    def _odom_cb(self, msg):
        pos = msg.pose.pose.position
        if self.last_pos:
            d = math.sqrt((pos.x-self.last_pos.x)**2 + (pos.y-self.last_pos.y)**2)
            if d < 0.3:
                return
        self.last_pos = pos
        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = 'map'
        ps.pose = msg.pose.pose
        self.trail_path.poses.append(ps)
        if self.map_info:
            cx = int((pos.x - self.map_info.origin.position.x) / self.map_info.resolution)
            cy = int((pos.y - self.map_info.origin.position.y) / self.map_info.resolution)
            self.visited_cells.add((cx, cy))

    def _map_cb(self, msg):
        self.map_info = msg.info

    def _score_event_cb(self, msg):
        event = msg.data
        if event == 'coin':
            self.coins_collected += 1
            self.score += self.COIN_SCORE
            self.get_logger().info(f'Coin +{self.COIN_SCORE} → {self.score}')
        elif event == 'fuel':
            self.fuels_collected += 1
            self.score += self.FUEL_SCORE
            self.get_logger().info(f'Fuel +{self.FUEL_SCORE} → {self.score}')
        elif event == 'dog_penalty':
            self.dog_penalties += 1
            self.score = max(0, self.score + self.DOG_PENALTY)
            self.get_logger().warn(f'Dog penalty {self.DOG_PENALTY} → {self.score}')
            self._trigger_red_flash()

    def _trigger_red_flash(self):
        """Publish red flash signal - can be read by any overlay tool."""
        msg = Bool(); msg.data = True
        self.red_flash_pub.publish(msg)
        if self._flash_timer:
            self._flash_timer.cancel()
        self._flash_timer = self.create_timer(1.5, self._clear_flash)

    def _clear_flash(self):
        msg = Bool(); msg.data = False
        self.red_flash_pub.publish(msg)
        if self._flash_timer:
            self._flash_timer.cancel()
            self._flash_timer = None

    def _delivery_cb(self, msg):
        self.deliveries = msg.data
        self.score += self.DELIVERY_SCORE
        self.get_logger().info(f'Delivery #{self.deliveries} +{self.DELIVERY_SCORE} → {self.score}')

    def _calculate_coverage(self):
        if not self.map_info:
            return 0.0
        total = self.map_info.width * self.map_info.height
        return min(100.0, (len(self.visited_cells) / max(1, total)) * 100.0)

    def _publish_trail(self):
        self.trail_path.header.stamp = self.get_clock().now().to_msg()
        self.trail_pub.publish(self.trail_path)

    def _publish_score(self):
        coverage = self._calculate_coverage()
        total = self.score + int(coverage * self.COVERAGE_BONUS)
        msg = Int32(); msg.data = total
        self.score_pub.publish(msg)
        msg2 = Float32(); msg2.data = coverage
        self.coverage_pub.publish(msg2)
        detail = String()
        detail.data = (f'Score={total} | Deliveries={self.deliveries} | '
                       f'Coins={self.coins_collected} | Fuels={self.fuels_collected} | '
                       f'DogPenalties={self.dog_penalties} | Coverage={coverage:.1f}%')
        self.score_detail_pub.publish(detail)
        self.get_logger().info(detail.data)


def main(args=None):
    rclpy.init(args=args)
    node = ScoringTrail()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
