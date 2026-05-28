#!/usr/bin/env python3
"""
Scoring and Trail node - fixed for ROS 2 Jazzy.

Fixes applied:
  - Handles 'pothole_penalty' score event (was silently ignored before)
  - Handles 'dog_penalty' correctly (event name now matches spawner)
  - Publishes live counts: coins, fuels, potholes hit, dogs hit, deliveries
  - Publishes /score_display String for any HUD overlay to consume
  - Red-flash Bool on any penalty event
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Int32, Float32, Bool
import math


class ScoringTrail(Node):
    def __init__(self):
        super().__init__('scoring_trail')

        # ── Counters ──────────────────────────────────────────────────────────
        self.score             = 0
        self.deliveries        = 0
        self.coins_collected   = 0
        self.fuels_collected   = 0
        self.potholes_hit      = 0
        self.dogs_hit          = 0

        # ── Point values ──────────────────────────────────────────────────────
        self.DELIVERY_SCORE    =  100
        self.COIN_SCORE        =   20
        self.FUEL_SCORE        =   30
        self.DOG_PENALTY       =  -50
        self.POTHOLE_PENALTY   =  -10
        self.COVERAGE_BONUS    =    2   # per % coverage

        # ── Trail tracking ────────────────────────────────────────────────────
        self.trail_path        = Path()
        self.trail_path.header.frame_id = 'map'
        self.visited_cells     = set()
        self.map_info          = None
        self.last_pos          = None

        # ── Publishers ────────────────────────────────────────────────────────
        self.trail_pub       = self.create_publisher(Path,    '/robot_trail',    10)
        self.score_pub       = self.create_publisher(Int32,   '/total_score',    10)
        self.coverage_pub    = self.create_publisher(Float32, '/coverage_percent', 10)
        self.score_detail_pub = self.create_publisher(String, '/score_detail',   10)
        self.score_display_pub = self.create_publisher(String, '/score_display', 10)
        self.red_flash_pub   = self.create_publisher(Bool,    '/red_flash',      10)

        # ── Subscriptions ─────────────────────────────────────────────────────
        self.create_subscription(Odometry,     '/odom',              self._odom_cb,         10)
        self.create_subscription(String,       '/score_event',       self._score_event_cb,  10)
        self.create_subscription(Int32,        '/deliveries_completed', self._delivery_cb,  10)
        self.create_subscription(OccupancyGrid, '/map',              self._map_cb,          10)

        # ── Timers ────────────────────────────────────────────────────────────
        self.create_timer(0.5, self._publish_trail)
        self.create_timer(1.0, self._publish_score)

        self._flash_timer = None
        self.get_logger().info('ScoringTrail started')

    # ── Odometry / trail ──────────────────────────────────────────────────────
    def _odom_cb(self, msg):
        pos = msg.pose.pose.position
        if self.last_pos:
            d = math.hypot(pos.x - self.last_pos.x, pos.y - self.last_pos.y)
            if d < 0.3:
                return
        self.last_pos = pos
        ps = PoseStamped()
        ps.header.stamp    = self.get_clock().now().to_msg()
        ps.header.frame_id = 'map'
        ps.pose = msg.pose.pose
        self.trail_path.poses.append(ps)
        if self.map_info:
            cx = int((pos.x - self.map_info.origin.position.x) / self.map_info.resolution)
            cy = int((pos.y - self.map_info.origin.position.y) / self.map_info.resolution)
            self.visited_cells.add((cx, cy))

    def _map_cb(self, msg):
        self.map_info = msg.info

    # ── Score events ──────────────────────────────────────────────────────────
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
            self.dogs_hit += 1
            self.score = max(0, self.score + self.DOG_PENALTY)
            self.get_logger().warn(f'Dog penalty {self.DOG_PENALTY} → {self.score}')
            self._trigger_red_flash()

        elif event == 'pothole_penalty':
            self.potholes_hit += 1
            self.score = max(0, self.score + self.POTHOLE_PENALTY)
            self.get_logger().warn(f'Pothole penalty {self.POTHOLE_PENALTY} → {self.score}')
            self._trigger_red_flash()

        else:
            self.get_logger().warn(f'Unknown score event: {event}')

    def _delivery_cb(self, msg):
        self.deliveries = msg.data
        self.score += self.DELIVERY_SCORE
        self.get_logger().info(
            f'Delivery #{self.deliveries} +{self.DELIVERY_SCORE} → {self.score}')

    # ── Flash ─────────────────────────────────────────────────────────────────
    def _trigger_red_flash(self):
        flash = Bool(); flash.data = True
        self.red_flash_pub.publish(flash)
        if self._flash_timer:
            self._flash_timer.cancel()
        self._flash_timer = self.create_timer(1.5, self._clear_flash)

    def _clear_flash(self):
        flash = Bool(); flash.data = False
        self.red_flash_pub.publish(flash)
        if self._flash_timer:
            self._flash_timer.cancel()
            self._flash_timer = None

    # ── Coverage ──────────────────────────────────────────────────────────────
    def _calculate_coverage(self) -> float:
        if not self.map_info:
            return 0.0
        total = self.map_info.width * self.map_info.height
        return min(100.0, len(self.visited_cells) / max(1, total) * 100.0)

    # ── Publishers ────────────────────────────────────────────────────────────
    def _publish_trail(self):
        self.trail_path.header.stamp = self.get_clock().now().to_msg()
        self.trail_pub.publish(self.trail_path)

    def _publish_score(self):
        coverage  = self._calculate_coverage()
        total     = self.score + int(coverage * self.COVERAGE_BONUS)

        msg = Int32();   msg.data = total
        self.score_pub.publish(msg)

        msg2 = Float32(); msg2.data = coverage
        self.coverage_pub.publish(msg2)

        detail = (f'Score={total} | Deliveries={self.deliveries} | '
                  f'Coins={self.coins_collected} | Fuels={self.fuels_collected} | '
                  f'Dogs={self.dogs_hit} | Potholes={self.potholes_hit} | '
                  f'Coverage={coverage:.1f}%')

        d_msg = String(); d_msg.data = detail
        self.score_detail_pub.publish(d_msg)
        self.score_display_pub.publish(d_msg)

        # Also log at reduced rate via the existing logger line
        self.get_logger().info(detail)


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
