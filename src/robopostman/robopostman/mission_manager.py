#!/usr/bin/env python3
"""
Mission Manager - Core delivery functionality:
1. Robot starts at depot (origin)
2. Navigates to parcel pickup point
3. Picks up parcel (waits 3s)
4. Navigates to delivery house
5. Delivers parcel (waits 3s)
6. Returns to depot for next parcel
7. Handles obstacles: stops for dog, slows for pothole
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Int32, Bool
from nav2_msgs.action import NavigateToPose
import yaml, os, math
from enum import Enum
from ament_index_python.packages import get_package_share_directory


class Mode(Enum):
    MANUAL = 0
    AUTONOMOUS = 1


class MissionState(Enum):
    IDLE = 0
    GOING_TO_PICKUP = 1
    PICKING_UP = 2
    GOING_TO_DELIVERY = 3
    DELIVERING = 4
    RETURNING = 5


# Parcel pickup location (depot/post office)
DEPOT_X = 0.0
DEPOT_Y = 0.0
PICKUP_WAIT = 3.0    # seconds to simulate picking up parcel
DELIVERY_WAIT = 3.0  # seconds to simulate delivering parcel


class MissionManager(Node):
    def __init__(self):
        super().__init__('mission_manager')
        self.declare_parameter('waypoints_file', '')
        waypoints_file = self.get_parameter('waypoints_file').get_parameter_value().string_value
        if not waypoints_file:
            pkg = get_package_share_directory('robopostman')
            waypoints_file = os.path.join(pkg, 'config', 'waypoints.yaml')

        self.waypoints = self._load_waypoints(waypoints_file)
        self.delivery_idx = 0
        self.deliveries_completed = 0

        self.mode = Mode.AUTONOMOUS
        self.mission_state = MissionState.IDLE
        self.nav_active = False
        self.stopped_for_obstacle = False
        self.slow_for_pothole = False
        self._resume_timer = None
        self._speed_timer = None
        self._wait_timer = None
        self.current_goal_handle = None
        self.robot_pose = None
        self.has_parcel = False

        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.mode_pub = self.create_publisher(String, '/robot_mode', 10)
        self.delivery_pub = self.create_publisher(Int32, '/deliveries_completed', 10)
        self.status_pub = self.create_publisher(String, '/mission_status', 10)
        # HUD display topic
        self.hud_pub = self.create_publisher(String, '/mission_hud', 10)

        self.create_subscription(String, '/mode_switch', self._mode_switch_cb, 10)
        self.create_subscription(String, '/detection_event', self._detection_cb, 10)
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self.create_subscription(Twist, '/cmd_vel_manual', self._manual_cmd_cb, 10)

        self.create_timer(2.0, self._mission_loop)
        self.create_timer(1.0, self._publish_mode)
        self.create_timer(1.0, self._publish_hud)

        self.get_logger().info(
            f'Mission Manager ready. {len(self.waypoints)} delivery houses loaded.')
        self._publish_status('IDLE - waiting for Nav2...')

    def _load_waypoints(self, filepath):
        try:
            with open(filepath, 'r') as f:
                data = yaml.safe_load(f)
            return data.get('waypoints', [])
        except Exception as e:
            self.get_logger().error(f'Failed to load waypoints: {e}')
            return []

    def _odom_cb(self, msg):
        self.robot_pose = msg.pose.pose

    def _mode_switch_cb(self, msg):
        if msg.data == 'manual':
            self.mode = Mode.MANUAL
            self._cancel_nav()
            self.get_logger().info('→ MANUAL mode')
        elif msg.data == 'auto':
            self.mode = Mode.AUTONOMOUS
            self.nav_active = False
            self.mission_state = MissionState.IDLE
            self.get_logger().info('→ AUTONOMOUS mode')

    def _manual_cmd_cb(self, msg):
        if self.mode == Mode.MANUAL:
            self.cmd_vel_pub.publish(msg)

    def _cancel_nav(self):
        if self.current_goal_handle:
            self.current_goal_handle.cancel_goal_async()
            self.current_goal_handle = None
        self.nav_active = False

    def _detection_cb(self, msg):
        event = msg.data
        if event == 'dog' and not self.stopped_for_obstacle:
            self.get_logger().warn('Dog detected - stopping!')
            self.stopped_for_obstacle = True
            self._stop_robot()
            self._cancel_nav()
            self._set_resume_timer(4.0)
        elif event == 'pothole' and not self.slow_for_pothole:
            self.get_logger().info('Pothole - slowing down')
            self.slow_for_pothole = True
            self._set_speed_timer(5.0)

    def _set_resume_timer(self, secs):
        if self._resume_timer:
            self._resume_timer.cancel()
        self._resume_timer = self.create_timer(secs, self._resume_from_stop)

    def _set_speed_timer(self, secs):
        if self._speed_timer:
            self._speed_timer.cancel()
        self._speed_timer = self.create_timer(secs, self._resume_speed)

    def _stop_robot(self):
        self.cmd_vel_pub.publish(Twist())

    def _resume_from_stop(self):
        if self._resume_timer:
            self._resume_timer.cancel()
            self._resume_timer = None
        self.stopped_for_obstacle = False
        self.nav_active = False
        self.get_logger().info('Resuming after obstacle')

    def _resume_speed(self):
        if self._speed_timer:
            self._speed_timer.cancel()
            self._speed_timer = None
        self.slow_for_pothole = False
        self.get_logger().info('Resuming normal speed')

    def _publish_mode(self):
        msg = String(); msg.data = self.mode.name
        self.mode_pub.publish(msg)

    def _publish_hud(self):
        """Publish HUD info - visible via rqt_console or custom overlay."""
        parcel_status = 'CARRYING PARCEL' if self.has_parcel else 'NO PARCEL'
        if self.delivery_idx < len(self.waypoints):
            target = self.waypoints[self.delivery_idx]['name']
        else:
            target = 'ALL DONE'
        msg = String()
        msg.data = (f'MODE:{self.mode.name} | STATE:{self.mission_state.name} | '
                    f'{parcel_status} | TARGET:{target} | '
                    f'DELIVERIES:{self.deliveries_completed}')
        self.hud_pub.publish(msg)

    def _publish_status(self, text):
        msg = String(); msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(f'[MISSION] {text}')

    def _mission_loop(self):
        if self.mode != Mode.AUTONOMOUS:
            return
        if self.stopped_for_obstacle or self.nav_active:
            return
        if self._wait_timer is not None:
            return
        if not self.waypoints:
            return

        if not self._nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('Nav2 not available yet')
            return

        # Cycle through all houses then restart
        if self.delivery_idx >= len(self.waypoints):
            self.delivery_idx = 0
            self._publish_status('All deliveries complete! Starting new cycle.')

        wp = self.waypoints[self.delivery_idx]

        if self.mission_state == MissionState.IDLE:
            # Step 1: go pick up parcel at depot
            self._publish_status(f'Going to DEPOT to pick up parcel for {wp["name"]}')
            self.mission_state = MissionState.GOING_TO_PICKUP
            self._send_nav_goal(DEPOT_X, DEPOT_Y, 'depot',
                                self._on_reached_depot)

        elif self.mission_state == MissionState.GOING_TO_DELIVERY:
            # Already handled by callback - safety fallback
            pass

    def _on_reached_depot(self):
        """Arrived at depot - simulate picking up parcel."""
        self.mission_state = MissionState.PICKING_UP
        self._publish_status('At depot - picking up parcel...')
        self._stop_robot()
        self._wait_timer = self.create_timer(PICKUP_WAIT, self._parcel_picked_up)

    def _parcel_picked_up(self):
        if self._wait_timer:
            self._wait_timer.cancel()
            self._wait_timer = None
        self.has_parcel = True
        wp = self.waypoints[self.delivery_idx]
        self._publish_status(f'Parcel loaded! Delivering to {wp["name"]}')
        self.mission_state = MissionState.GOING_TO_DELIVERY
        self._send_nav_goal(wp['x'], wp['y'], wp['name'],
                            self._on_reached_delivery)

    def _on_reached_delivery(self):
        """Arrived at delivery house - simulate delivering."""
        self.mission_state = MissionState.DELIVERING
        wp = self.waypoints[self.delivery_idx]
        self._publish_status(f'At {wp["name"]} - delivering parcel...')
        self._stop_robot()
        self._wait_timer = self.create_timer(DELIVERY_WAIT, self._parcel_delivered)

    def _parcel_delivered(self):
        if self._wait_timer:
            self._wait_timer.cancel()
            self._wait_timer = None
        self.has_parcel = False
        self.deliveries_completed += 1
        wp = self.waypoints[self.delivery_idx]
        self._publish_status(
            f'Delivered to {wp["name"]}! Total: {self.deliveries_completed}')
        msg = Int32(); msg.data = self.deliveries_completed
        self.delivery_pub.publish(msg)
        self.delivery_idx += 1
        self.mission_state = MissionState.IDLE
        self.nav_active = False

    def _send_nav_goal(self, x, y, name, on_success_cb):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.w = 1.0
        self.nav_active = True
        self._on_success_cb = on_success_cb
        f = self._nav_client.send_goal_async(goal)
        f.add_done_callback(lambda fut: self._goal_response_cb(fut, name))

    def _goal_response_cb(self, future, name):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn(f'Goal rejected for {name}')
            self.nav_active = False
            self.mission_state = MissionState.IDLE
            return
        self.current_goal_handle = handle
        handle.get_result_async().add_done_callback(
            lambda f: self._goal_result_cb(f, name))

    def _goal_result_cb(self, future, name):
        self.nav_active = False
        self.current_goal_handle = None
        result = future.result()
        if result.status == 4:  # SUCCEEDED
            self.get_logger().info(f'Reached {name}')
            if self._on_success_cb:
                cb = self._on_success_cb
                self._on_success_cb = None
                cb()
        else:
            self.get_logger().warn(f'Navigation to {name} failed (status={result.status}), retrying...')
            self.mission_state = MissionState.IDLE


def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
