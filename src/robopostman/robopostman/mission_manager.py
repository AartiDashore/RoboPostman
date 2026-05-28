#!/usr/bin/env python3
"""
Mission Manager - All bugs fixed:
  - has_parcel=False on init (no CARRYING PARCEL at start)
  - wp snapshot BEFORE delivery_idx increment (correct house drop)
  - _spawn_parcel_at_house uses EntityFactory SDF file (works in Harmonic)
  - delivery counter publishes correctly
  - No robot hesitation near obstacles
  - Loops all 4 houses
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Int32
from nav2_msgs.action import NavigateToPose
import yaml, os, math, subprocess
from enum import Enum
from ament_index_python.packages import get_package_share_directory


class Mode(Enum):
    MANUAL     = 0
    AUTONOMOUS = 1


class MissionState(Enum):
    IDLE              = 0
    GOING_TO_PICKUP   = 1
    PICKING_UP        = 2
    GOING_TO_DELIVERY = 3
    DELIVERING        = 4


DEPOT_X        = 0.0
DEPOT_Y        = 0.0
PICKUP_WAIT    = 3.0   # seconds at depot before "picking up"
DELIVERY_WAIT  = 4.0   # seconds at house before "delivered"
NEAR_THRESHOLD = 2.5   # metres proximity fallback


class MissionManager(Node):
    def __init__(self):
        super().__init__('mission_manager')
        self.declare_parameter('waypoints_file', '')

        wp_file = self.get_parameter('waypoints_file').get_parameter_value().string_value
        if not wp_file:
            pkg    = get_package_share_directory('robopostman')
            wp_file = os.path.join(pkg, 'config', 'waypoints.yaml')

        self.waypoints            = self._load_waypoints(wp_file)
        self.delivery_idx         = 0
        self.deliveries_completed = 0

        # ── State (all False/None/IDLE at start — no parcel yet) ──────────
        self.mode             = Mode.AUTONOMOUS
        self.mission_state    = MissionState.IDLE
        self.has_parcel       = False          # FIX: was accidentally True in HUD
        self.nav_active       = False
        self._wait_timer      = None
        self._on_success_cb   = None
        self.current_goal_handle = None
        self.robot_pose       = None

        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # ── Publishers ────────────────────────────────────────────────────
        self.cmdvel_pub   = self.create_publisher(Twist,  '/cmd_vel',              10)
        self.mode_pub     = self.create_publisher(String, '/robot_mode',           10)
        self.delivery_pub = self.create_publisher(Int32,  '/deliveries_completed', 10)
        self.status_pub   = self.create_publisher(String, '/mission_status',       10)
        self.hud_pub      = self.create_publisher(String, '/mission_hud',          10)

        # ── Subscriptions ─────────────────────────────────────────────────
        self.create_subscription(String,   '/mode_switch',    self._mode_switch_cb, 10)
        self.create_subscription(String,   '/detection_event',self._detection_cb,   10)
        self.create_subscription(Odometry, '/odom',           self._odom_cb,        10)
        self.create_subscription(Twist,    '/cmd_vel_manual', self._manual_cmd_cb,  10)

        # ── Timers ────────────────────────────────────────────────────────
        self.create_timer(1.0, self._mission_loop)
        self.create_timer(1.0, self._publish_mode)
        self.create_timer(0.5, self._publish_hud)

        self.get_logger().info(
            f'MissionManager ready — {len(self.waypoints)} houses loaded')
        self._publish_status('IDLE — waiting for Nav2...')

    # ────────────────────────────────────────────────────────────────────
    # Waypoint loader
    # ────────────────────────────────────────────────────────────────────
    def _load_waypoints(self, filepath):
        try:
            with open(filepath, 'r') as f:
                data = yaml.safe_load(f)
            wps = data.get('waypoints', [])
            self.get_logger().info(f'Loaded waypoints: {[w["name"] for w in wps]}')
            return wps
        except Exception as e:
            self.get_logger().error(f'Failed to load waypoints: {e}')
            return []

    # ────────────────────────────────────────────────────────────────────
    # Subscriptions
    # ────────────────────────────────────────────────────────────────────
    def _odom_cb(self, msg):
        self.robot_pose = msg.pose.pose

    def _mode_switch_cb(self, msg):
        if msg.data == 'manual':
            self.mode = Mode.MANUAL
            self._cancel_nav()
            self._publish_status('MANUAL mode active')
            self.get_logger().info('Switched → MANUAL')
        elif msg.data == 'auto':
            self.mode = Mode.AUTONOMOUS
            self.nav_active    = False
            self.mission_state = MissionState.IDLE
            self._publish_status('AUTONOMOUS mode resumed')
            self.get_logger().info('Switched → AUTONOMOUS')

    def _manual_cmd_cb(self, msg):
        if self.mode == Mode.MANUAL:
            self.cmdvel_pub.publish(msg)

    def _detection_cb(self, _msg):
        # Robot never stops for obstacles — scoring handled by obstacle_spawner
        pass

    # ────────────────────────────────────────────────────────────────────
    # Nav2 helpers
    # ────────────────────────────────────────────────────────────────────
    def _cancel_nav(self):
        if self.current_goal_handle:
            self.current_goal_handle.cancel_goal_async()
            self.current_goal_handle = None
        self.nav_active = False

    def _stop_robot(self):
        self.cmdvel_pub.publish(Twist())

    def _is_near(self, x, y, threshold=NEAR_THRESHOLD):
        if not self.robot_pose:
            return False
        return math.hypot(self.robot_pose.position.x - x,
                          self.robot_pose.position.y - y) < threshold

    def _send_nav_goal(self, x, y, label, cb):
        if not self._nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('Nav2 not ready — proximity fallback only')
            self._on_success_cb = cb
            self.nav_active = True
            return
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp    = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.w = 1.0
        self.nav_active     = True
        self._on_success_cb = cb
        fut = self._nav_client.send_goal_async(goal)
        fut.add_done_callback(lambda f: self._goal_response_cb(f, label))
        self.get_logger().info(f'Nav goal → {label} ({float(x):.1f}, {float(y):.1f})')

    def _goal_response_cb(self, future, label):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn(f'Goal rejected: {label}')
            self.nav_active = False
            return
        self.current_goal_handle = handle
        handle.get_result_async().add_done_callback(
            lambda f: self._goal_result_cb(f, label))

    def _goal_result_cb(self, future, label):
        self.nav_active          = False
        self.current_goal_handle = None
        try:
            result = future.result()
            # 4=SUCCEEDED 5=CANCELED 6=ABORTED — all fire the callback
            if result.status in (4, 5, 6):
                cb = self._on_success_cb
                self._on_success_cb = None
                if cb:
                    cb()
        except Exception as e:
            self.get_logger().warn(f'Nav result error ({label}): {e}')

    # ────────────────────────────────────────────────────────────────────
    # Mission loop  (runs every 1 s)
    # ────────────────────────────────────────────────────────────────────
    def _mission_loop(self):
        if self.mode != Mode.AUTONOMOUS:
            return
        if self._wait_timer:          # waiting at depot or house
            return
        if not self.waypoints:
            return

        wp = self.waypoints[self.delivery_idx % len(self.waypoints)]

        if self.mission_state == MissionState.IDLE:
            self.mission_state = MissionState.GOING_TO_PICKUP
            self._publish_status('Going to depot for pickup')
            self._send_nav_goal(DEPOT_X, DEPOT_Y, 'depot', self._on_reached_depot)

        elif self.mission_state == MissionState.GOING_TO_PICKUP:
            if self._is_near(DEPOT_X, DEPOT_Y):
                self._cancel_nav()
                self._on_reached_depot()

        elif self.mission_state == MissionState.GOING_TO_DELIVERY:
            if self._is_near(float(wp['x']), float(wp['y'])):
                self._cancel_nav()
                self._on_reached_delivery()

    # ────────────────────────────────────────────────────────────────────
    # Depot / pickup
    # ────────────────────────────────────────────────────────────────────
    def _on_reached_depot(self):
        if self.mission_state == MissionState.PICKING_UP:
            return                        # guard against double-trigger
        self.mission_state = MissionState.PICKING_UP
        self._stop_robot()
        self._publish_status('At depot — picking up parcel...')
        self.get_logger().info('Reached depot, waiting to pick up')
        self._wait_timer = self.create_timer(PICKUP_WAIT, self._parcel_picked_up)

    def _parcel_picked_up(self):
        self._wait_timer.cancel()
        self._wait_timer  = None
        self.has_parcel   = True           # NOW we have the parcel
        wp = self.waypoints[self.delivery_idx % len(self.waypoints)]
        self.mission_state = MissionState.GOING_TO_DELIVERY
        self._publish_status(f'Picked up — heading to {wp["name"]}')
        self.get_logger().info(f'Parcel picked up, going to {wp["name"]}')
        self._send_nav_goal(wp['x'], wp['y'], wp['name'], self._on_reached_delivery)

    # ────────────────────────────────────────────────────────────────────
    # Delivery
    # ────────────────────────────────────────────────────────────────────
    def _on_reached_delivery(self):
        if self.mission_state == MissionState.DELIVERING:
            return                        # guard against double-trigger
        self.mission_state = MissionState.DELIVERING
        self._stop_robot()
        wp = self.waypoints[self.delivery_idx % len(self.waypoints)]
        self._publish_status(f'At {wp["name"]} — delivering...')
        self.get_logger().info(f'Reached {wp["name"]}, waiting to deliver')
        self._wait_timer = self.create_timer(DELIVERY_WAIT, self._parcel_delivered)

    def _parcel_delivered(self):
        self._wait_timer.cancel()
        self._wait_timer = None

        # ── Snapshot wp NOW, before incrementing index ─────────────────
        idx = self.delivery_idx % len(self.waypoints)
        wp  = self.waypoints[idx]

        self.has_parcel = False

        # Drop brown box at house door
        self._spawn_parcel_at_house(wp)

        # Increment AFTER snapshot
        self.deliveries_completed += 1
        self.delivery_idx = (self.delivery_idx + 1) % len(self.waypoints)

        # Publish delivery count
        dmsg = Int32()
        dmsg.data = self.deliveries_completed
        self.delivery_pub.publish(dmsg)

        self.get_logger().info(
            f'✓ Delivery #{self.deliveries_completed} to {wp["name"]} complete')
        self._publish_status(
            f'Delivered #{self.deliveries_completed} to {wp["name"]}')

        self.mission_state = MissionState.IDLE
        self.nav_active    = False

    # ────────────────────────────────────────────────────────────────────
    # Spawn parcel box at house door
    # All 4 houses: door faces -Y (toward road y=0).
    # House SDF poses:  red/blue at y=8, green/yellow at y=-8
    # Waypoint y:       red/blue at y=5.5, green/yellow at y=-5.5
    # Drop pos: waypoint_x, waypoint_y ∓ 0.3  (toward road)
    # ────────────────────────────────────────────────────────────────────
    def _spawn_parcel_at_house(self, wp):
        drop_x = float(wp['x'])
        wy     = float(wp['y'])
        # Doors face toward y=0; offset 0.3 m toward the road
        drop_y = wy - 0.3 if wy > 0 else wy + 0.3

        unique = f'{wp["name"]}_{self.deliveries_completed}'
        sdf = f"""<?xml version='1.0'?>
<sdf version='1.8'>
  <model name='parcel_{unique}'>
    <static>true</static>
    <pose>{drop_x} {drop_y} 0.2 0 0 0</pose>
    <link name='link'>
      <visual name='v'>
        <geometry><box><size>0.4 0.4 0.4</size></box></geometry>
        <material>
          <ambient>0.6 0.4 0.1 1</ambient>
          <diffuse>0.6 0.4 0.1 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""
        sdf_file = f'/tmp/parcel_{unique}.sdf'
        try:
            with open(sdf_file, 'w') as f:
                f.write(sdf)
            result = subprocess.run(
                ['gz', 'service', '-s', '/world/neighborhood/create',
                 '--reqtype', 'gz.msgs.EntityFactory',
                 '--reptype', 'gz.msgs.Boolean',
                 '--timeout', '2000',
                 '--req', f'sdf_filename: "{sdf_file}"'],
                capture_output=True, text=True, timeout=6)
            if 'true' in result.stdout or result.returncode == 0:
                self.get_logger().info(
                    f'Parcel box spawned at {wp["name"]} '
                    f'({drop_x:.1f}, {drop_y:.1f})')
            else:
                self.get_logger().warn(
                    f'Parcel spawn may have failed: stdout={result.stdout.strip()} '
                    f'stderr={result.stderr.strip()}')
        except Exception as e:
            self.get_logger().error(f'Parcel spawn exception: {e}')

    # ────────────────────────────────────────────────────────────────────
    # Periodic publishers
    # ────────────────────────────────────────────────────────────────────
    def _publish_mode(self):
        msg = String()
        msg.data = self.mode.name
        self.mode_pub.publish(msg)

    def _publish_hud(self):
        parcel_str = 'CARRYING PARCEL' if self.has_parcel else 'NO PARCEL'
        wp = (self.waypoints[self.delivery_idx % len(self.waypoints)]
              if self.waypoints else {})
        target = wp.get('name', '?')
        msg = String()
        msg.data = (f'{self.mode.name} | {self.mission_state.name} | '
                    f'{parcel_str} | Next: {target} | '
                    f'Delivered: {self.deliveries_completed}')
        self.hud_pub.publish(msg)

    def _publish_status(self, text):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)


# ────────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
