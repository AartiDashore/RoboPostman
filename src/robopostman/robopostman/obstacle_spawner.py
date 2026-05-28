#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Bool
import random, math, subprocess

OBSTACLE_TYPES = {
    'pothole': {
        'shape': 'cylinder', 'radius': 0.4, 'length': 0.05, 'z': 0.025,
        'r': 1.0, 'g': 1.0, 'b': 0.0, 'reward': False,
        'score_event': 'pothole_penalty', 'contact_radius': 1.2,
    },
    'dog': {
        'shape': 'box', 'sx': 0.4, 'sy': 0.25, 'sz': 0.3, 'z': 0.15,
        'r': 1.0, 'g': 0.5, 'b': 0.0, 'reward': False,
        'score_event': 'dog_penalty', 'contact_radius': 1.2,
    },
    'coin': {
        'shape': 'cylinder', 'radius': 0.2, 'length': 0.05, 'z': 0.025,
        'r': 0.0, 'g': 1.0, 'b': 0.0, 'reward': True,
        'score_event': 'coin', 'contact_radius': 1.2,
    },
    'fuel': {
        'shape': 'cylinder', 'radius': 0.15, 'length': 0.4, 'z': 0.2,
        'r': 0.0, 'g': 0.0, 'b': 1.0, 'reward': True,
        'score_event': 'fuel', 'contact_radius': 1.2,
    },
}

MAX_OBSTACLES = 8
MIN_FROM_ROBOT = 2.0
MIN_BETWEEN_OBJECTS = 1.0
STALE_AGE = 60.0
STALE_FAR = 6.0


def _make_sdf(name, obs_type, x, y):
    t = OBSTACLE_TYPES[obs_type]
    r, g, b, z = t['r'], t['g'], t['b'], t['z']
    if t['shape'] == 'cylinder':
        geom = f'<cylinder><radius>{t["radius"]}</radius><length>{t["length"]}</length></cylinder>'
    else:
        geom = f'<box><size>{t["sx"]} {t["sy"]} {t["sz"]}</size></box>'
    return f"""<?xml version='1.0'?>
<sdf version='1.8'>
  <model name='{name}'>
    <static>true</static>
    <pose>{x} {y} {z} 0 0 0</pose>
    <link name='link'>
      <visual name='visual'><geometry>{geom}</geometry>
        <material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse></material>
      </visual>
      <collision name='collision'><geometry>{geom}</geometry></collision>
    </link>
  </model>
</sdf>"""


class ObstacleSpawner(Node):
    def __init__(self):
        super().__init__('obstacle_spawner')
        self.robot_pos = Point()
        self.obstacles = {}
        self.counter = 0
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self.score_event_pub = self.create_publisher(String, '/score_event', 10)
        self.detection_pub = self.create_publisher(String, '/detection_event', 10)
        self.flash_pub = self.create_publisher(Bool, '/object_flash', 10)
        self.create_timer(8.0, self._maybe_spawn)
        self.create_timer(0.2, self._check_contacts)
        self.create_timer(10.0, self._remove_stale)
        self.get_logger().info('ObstacleSpawner ready')

    def _odom_cb(self, msg):
        self.robot_pos = msg.pose.pose.position

    def _maybe_spawn(self):
        if len(self.obstacles) >= MAX_OBSTACLES:
            return
        obs_type = random.choice(list(OBSTACLE_TYPES.keys()))
        for _ in range(30):
            angle = random.uniform(0, 2 * math.pi)
            dist = random.uniform(MIN_FROM_ROBOT + 0.5, 5.0)
            x = self.robot_pos.x + dist * math.cos(angle)
            y = self.robot_pos.y + dist * math.sin(angle)
            if self._is_clear(x, y):
                self._spawn(obs_type, x, y)
                return

    def _is_clear(self, x, y):
        if math.hypot(x - self.robot_pos.x, y - self.robot_pos.y) < MIN_FROM_ROBOT:
            return False
        for info in self.obstacles.values():
            if math.hypot(x - info['x'], y - info['y']) < MIN_BETWEEN_OBJECTS:
                return False
        return True

    def _spawn(self, obs_type, x, y):
        name = f'{obs_type}_{self.counter}'
        self.counter += 1
        sdf = _make_sdf(name, obs_type, x, y)
        sdf_file = f'/tmp/{name}.sdf'
        with open(sdf_file, 'w') as f:
            f.write(sdf)
        try:
            result = subprocess.run(
                ['gz', 'service', '-s', '/world/neighborhood/create',
                 '--reqtype', 'gz.msgs.EntityFactory',
                 '--reptype', 'gz.msgs.Boolean',
                 '--timeout', '2000',
                 '--req', f'sdf_filename: "{sdf_file}"'],
                capture_output=True, text=True, timeout=5)
            success = 'data: true' in result.stdout or result.returncode == 0
        except Exception as e:
            self.get_logger().error(f'Spawn error: {e}')
            return
        if success:
            self.obstacles[name] = {
                'x': x, 'y': y, 'type': obs_type,
                'spawn_time': self.get_clock().now().nanoseconds / 1e9,
                'contacted': False,
            }
            self.get_logger().info(f'Spawned {obs_type} "{name}" at ({x:.1f},{y:.1f})')

    def _check_contacts(self):
        to_remove = []
        for name, info in self.obstacles.items():
            if info['contacted']:
                continue
            t = OBSTACLE_TYPES[info['type']]
            dist = math.hypot(self.robot_pos.x - info['x'], self.robot_pos.y - info['y'])
            if dist < t['contact_radius'] * 3.0:
                det = String(); det.data = info['type']
                self.detection_pub.publish(det)
            if dist < t['contact_radius']:
                info['contacted'] = True
                evt = String(); evt.data = t['score_event']
                self.score_event_pub.publish(evt)
                flash = Bool(); flash.data = True
                self.flash_pub.publish(flash)
                self.get_logger().info(f'Contact: {info["type"]} "{name}" -> {t["score_event"]}')
                to_remove.append(name)
        for name in to_remove:
            self._delete(name)

    def _remove_stale(self):
        now = self.get_clock().now().nanoseconds / 1e9
        to_remove = [
            name for name, info in self.obstacles.items()
            if (not info['contacted']
                and now - info['spawn_time'] > STALE_AGE
                and math.hypot(self.robot_pos.x - info['x'], self.robot_pos.y - info['y']) > STALE_FAR)
        ]
        for name in to_remove:
            self._delete(name)

    def _delete(self, name):
        if name not in self.obstacles:
            return
        del self.obstacles[name]
        try:
            subprocess.run(
                ['gz', 'service', '-s', '/world/neighborhood/remove',
                 '--reqtype', 'gz.msgs.Entity',
                 '--reptype', 'gz.msgs.Boolean',
                 '--timeout', '2000',
                 '--req', f'name: "{name}" type: 2'],
                capture_output=True, text=True, timeout=5)
        except Exception as e:
            self.get_logger().warn(f'Delete error for {name}: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleSpawner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == '__main__':
    main()
