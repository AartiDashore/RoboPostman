#!/usr/bin/env python3
"""
Obstacle Spawner Node - Gazebo Harmonic compatible
Uses gz service calls via subprocess since ros_gz spawn service
is not available in all configurations.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from std_msgs.msg import String

import random
import math
import subprocess
import json


OBSTACLE_TYPES = {
    'pothole': {
        'shape': 'cylinder',
        'radius': 0.4,
        'length': 0.05,
        'z': 0.025,
        'r': 1.0, 'g': 1.0, 'b': 0.0,
        'event': 'pothole',
        'reward': False,
    },
    'dog': {
        'shape': 'box',
        'sx': 0.4, 'sy': 0.25, 'sz': 0.3,
        'z': 0.15,
        'r': 1.0, 'g': 0.5, 'b': 0.0,
        'event': 'dog',
        'reward': False,
    },
    'coin': {
        'shape': 'cylinder',
        'radius': 0.2,
        'length': 0.05,
        'z': 0.025,
        'r': 0.0, 'g': 1.0, 'b': 0.0,
        'event': 'coin',
        'reward': True,
    },
    'fuel': {
        'shape': 'cylinder',
        'radius': 0.15,
        'length': 0.4,
        'z': 0.2,
        'r': 0.0, 'g': 0.0, 'b': 1.0,
        'event': 'fuel',
        'reward': True,
    },
}

SPAWN_ZONES = [
    (-4.0, -2.0, -1.0, 2.0),
    (1.0,  -2.0,  4.0, 2.0),
    (-4.0,  1.0,  4.0, 4.0),
    (-4.0, -4.0,  4.0, -1.0),
]


def make_sdf(name, obs_type, x, y):
    t = OBSTACLE_TYPES[obs_type]
    r, g, b = t['r'], t['g'], t['b']
    z = t['z']

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
      <visual name='visual'>
        <geometry>{geom}</geometry>
        <material>
          <ambient>{r} {g} {b} 1</ambient>
          <diffuse>{r} {g} {b} 1</diffuse>
        </material>
      </visual>
      <collision name='collision'>
        <geometry>{geom}</geometry>
      </collision>
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
        self.detection_pub = self.create_publisher(String, '/detection_event', 10)
        self.score_event_pub = self.create_publisher(String, '/score_event', 10)

        self.spawn_timer = self.create_timer(15.0, self._spawn_obstacle)
        self.check_timer = self.create_timer(1.0, self._check_obstacles)

        self.get_logger().info('Obstacle Spawner started (gz subprocess mode)')

    def _odom_cb(self, msg):
        self.robot_pos = msg.pose.pose.position

    def _distance_to_robot(self, x, y):
        dx = x - self.robot_pos.x
        dy = y - self.robot_pos.y
        return math.sqrt(dx*dx + dy*dy)

    def _spawn_obstacle(self):
        obs_type = random.choice(list(OBSTACLE_TYPES.keys()))
        name = f'{obs_type}_{self.counter}'
        self.counter += 1

        zone = random.choice(SPAWN_ZONES)
        x = random.uniform(zone[0], zone[2])
        y = random.uniform(zone[1], zone[3])

        sdf = make_sdf(name, obs_type, x, y)
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
                capture_output=True, text=True, timeout=5
            )
            if 'data: true' in result.stdout or result.returncode == 0:
                self.obstacles[name] = {
                    'x': x, 'y': y,
                    'type': obs_type,
                    'spawn_time': self.get_clock().now().nanoseconds / 1e9,
                    'collected': False,
                }
                self.get_logger().info(f'Spawned {obs_type} "{name}" at ({x:.1f},{y:.1f})')
            else:
                self.get_logger().warn(f'Spawn failed: {result.stderr}')
        except Exception as e:
            self.get_logger().error(f'Spawn error: {e}')

    def _check_obstacles(self):
        now = self.get_clock().now().nanoseconds / 1e9
        to_remove = []

        for name, info in self.obstacles.items():
            if info['collected']:
                to_remove.append(name)
                continue

            dist = self._distance_to_robot(info['x'], info['y'])
            obs_type = info['type']
            age = now - info['spawn_time']
            t = OBSTACLE_TYPES[obs_type]

            if dist < 3.0:
                event = String()
                event.data = obs_type
                self.detection_pub.publish(event)

            if dist < 0.8 and t['reward']:
                info['collected'] = True
                score_msg = String()
                score_msg.data = obs_type
                self.score_event_pub.publish(score_msg)
                to_remove.append(name)
                self.get_logger().info(f'Collected {obs_type}: {name}')

            if not t['reward'] and age > 30.0 and dist < 2.0:
                to_remove.append(name)
                self.get_logger().info(f'Auto-removing {name}')

        for name in set(to_remove):
            self._delete_obstacle(name)

    def _delete_obstacle(self, name):
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
                capture_output=True, text=True, timeout=5
            )
        except Exception as e:
            self.get_logger().warn(f'Delete error: {e}')


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
