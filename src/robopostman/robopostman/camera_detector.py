#!/usr/bin/env python3
"""
Camera Detector Node
Uses OpenCV + HSV filtering to detect colored obstacles from camera feed.
Publishes detection events.
"""
 
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
 
 
# HSV ranges for each obstacle color
COLOR_RANGES = {
    'orange_dog': {
        'lower': np.array([10, 100, 100]),
        'upper': np.array([25, 255, 255]),
        'event': 'dog',
    },
    'yellow_pothole': {
        'lower': np.array([25, 100, 100]),
        'upper': np.array([35, 255, 255]),
        'event': 'pothole',
    },
    'green_coin': {
        'lower': np.array([40, 100, 100]),
        'upper': np.array([80, 255, 255]),
        'event': 'coin',
    },
    'blue_fuel': {
        'lower': np.array([100, 150, 100]),
        'upper': np.array([130, 255, 255]),
        'event': 'fuel',
    },
}
 
MIN_CONTOUR_AREA = 500
 
 
class CameraDetector(Node):
    def __init__(self):
        super().__init__('camera_detector')
        self.bridge = CvBridge()
 
        self.detection_pub = self.create_publisher(String, '/detection_event', 10)
 
        self.create_subscription(Image, '/camera/image_raw', self._image_cb, 10)
 
        # Cooldown: avoid spamming events
        self.last_detected = {}
        self.cooldown_sec = 2.0
 
        self.get_logger().info('Camera Detector started')
 
    def _image_cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Image conversion failed: {e}')
            return
 
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        now = self.get_clock().now().nanoseconds / 1e9
 
        for label, cfg in COLOR_RANGES.items():
            mask = cv2.inRange(hsv, cfg['lower'], cfg['upper'])
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
 
            for cnt in contours:
                if cv2.contourArea(cnt) > MIN_CONTOUR_AREA:
                    event = cfg['event']
                    last = self.last_detected.get(event, 0)
                    if now - last > self.cooldown_sec:
                        self.last_detected[event] = now
                        msg_out = String()
                        msg_out.data = event
                        self.detection_pub.publish(msg_out)
                        self.get_logger().info(f'Detected: {event}')
                    break
 
 
def main(args=None):
    rclpy.init(args=args)
    node = CameraDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
 
 
if __name__ == '__main__':
    main()
