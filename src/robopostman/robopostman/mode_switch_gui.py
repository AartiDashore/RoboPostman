#!/usr/bin/env python3
"""Simple terminal-based mode switcher. Run in a separate terminal."""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ModeSwitcher(Node):
    def __init__(self):
        super().__init__('mode_switcher')
        self.pub = self.create_publisher(String, '/mode_switch', 10)
        self.get_logger().info('Mode Switcher ready.')
        self.create_timer(0.5, self.prompt)
        self._prompted = False

    def prompt(self):
        if not self._prompted:
            self._prompted = True
            print('\n=== MODE SWITCHER ===')
            print('Type "m" + Enter for MANUAL')
            print('Type "a" + Enter for AUTONOMOUS')
            print('Ctrl+C to quit\n')
            import threading
            threading.Thread(target=self._read_input, daemon=True).start()

    def _read_input(self):
        while rclpy.ok():
            try:
                val = input('> ').strip().lower()
                msg = String()
                if val == 'm':
                    msg.data = 'manual'
                    self.pub.publish(msg)
                    print('>> Switched to MANUAL')
                elif val == 'a':
                    msg.data = 'auto'
                    self.pub.publish(msg)
                    print('>> Switched to AUTONOMOUS')
            except EOFError:
                break


def main(args=None):
    rclpy.init(args=args)
    node = ModeSwitcher()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
