#!/usr/bin/env python3
"""
Keyboard Teleop Node
Arrow keys for manual control. Tab to switch modes.
"""
 
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import sys
import termios
import tty
import select
 
 
INSTRUCTIONS = """
RoboPostman Keyboard Control
-----------------------------
Arrow Up    : Forward
Arrow Down  : Backward
Arrow Left  : Turn Left
Arrow Right : Turn Right
Tab         : Toggle Manual/Auto mode
s           : Stop
q           : Quit
"""
 
KEY_UP = '\x1b[A'
KEY_DOWN = '\x1b[B'
KEY_RIGHT = '\x1b[C'
KEY_LEFT = '\x1b[D'
KEY_TAB = '\t'
 
LINEAR_SPEED = 0.3
ANGULAR_SPEED = 1.0
 
 
def get_key(timeout=0.1):
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)
            if key == '\x1b':
                extra = sys.stdin.read(2)
                key = key + extra
        else:
            key = ''
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return key
 
 
class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_manual', 10)
        self.mode_pub = self.create_publisher(String, '/mode_switch', 10)
        self.mode = 'auto'
        print(INSTRUCTIONS)
 
    def run(self):
        while rclpy.ok():
            key = get_key()
            twist = Twist()
 
            if key == KEY_UP:
                twist.linear.x = LINEAR_SPEED
            elif key == KEY_DOWN:
                twist.linear.x = -LINEAR_SPEED
            elif key == KEY_LEFT:
                twist.angular.z = ANGULAR_SPEED
            elif key == KEY_RIGHT:
                twist.angular.z = -ANGULAR_SPEED
            elif key == KEY_TAB:
                self.mode = 'manual' if self.mode == 'auto' else 'auto'
                msg = String()
                msg.data = self.mode
                self.mode_pub.publish(msg)
                print(f'Mode: {self.mode.upper()}')
                continue
            elif key == 's':
                pass  # zero twist = stop
            elif key == 'q':
                break
            else:
                continue
 
            self.cmd_pub.publish(twist)
 
 
def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
 
 
if __name__ == '__main__':
    main()
