#!/usr/bin/python3
"""
Simple Binosense Control Example in Python
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
import math
import time


class BinosenseSimpleController(Node):
    def __init__(self):
        super().__init__('binosense_simple_controller')

        self.position_pub = self.create_publisher(
            Float32MultiArray,
            'motor_command',
            10
        )

        self.twist_pub = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )

        self.motor_sub = self.create_subscription(
            Float32MultiArray,
            'motor_state',
            self.motor_state_callback,
            10
        )

        self.current_positions = [0.0] * 6
        self.get_logger().info('Binosense Simple Controller Started')

    def motor_state_callback(self, msg):
        if len(msg.data) >= 6:
            self.current_positions = list(msg.data)

    def go_home(self):
        """Move all motors to home position (0 degrees)"""
        msg = Float32MultiArray()
        msg.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.position_pub.publish(msg)
        self.get_logger().info('Going home...')

    def set_position(self, right_pitch, right_roll, right_yaw,
                     left_pitch, left_roll, left_yaw):
        """Set absolute position for all motors"""
        msg = Float32MultiArray()
        msg.data = [right_pitch, right_roll, right_yaw,
                    left_pitch, left_roll, left_yaw]
        self.position_pub.publish(msg)
        self.get_logger().info(f'Setting position: {msg.data}')

    def look_left(self, angle=15.0):
        """Look both eyes left"""
        msg = Float32MultiArray()
        current = self.current_positions.copy()
        current[2] = angle
        current[5] = angle
        msg.data = current
        self.position_pub.publish(msg)

    def look_right(self, angle=15.0):
        """Look both eyes right"""
        msg = Float32MultiArray()
        current = self.current_positions.copy()
        current[2] = -angle
        current[5] = -angle
        msg.data = current
        self.position_pub.publish(msg)

    def look_up(self, angle=10.0):
        """Look both eyes up"""
        msg = Float32MultiArray()
        current = self.current_positions.copy()
        current[0] = angle
        current[3] = angle
        msg.data = current
        self.position_pub.publish(msg)

    def look_down(self, angle=10.0):
        """Look both eyes down"""
        msg = Float32MultiArray()
        current = self.current_positions.copy()
        current[0] = -angle
        current[3] = -angle
        msg.data = current
        self.position_pub.publish(msg)

    def send_velocity(self, yaw_rate=0.0, pitch_rate=0.0, roll_rate=0.0):
        """Send velocity command using Twist"""
        msg = Twist()
        msg.angular.z = yaw_rate
        msg.angular.y = pitch_rate
        msg.angular.x = roll_rate
        self.twist_pub.publish(msg)

    def sweep_demo(self, duration=10.0):
        """Perform a sweep motion demo"""
        self.get_logger().info('Starting sweep demo...')
        start_time = time.time()

        while time.time() - start_time < duration:
            t = time.time() - start_time
            yaw = 15.0 * math.sin(t * 2.0)
            pitch = 10.0 * math.sin(t)

            self.set_position(pitch, 0.0, yaw, pitch, 0.0, yaw)
            time.sleep(0.05)

        self.go_home()
        self.get_logger().info('Sweep demo complete')


def main(args=None):
    rclpy.init(args=args)
    controller = BinosenseSimpleController()

    try:
        print("\n=== Binosense Simple Controller ===")
        print("Available commands:")
        print("  home     - Go to home position")
        print("  left     - Look left")
        print("  right    - Look right")
        print("  up       - Look up")
        print("  down     - Look down")
        print("  sweep    - Run sweep demo")
        print("  pos <r_p> <r_r> <r_y> <l_p> <l_r> <l_y> - Set position")
        print("  quit     - Exit")
        print()

        while rclpy.ok():
            try:
                cmd = input("> ").strip().lower()

                if cmd == 'home' or cmd == 'h':
                    controller.go_home()
                elif cmd == 'left' or cmd == 'l':
                    controller.look_left()
                elif cmd == 'right' or cmd == 'r':
                    controller.look_right()
                elif cmd == 'up' or cmd == 'u':
                    controller.look_up()
                elif cmd == 'down' or cmd == 'd':
                    controller.look_down()
                elif cmd == 'sweep' or cmd == 's':
                    controller.sweep_demo(duration=5.0)
                elif cmd.startswith('pos '):
                    parts = cmd.split()
                    if len(parts) == 7:
                        try:
                            angles = [float(x) for x in parts[1:7]]
                            controller.set_position(*angles)
                        except ValueError:
                            print("Invalid angle values")
                    else:
                        print("Usage: pos <r_p> <r_r> <r_y> <l_p> <l_r> <l_y>")
                elif cmd == 'quit' or cmd == 'q' or cmd == 'exit':
                    controller.go_home()
                    break
                elif cmd:
                    print("Unknown command")

                rclpy.spin_once(controller, timeout_sec=0.1)

            except (EOFError, KeyboardInterrupt):
                controller.go_home()
                break

    except Exception as e:
        print(f"Error: {e}")
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
