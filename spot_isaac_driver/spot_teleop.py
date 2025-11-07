#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Original credits: https://github.com/ros-teleop/teleop_twist_keyboard
# ROS 2 port by Disaster Relief Robotics / Space Station OS 2025

import os
import sys
import select
import termios
import tty
import numpy as np
if not hasattr(np, 'float'):
    np.float = float
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from spot_isaac_interface.msg import Pose as PoseLite
from sensor_msgs.msg import Joy
import tf_transformations


class Teleop(Node):
    def __init__(self):
        super().__init__('champ_teleop')

        # Publishers
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 1)
        self.pose_lite_pub = self.create_publisher(PoseLite, '/body_pose/raw', 1)
        self.pose_pub = self.create_publisher(Pose, '/body_pose', 1)

        # Subscriber (joystick)
        self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        # Parameters
        self.speed = self.declare_parameter('speed', 0.2).value
        self.turn = self.declare_parameter('turn', 1.0).value
        self.swing_height = self.declare_parameter('gait.swing_height', 0.0).value
        self.nominal_height = self.declare_parameter('gait.nominal_height', 0.0).value

        # Terminal settings
        self.settings = termios.tcgetattr(sys.stdin)

        # Instructions
        self.msg = """
Reading from the keyboard and Publishing to Twist!
---------------------------
Moving around:
u    i    o
j    k    l
m    ,    .
For Holonomic mode (strafing), hold down the shift key:
---------------------------
U    I    O
J    K    L
M    <    >
t : up (+z)
b : down (-z)
anything else : stop
q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
CTRL-C to quit
"""

        # Bindings
        self.velocityBindings = {
            'i': (1, 0, 0, 0),
            'o': (1, 0, 0, -1),
            'j': (0, 0, 0, 1),
            'l': (0, 0, 0, -1),
            'u': (1, 0, 0, 1),
            ',': (-1, 0, 0, 0),
            '.': (-1, 0, 0, 1),
            'm': (-1, 0, 0, -1),
            'O': (1, -1, 0, 0),
            'I': (1, 0, 0, 0),
            'J': (0, 1, 0, 0),
            'L': (0, -1, 0, 0),
            'U': (1, 1, 0, 0),
            '<': (-1, 0, 0, 0),
            '>': (-1, -1, 0, 0),
            'M': (-1, 1, 0, 0),
            'v': (0, 0, 1, 0),
            'n': (0, 0, -1, 0),
        }

        self.speedBindings = {
            'q': (1.1, 1.1),
            'z': (0.9, 0.9),
            'w': (1.1, 1.0),
            'x': (0.9, 1.0),
            'e': (1.0, 1.1),
            'c': (1.0, 0.9),
        }

        # Start blocking key loop (like ROS 1)
        self.poll_keys()

    # -------------------- Joystick callback --------------------
    def joy_callback(self, data: Joy):
        twist = Twist()
        twist.linear.x = data.axes[1] * self.speed
        twist.linear.y = data.buttons[4] * data.axes[0] * self.speed
        twist.angular.z = (not data.buttons[4]) * data.axes[0] * self.turn
        self.vel_pub.publish(twist)

        pose_lite = PoseLite()
        pose_lite.roll = (not data.buttons[5]) * -data.axes[3] * 0.349066
        pose_lite.pitch = data.axes[4] * 0.174533
        pose_lite.yaw = data.buttons[5] * data.axes[3] * 0.436332
        if data.axes[5] < 0:
            pose_lite.z = data.axes[5] * 0.5
        self.pose_lite_pub.publish(pose_lite)

        pose = Pose()
        pose.position.z = pose_lite.z
        q = tf_transformations.quaternion_from_euler(
            pose_lite.roll, pose_lite.pitch, pose_lite.yaw
        )
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = q
        self.pose_pub.publish(pose)

    # -------------------- Keyboard polling --------------------
    def poll_keys(self):
        x = y = z = th = 0
        status = 0
        cmd_attempts = 0

        try:
            while rclpy.ok():
                os.system('clear')  # keep the message always visible
                print(self.msg)
                print(self.vels(self.speed, self.turn))
                sys.stdout.flush()

                key = self.getKey()
                if key in self.velocityBindings.keys():
                    x, y, z, th = self.velocityBindings[key]
                    if cmd_attempts > 1:
                        twist = Twist()
                        twist.linear.x = x * self.speed
                        twist.linear.y = y * self.speed
                        twist.linear.z = z * self.speed
                        twist.angular.z = th * self.turn
                        self.vel_pub.publish(twist)
                    cmd_attempts += 1

                elif key in self.speedBindings.keys():
                    self.speed *= self.speedBindings[key][0]
                    self.turn *= self.speedBindings[key][1]
                    print(self.vels(self.speed, self.turn))
                    if status == 14:
                        print(self.msg)
                    status = (status + 1) % 15

                else:
                    cmd_attempts = 0
                    if key == '\x03':  # Ctrl-C
                        break

        except Exception as e:
            print(e)

        finally:
            self.stop_robot()
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

    def stop_robot(self):
        twist = Twist()
        self.vel_pub.publish(twist)

    # -------------------- Utilities --------------------
    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        key = sys.stdin.read(1) if rlist else ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def vels(self, speed, turn):
        return f"currently:\tspeed {speed:.2f}\tturn {turn:.2f}"


def main(args=None):
    rclpy.init(args=args)
    node = Teleop()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
