#!/usr/bin/env python

import sys
import time
from math import radians, isclose

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from scripted_bot_driver.move_parent import MoveParent
from scripted_bot_interfaces.msg import RotateDebug
from geometry_msgs.msg import Twist

class Rotate(MoveParent):
    def __init__(self):
        super().__init__('rotate')

        # Publisher for debug data
        self.debug_msg = RotateDebug()
        self.debug_pub = self.create_publisher(RotateDebug, 'rotate_debug', 10)

    def parse_argv(self, argv):
        self.get_logger().info('parsing move_spec {}'.format(argv))
        self.run_once = True
        self.target_heading = None
        self.rate = None

        if len(argv) < 1 or len(argv) > 2:
            self.get_logger().fatal('Incorrect number of args given to Rotate: {}'.format(len(argv)))
            return -1

        try:
            self.target_heading = float(argv[0])  # first arg is the target heading
            self.target_heading = radians(self.target_heading)  # convert degrees to radians

            # optional second arg is the rate of rotation
            if len(argv) == 2:
                self.rate = float(argv[1])
                self.get_logger().info('Using supplied rate {}'.format(self.rate))
            else:
                self.rate = radians(30)  # default rate: 30 degrees/sec in radians
        except ValueError:
            self.get_logger().error('Invalid argument given: {}'.format(argv))
            return -1
        return len(argv)  # return number of args consumed

    def print(self):
        self.get_logger().info('Rotate to heading {} radians at rate: {} rad/s'.format(self.target_heading, self.rate))

    def run(self):
        if not self.is_odom_started():
            self.get_logger().error('ERROR: robot odometry has not started - exiting')
            return True

        if self.run_once:
            self.initial_yaw = self.get_current_yaw()
            self.target_yaw = self.initial_yaw + self.target_heading
            self.get_logger().info('Initial yaw: {}, target yaw: {}'.format(self.initial_yaw, self.target_yaw))
            self.debug_msg.initial_yaw = self.initial_yaw
            self.run_once = False

        current_yaw = self.get_current_yaw()
        yaw_error = self.target_yaw - current_yaw
        self.get_logger().info('Current yaw: {}, Yaw error: {}'.format(current_yaw, yaw_error))

        if isclose(yaw_error, 0, abs_tol=radians(1)):  # Consider as reached if within 1 degree
            self.get_logger().info('Reached target yaw: {}'.format(current_yaw))
            return True

        # calculate the rotation command
        angular_z = self.slew_rot(self.rate if yaw_error > 0 else -self.rate)
        self.send_move_cmd(0.0, angular_z)

        # publish debug data
        self.debug_msg.current_yaw = current_yaw
        self.debug_msg.yaw_error = yaw_error
        self.debug_msg.commanded_angular = angular_z
        self.debug_pub.publish(self.debug_msg)

        return False

    def get_current_yaw(self):
        # Extract the current yaw from the odometry
        quaternion = self.odom.pose.pose.orientation
        _, _, yaw = self.euler_from_quaternion(quaternion)
        return yaw

    def start_action_server(self):
        self.create_action_server('rotate')

    def get_feedback(self):
        text_feedback = 'Rotating at {}, yaw error: '.format(self.rate)
        progress_feedback = self.target_yaw - self.get_current_yaw()
        return text_feedback, progress_feedback

    def finish_cb(self):
        # clean up after a move and reset defaults for next move
        self.set_defaults()

        results = [self.target_yaw]
        return results

    @staticmethod
    def euler_from_quaternion(quat):
        # Convert quaternion to euler angles
        import tf_transformations
        return tf_transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])

    def usage():
        print('Usage: rotate.py <target_heading> [rate] - rotate to the specified heading in degrees, with optional rate in degrees/sec')
        sys.exit()

def main():
    rclpy.init()
    nh = Rotate()
    nh.start_action_server()
    nh.start_spin_thread()

    rclpy.spin(nh)

if __name__ == '__main__':
    main()
