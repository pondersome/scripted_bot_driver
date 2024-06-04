#!/usr/bin/env python

import sys
import time
from math import radians, degrees, isclose

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from scripted_bot_interfaces.action import Rotate
from scripted_bot_interfaces.msg import RotateDebug


class Rotate(Node):
    def __init__(self):
        super().__init__('rotate')

        self._action_server = ActionServer(
            self,
            Rotate,
            'rotate',
            execute_callback=self.execute_callback
        )

        self.debug_msg = RotateDebug()
        self.debug_pub = self.create_publisher(RotateDebug, 'rotate_debug', 10)

        self.target_heading = 0.0
        self.rotation_speed = 0.0
        self.current_heading = 0.0
        self.initial_heading = 0.0
        self.delta_rotation = 0.0
        self.run_once = True

    def parse_argv(self, argv):
        self.get_logger().info('parsing move_spec {}'.format(argv))
        self.run_once = True

        if len(argv) < 1 or len(argv) > 2:
            self.get_logger().fatal('Incorrect number of args given to Rotate: {}'.format(len(argv)))
            return -1

        try:
            self.target_heading = float(argv[0])  # Target heading in degrees
            self.rotation_speed = float(argv[1]) if len(argv) == 2 else DEFAULT_ROTATION_SPEED
            self.get_logger().info('Rotate to {} degrees at speed: {}'.format(self.target_heading, self.rotation_speed))
        except ValueError:
            self.get_logger().error('Invalid argument given: {}'.format(argv))
            return -1
        return len(argv)  # return number of args consumed

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        self.parse_argv(goal_handle.request.move_spec)

        self.initial_heading = self.get_current_heading()
        self.current_heading = self.initial_heading
        self.delta_rotation = 0.0
        self.run_once = True

        while not self.is_goal_reached():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Rotate.Result()

            self.rotate_robot(self.rotation_speed)
            self.current_heading = self.get_current_heading()
            self.delta_rotation = abs(self.current_heading - self.initial_heading)

            self.publish_debug()

            feedback_msg = Rotate.Feedback()
            feedback_msg.feedback_text = 'Rotating'
            feedback_msg.progress = self.delta_rotation
            goal_handle.publish_feedback(feedback_msg)

            time.sleep(0.1)

        goal_handle.succeed()

        result = Rotate.Result()
        result.move_results = [self.delta_rotation]
        self.get_logger().info('Goal succeeded')

        return result

    def rotate_robot(self, rotation_speed):
        # Send command to robot to rotate at the given speed
        self.get_logger().info('Rotating at speed: {}'.format(rotation_speed))
        # Implementation needed here to rotate the robot

    def get_current_heading(self):
        # Retrieve the current heading of the robot
        self.get_logger().info('Getting current heading')
        # Implementation needed here to get the actual heading
        return 0.0

    def is_goal_reached(self):
        return isclose(self.delta_rotation, self.target_heading, abs_tol=0.5)

    def publish_debug(self):
        self.debug_msg.target_heading = self.target_heading
        self.debug_msg.rotation_speed = self.rotation_speed
        self.debug_msg.initial_heading = self.initial_heading
        self.debug_msg.commanded_angular = self.rotation_speed
        self.debug_msg.delta_rotation = self.delta_rotation
        self.debug_pub.publish(self.debug_msg)


def main(args=None):
    rclpy.init(args=args)
    node = Rotate()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
