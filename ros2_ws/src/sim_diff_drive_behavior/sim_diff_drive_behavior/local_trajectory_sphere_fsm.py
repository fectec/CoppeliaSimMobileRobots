#!/usr/bin/env python3

import sys

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist

from custom_interfaces.msg import SphereDetection

# FSM state constants
NO_SPHERE_IDLE       = 0
GREEN_SPHERE_FORWARD = 1
ORANGE_SPHERE_RIGHT  = 2
BLUE_SPHERE_LEFT     = 3
YELLOW_SPHERE_PAUSE  = 4

class LocalTrajectorySphereFSM(Node):

    def __init__(self):
        super().__init__('local_trajectory_sphere_fsm')

        # State register
        self.state = NO_SPHERE_IDLE

        # Last detection
        self.detection = SphereDetection()

        # Publishers
        self.waypoint_pub = self.create_publisher(
            Float32MultiArray, 'sim_diff_drive/point_PID/waypoint', 10)
        self.cmd_pub      = self.create_publisher(
            Twist,            'cmd_vel',                        10)

        # Subscriber acts as clock edge
        self.create_subscription(
            SphereDetection,
            'sphere_detection',
            self.sphere_callback,
            10)

        self.get_logger().info('LocalTrajectorySphereFSM Start.')

    def sphere_callback(self, msg):
        self.detection = msg
        color = self.detection.color

        if self.state == NO_SPHERE_IDLE:
            # IDLE → stop
            out_cmd = Twist()
            self.cmd_pub.publish(out_cmd)

        elif self.state == GREEN_SPHERE_FORWARD:
            # GREEN_SPHERE_FORWARD → send centroid as waypoint
            wp = Float32MultiArray()
            wp.data = [
                self.detection.centroid_x,
                self.detection.centroid_y
            ]
            self.waypoint_pub.publish(wp)

        elif self.state == ORANGE_SPHERE_RIGHT:
            # ORANGE_SPHERE_RIGHT → rotate right 
            pass

        elif self.state == BLUE_SPHERE_LEFT:
            # BLUE_SPHERE_LEFT → rotate left
            pass

        elif self.state == YELLOW_SPHERE_PAUSE:
            # YELLOW_SPHERE_PAUSE → zero cmd, wait timer
            pass

        ns = self.state

        if self.state == NO_SPHERE_IDLE:
            if   color == SphereDetection.COLOR_GREEN:  ns = GREEN_SPHERE_FORWARD
            elif color == SphereDetection.COLOR_ORANGE: ns = ORANGE_SPHERE_RIGHT
            elif color == SphereDetection.COLOR_BLUE:   ns = BLUE_SPHERE_LEFT
            elif color == SphereDetection.COLOR_YELLOW: ns = YELLOW_SPHERE_PAUSE

        elif self.state == GREEN_SPHERE_FORWARD:
            if   color == SphereDetection.COLOR_NONE:   ns = NO_SPHERE_IDLE
            elif color == SphereDetection.COLOR_ORANGE: ns = ORANGE_SPHERE_RIGHT
            elif color == SphereDetection.COLOR_BLUE:   ns = BLUE_SPHERE_LEFT
            elif color == SphereDetection.COLOR_YELLOW: ns = YELLOW_SPHERE_PAUSE

        elif self.state == ORANGE_SPHERE_RIGHT:
            if   color == SphereDetection.COLOR_GREEN:  ns = GREEN_SPHERE_FORWARD
            elif color == SphereDetection.COLOR_NONE:   ns = NO_SPHERE_IDLE

        elif self.state == BLUE_SPHERE_LEFT:
            if   color == SphereDetection.COLOR_GREEN:  ns = GREEN_SPHERE_FORWARD
            elif color == SphereDetection.COLOR_NONE:   ns = NO_SPHERE_IDLE

        elif self.state == YELLOW_SPHERE_PAUSE:
            if   color == SphereDetection.COLOR_GREEN:  ns = GREEN_SPHERE_FORWARD
            elif color == SphereDetection.COLOR_NONE:   ns = NO_SPHERE_IDLE

        if ns != self.state:
            self.get_logger().info(f'{self._state_name(self.state)} → {self._state_name(ns)}')
        self.state = ns

    def _state_name(self, s):
        return {
            NO_SPHERE_IDLE      : 'NO_SPHERE_IDLE',
            GREEN_SPHERE_FORWARD: 'GREEN_SPHERE_FORWARD',
            ORANGE_SPHERE_RIGHT : 'ORANGE_SPHERE_RIGHT',
            BLUE_SPHERE_LEFT    : 'BLUE_SPHERE_LEFT',
            YELLOW_SPHERE_PAUSE : 'YELLOW_SPHERE_PAUSE'
        }[s]

def main(args=None):
    rclpy.init(args=args)

    try:
        node = LocalTrajectorySphereFSM()
    except Exception as e:
        print(f"[FATAL] LocalTrajectorySphereFSM failed to initialize: {e}.", file=sys.stderr)
        rclpy.shutdown()
        return

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted with Ctrl+C.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()