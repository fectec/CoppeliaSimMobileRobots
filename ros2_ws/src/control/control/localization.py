#!/usr/bin/env python3

import rclpy
import transforms3d
import numpy as np
import math

from rclpy.node import Node
from rclpy import qos
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion

# ========================
# Utility Functions
# ========================
def wrap_to_pi(angle):
    """Wrap angle to [-pi, pi]"""
    result = np.fmod((angle + math.pi), 2.0 * math.pi)
    if result < 0:
        result += 2.0 * math.pi
    return result - math.pi

# ========================
# Dead Reckoning Node
# ========================
class DeadReckoning(Node):
    """
    Estimates the robot's pose (x, y, theta) by dead reckoning, using the angular
    velocities of the left and right wheels of a differential-drive robot. The 
    robot's position and orientation are updated over time using Euler integration
    of the kinematic model.

    Two equivalent formulations are used to describe the motion:

    1. Based on wheel angular velocities:
        x_{k+1}     = x_k + (r/2) * (w_r + w_l) * cos(theta_k) * dt
        y_{k+1}     = y_k + (r/2) * (w_r + w_l) * sin(theta_k) * dt
        theta_{k+1} = theta_k + (r/l) * (w_r - w_l) * dt

    2. Using computed linear and angular velocity:
        V     = (r/2) * (w_r + w_l)
        Omega = (r/l) * (w_r - w_l)

        x_{k+1}     = x_k + V * cos(theta_k) * dt
        y_{k+1}     = y_k + V * sin(theta_k) * dt
        theta_{k+1} = theta_k + Omega * dt

    Where:
      - r     = wheel_radius
      - l     = wheel_base (distance between wheels)
      - w_r   = angular velocity of the right wheel (rad/s)
      - w_l   = angular velocity of the left wheel (rad/s)
      - V     = linear velocity of the robot (m/s)
      - Omega = angular velocity of the robot (rad/s)
      - dt    = time step between updates (s)
      - theta_k = robot heading at step k

    The orientation angle theta is wrapped to the interval [-pi, pi] using a 
    utility function to avoid unbounded growth. The result is a continuously 
    updated pose estimate published as a nav_msgs/Odometry message on the /odom topic.
    """

    def __init__(self):
        super().__init__('dead_reckoning')

        # Declare parameters for wheel geometry and update rate
        self.declare_parameter('wheel_base', 0.33)
        self.declare_parameter('wheel_radius', 0.195 / 2)
        self.declare_parameter('update_rate', 20.0)

        # Load parameter values
        self.wheel_base = self.get_parameter('wheel_base').get_parameter_value().double_value
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.update_rate = self.get_parameter('update_rate').get_parameter_value().double_value
        
        # Robot pose
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Wheel angular velocities (rad/s)
        self.w_r = 0.0
        self.w_l = 0.0

        # Internal time reference
        self.first_update = True
        self.last_time = None

         # Subscribe to wheel speeds
        self.create_subscription(
            Float32,
            '/VelocityEncR',
            self.right_wheel_callback,
            qos.qos_profile_sensor_data
        )
        self.create_subscription(
            Float32,
            '/VelocityEncL',
            self.left_wheel_callback,
            qos.qos_profile_sensor_data
        )

        # Publish odometry
        self.odom_pub = self.create_publisher(
            Odometry,
            '/odom',
            qos.qos_profile_sensor_data
        )   

        # We'll skip updates if dt is too small
        self.min_dt = 1.0 / self.update_rate

        # Timer for periodic updates
        self.timer = self.create_timer(self.min_dt, self.update_odometry)

        self.get_logger().info("Differential-Drive Dead-Reckoning Node Started.") 

    def right_wheel_callback(self, msg):
        """
        Update right wheel angular velocity from the encoder (rad/s).
        """
        self.w_r = msg.data

    def left_wheel_callback(self, msg):
        """
        Update left wheel angular velocity from the encoder (rad/s).
        """
        self.w_l = msg.data

    def update_odometry(self):
        """
        Integrate the differential-drive equations to compute
        (x, y, theta). Then publish on /odom.
        """
        current_time = self.get_clock().now()

        if self.first_update:
            self.last_time = current_time
            self.first_update = False
            return

        # Time step
        dt = (current_time - self.last_time).nanoseconds * 1e-9
        if dt < self.min_dt:
            return
        
        # Convert wheel speeds (rad/s) to linear speeds (m/s)
        v_r = self.wheel_radius * self.w_r
        v_l = self.wheel_radius * self.w_l

        # Robot linear and angular velocity
        V = 0.5 * (v_r + v_l)
        Omega = (v_r - v_l) / self.wheel_base

        # Integrate pose using Euler's method
        self.x += V * math.cos(self.theta) * dt
        self.y += V * math.sin(self.theta) * dt
        self.theta = wrap_to_pi(self.theta + Omega * dt)

        # Prepare Odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        # Convert Euler -> Quaternion
        # euler2quat -> returns (w, x, y, z)
        quat = transforms3d.euler.euler2quat(0.0, 0.0, self.theta)
        odom_msg.pose.pose.orientation = Quaternion(
            x=quat[1],
            y=quat[2],
            z=quat[3],
            w=quat[0]
        )

        # Fill in twist
        odom_msg.twist.twist.linear.x = V
        odom_msg.twist.twist.angular.z = Omega

        # Publish
        self.odom_pub.publish(odom_msg)
        self.last_time = current_time

        # Log the updated pose
        self.get_logger().info(
            f"DeadReckoning -> x: {self.x:.3f}, y: {self.y:.3f}, theta: {self.theta:.3f} rad"
        )
        
def main(args=None):
    rclpy.init(args=args)
    node = DeadReckoning()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received. Shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()