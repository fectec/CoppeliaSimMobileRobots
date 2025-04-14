#!/usr/bin/env python3

import rclpy
import numpy as np
import math

from rclpy.node import Node
from rclpy import qos
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry

# ========================
# Utility Functions
# ========================
def wrap_to_pi(theta):
    """Wrap angle to [-pi, pi]"""
    result = np.fmod((theta + np.pi), (2 * np.pi))
    if result < 0:
        result += 2 * np.pi
    return result - np.pi

def quaternion_to_euler(x, y, z, w):
    """Convert quaternion to Euler angles (roll, pitch, yaw)"""
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = max(min(t2, 1.0), -1.0)
    pitch = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)

    return roll, pitch, yaw

# ========================
# PID Position Controller Node
# ========================
class PIDPositionController(Node):
    """
    Computes velocity commands (V, w) using PID control to move a differential-drive
    robot toward a desired goal pose (x_g, y_g, theta_g) based on current odometry
    (x_r, y_r, theta_r).

    Subscribes to:
      - /setpoint (std_msgs/Float32MultiArray): Desired goal position
      - /odom (nav_msgs/Odometry): Current robot pose and orientation

    Publishes to:
      - /cmd_vel (geometry_msgs/Twist): Commanded linear and angular velocity

    Control logic:
      - Computes position error:
            e_x = x_g - x_r
            e_y = y_g - y_r
            e_d = sqrt(e_x² + e_y²)
            e_theta = atan2(e_y, e_x) - theta_r
      - Wraps e_theta to [-pi, pi]
      - Applies full PID controllers:
            V = Kp_v * e_d + Ki_v * ∫e_d + Kd_v * Δe_d
            w = Kp_w * e_theta + Ki_w * ∫e_theta + Kd_w * Δe_theta
      - Stops the robot when both e_d and e_theta are within predefined tolerances

    Parameters:
      - Kp_v, Ki_v, Kd_v: PID gains for linear velocity
      - Kp_w, Ki_w, Kd_w: PID gains for angular velocity
      - position_tolerance: distance threshold to stop
      - angle_tolerance: angle threshold to stop
      - update_rate: control loop frequency (Hz)
    """
    
    def __init__(self):
        super().__init__('pid_position_controller')

        # Declare PID parameters
        self.declare_parameter('Kp_v', 1.0)
        self.declare_parameter('Ki_v', 0.0)
        self.declare_parameter('Kd_v', 0.0)

        self.declare_parameter('Kp_w', 0.5)
        self.declare_parameter('Ki_w', 0.0)
        self.declare_parameter('Kd_w', 0.0)

        # Declare stop tolerances
        self.declare_parameter('position_tolerance', 0.05)
        self.declare_parameter('angle_tolerance', 0.05)

        # Declare control loop update rate
        self.declare_parameter('update_rate', 10.0) 

        # Load parameters
        self.Kp_v = self.get_parameter('Kp_v').get_parameter_value().double_value
        self.Ki_v = self.get_parameter('Ki_v').get_parameter_value().double_value
        self.Kd_v = self.get_parameter('Kd_v').get_parameter_value().double_value

        self.Kp_w = self.get_parameter('Kp_w').get_parameter_value().double_value
        self.Ki_w = self.get_parameter('Ki_w').get_parameter_value().double_value
        self.Kd_w = self.get_parameter('Kd_w').get_parameter_value().double_value

        self.position_tolerance = self.get_parameter('position_tolerance').get_parameter_value().double_value
        self.angle_tolerance = self.get_parameter('angle_tolerance').get_parameter_value().double_value
        
        self.update_rate = self.get_parameter('update_rate').get_parameter_value().double_value

        # Robot state
        self.current_pose = Pose2D()
        self.setpoint = Pose2D()

        # PID internals
        self.last_time = self.get_clock().now()
        self.last_e_d = 0.0
        self.last_e_theta = 0.0
        self.integral_e_d = 0.0
        self.integral_e_theta = 0.0
        self.stopped = False

        # Publisher
        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            qos.QoSProfile(depth=10, reliability=qos.ReliabilityPolicy.RELIABLE)
        )

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            qos.qos_profile_sensor_data
        )
        self.goal_sub = self.create_subscription(
            Float32MultiArray,
            '/setpoint',
            self.setpoint_callback,
            qos.qos_profile_sensor_data
        )

        # Timer for control loop
        self.timer = self.create_timer(1.0 / self.update_rate, self.control_loop)

        self.get_logger().info("PID Position Controller Node Started.")

    def setpoint_callback(self, msg):
        self.setpoint.x = msg.data[0]
        self.setpoint.y = msg.data[1]
        self.setpoint.theta = msg.data[2]

    def odom_callback(self, msg):
        self.current_pose.x = msg.pose.pose.position.x
        self.current_pose.y = msg.pose.pose.position.y
        orientation = msg.pose.pose.orientation
        _, _, yaw = quaternion_to_euler(orientation.x, orientation.y, orientation.z, orientation.w)
        self.current_pose.theta = yaw

    def control_loop(self):
        # Compute errors
        e_x = self.setpoint.x - self.current_pose.x
        e_y = self.setpoint.y - self.current_pose.y
        e_d = math.sqrt(e_x**2 + e_y**2)
        e_theta = wrap_to_pi(math.atan2(e_y, e_x) - self.current_pose.theta)

        # Time delta
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        if dt == 0.0:
            return

        # Auto-stop logic
        if e_d < self.position_tolerance and abs(e_theta) < self.angle_tolerance:
            stop_cmd = Twist()
            self.cmd_pub.publish(stop_cmd)
            if not self.stopped:
                self.get_logger().info("Target reached. Stopping the robot.")
                self.stopped = True
            return
        else:
            self.stopped = False  # Reset flag if robot moves again

        # Linear PID
        self.integral_e_d += e_d * dt
        de_d = (e_d - self.last_e_d) / dt
        V = self.Kp_v * e_d + self.Ki_v * self.integral_e_d + self.Kd_v * de_d

        # Angular PID
        self.integral_e_theta += e_theta * dt
        de_theta = (e_theta - self.last_e_theta) / dt
        w = self.Kp_w * e_theta + self.Ki_w * self.integral_e_theta + self.Kd_w * de_theta

        # Save for next loop
        self.last_e_d = e_d
        self.last_e_theta = e_theta
        self.last_time = now

        # Publish Twist command
        cmd = Twist()
        cmd.linear.x = V
        cmd.angular.z = w
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = PIDPositionController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received. Shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()