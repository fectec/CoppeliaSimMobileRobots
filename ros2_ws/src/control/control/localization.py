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
def wrap_to_pi(theta):
    # Wrap angle to [-pi, pi]
    result = np.fmod(theta + math.pi, 2.0 * math.pi)
    if result < 0:
        result += 2.0 * math.pi
    return result - math.pi

def quaternion_to_euler(x, y, z, w):
    # Convert quaternion (x, y, z, w) to Euler angles (roll, pitch, yaw)
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
# Dead Reckoning Node Using Simulation Time
# ========================
class DeadReckoning(Node):
    """
    Estimates the robot's pose (x, y, theta) by dead reckoning, using the angular
    velocities of the left and right wheels of a differential-drive robot. The 
    robot's position and orientation are updated over time using Euler integration
    of the kinematic model.

    The orientation is maintained in the range [-pi, pi].

    **Simulation Time:**
    This node subscribes to the /simulationTime topic to obtain the simulation time 
    (in seconds) published from CoppeliaSim. That simulation time is used for the
    integration time-step (dt).
    """

    def __init__(self):
        super().__init__('dead_reckoning')

        # Declare parameters for wheel geometry (m) and update rate (Hz)
        self.declare_parameter('wheel_base', 0.119)
        self.declare_parameter('wheel_radius', 0.027)
        self.declare_parameter('update_rate', 40.0)

        # Load parameter values
        self.wheel_base = self.get_parameter('wheel_base').get_parameter_value().double_value
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.update_rate = self.get_parameter('update_rate').get_parameter_value().double_value 
        
        # Robot pose (x, y) and heading (theta in [-pi, pi])
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Wheel angular velocities (rad/s)
        self.omega_r = 0.0
        self.omega_l = 0.0

        # For simulation time (s) tracking
        self.sim_time = None        # current simulation time
        self.last_sim_time = None   # last simulation time received

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
        # Subscribe to simulation time (Float32 message from CoppeliaSim)
        self.create_subscription(
            Float32,
            '/simulationTime',
            self.sim_time_callback,
            qos.qos_profile_sensor_data
        )

        # Publisher for odometry
        self.odom_pub = self.create_publisher(
            Odometry,
            '/odom',
            qos.qos_profile_sensor_data
        )   

        # Timer for periodic updates; use a minimum dt from update_rate
        self.min_dt = 1.0 / self.update_rate
        self.timer = self.create_timer(self.min_dt, self.update_odometry)

        self.get_logger().info("Differential-Drive Dead-Reckoning Node Started.")

    def right_wheel_callback(self, msg):
        """Update right wheel angular velocity (rad/s) from the encoder."""
        self.omega_r = msg.data

    def left_wheel_callback(self, msg):
        """Update left wheel angular velocity (rad/s) from the encoder."""
        self.omega_l = msg.data

    def sim_time_callback(self, msg):
        """
        Callback for /simulationTime topic.
        The simulation time (in seconds) is provided in msg.data.
        """
        self.sim_time = msg.data

    def update_odometry(self):
        """
        Integrate the differential-drive equations using simulation time dt
        to update (x, y, theta) and publish the odometry message.
        """
        # Ensure we have received simulation time
        if self.sim_time is None:
            return

        # Initialize last_sim_time on first valid update
        if self.last_sim_time is None:
            self.last_sim_time = self.sim_time
            return

        # Compute dt from simulation time differences
        dt = self.sim_time - self.last_sim_time
        if dt < self.min_dt:
            return
        self.last_sim_time = self.sim_time

        # Convert wheel angular velocities (rad/s) to linear speeds (m/s)
        v_r = self.wheel_radius * self.omega_r
        v_l = self.wheel_radius * self.omega_l

        # Compute linear (m/s) and angular (rad/s) velocities of the robot
        V = 0.5 * (v_r + v_l)
        Omega = (v_r - v_l) / self.wheel_base

        # Integrate pose using Euler's method
        self.x += V * math.cos(self.theta) * dt
        self.y += V * math.sin(self.theta) * dt
        # Use wrap_to_pi to maintain theta in [-pi, pi]
        self.theta = wrap_to_pi(self.theta + Omega * dt)

        # Prepare the Odometry message
        odom_msg = Odometry()
        # Set the header stamp using simulation time
        odom_msg.header.stamp = rclpy.time.Time(seconds=self.sim_time).to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        # Convert Euler (with theta in [-pi, pi]) to Quaternion
        quat = transforms3d.euler.euler2quat(0.0, 0.0, self.theta)
        odom_msg.pose.pose.orientation = Quaternion(
            x=quat[1],
            y=quat[2],
            z=quat[3],
            w=quat[0]
        )

        # Fill in twist (velocities)
        odom_msg.twist.twist.linear.x = V
        odom_msg.twist.twist.angular.z = Omega

        # Publish odometry
        self.odom_pub.publish(odom_msg)

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