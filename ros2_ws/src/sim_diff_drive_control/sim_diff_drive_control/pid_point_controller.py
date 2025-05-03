#!/usr/bin/env python3

import math
import sys

import rclpy
from rclpy.node import Node
from rclpy import qos
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult

from tf_transformations import euler_from_quaternion
from sim_diff_drive_utils.utils.math_helpers import wrap_to_pi

from std_msgs.msg import Float32, Float32MultiArray
from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry

from std_srvs.srv import SetBool

class PIDPointController(Node):
    """
    Computes velocity commands (V, Omega) using PID control to drive a differential‑drive 
    robot toward a desired goal pose (x_g, y_g, theta_g) based on current odometry (x_r, y_r, theta_r).

    Subscribes to:
      - /sim_diff_drive/odom (nav_msgs/Odometry): Current robot pose and orientation
      - /sim_diff_drive/point_PID/waypoint (std_msgs/Float32MultiArray): Desired goal, as [x_g, y_g, theta_g]

    Publishes to:
      - /cmd_vel (geometry_msgs/Twist): Commanded linear and angular velocity

    Control logic:
      - Compute global position errors:
            e_x = x_g - x_r
            e_y = y_g - y_r
      - Compute the signed distance error (projected along the robot's forward direction):
            signed_e_d = e_x*cos(theta_r) + e_y*sin(theta_r)
      - Compute the angular error (wrapped to [-pi, pi]):
            e_theta = wrap_to_pi(atan2(e_y, e_x) - theta_r)
      - The PID controllers compute:
            V     = Kp_V * (signed_e_d) + Ki_V * ∫(signed_e_d) dt + Kd_V * d(signed_e_d)/dt
            Omega = Kp_Omega * e_theta + Ki_Omega * ∫(e_theta) dt + Kd_Omega * d(e_theta)/dt

      The robot stops when both the absolute distance and angular errors are below the tolerances.

      This node also includes a service (/sim_diff_drive/point_PID/PID_stop) that, when called, forces the controller to publish a zero Twist.
      In addition, if a new waypoint is received (via /sim_diff_drive/point_PID/waypoint), the controller automatically resumes.
    """

    def __init__(self):
        super().__init__('pid_point_controller')
        
        # Declare parameters
        self.declare_parameter('update_rate',         10.0)         # Hz
        self.declare_parameter('Kp_V',                1.0)      
        self.declare_parameter('Ki_V',                0.0)
        self.declare_parameter('Kd_V',                0.0)
        self.declare_parameter('Kp_Omega',            1.5)
        self.declare_parameter('Ki_Omega',            0.0)
        self.declare_parameter('Kd_Omega',            0.0)
        self.declare_parameter('goal_tolerance',      0.01)         # m
        self.declare_parameter('heading_tolerance',   0.01)         # m
        self.declare_parameter('use_constant_linear_speed', True)   # Boolean to toggle constant linear speed
        self.declare_parameter('constant_linear_speed',     1.0)    # m/s

        # Retrieve parameters
        self.update_rate               = self.get_parameter('update_rate').value
        self.Kp_V                      = self.get_parameter('Kp_V').value
        self.Ki_V                      = self.get_parameter('Ki_V').value
        self.Kd_V                      = self.get_parameter('Kd_V').value
        self.Kp_Omega                  = self.get_parameter('Kp_Omega').value
        self.Ki_Omega                  = self.get_parameter('Ki_Omega').value
        self.Kd_Omega                  = self.get_parameter('Kd_Omega').value
        self.goal_tolerance            = self.get_parameter('goal_tolerance').value
        self.heading_tolerance         = self.get_parameter('heading_tolerance').value
        self.use_constant_linear_speed = self.get_parameter('use_constant_linear_speed').value
        self.constant_linear_speed     = self.get_parameter('constant_linear_speed').value

        # Timer for the control loop
        self.timer = self.create_timer(1.0 / self.update_rate, self.control_loop)

        # Register the on‐set‐parameters callback
        self.add_on_set_parameters_callback(self.parameter_callback)
        
        # Immediately validate the initial values
        init_params = [
            Parameter('update_rate',               Parameter.Type.DOUBLE, self.update_rate),
            Parameter('Kp_V',                      Parameter.Type.DOUBLE, self.Kp_V),
            Parameter('Ki_V',                      Parameter.Type.DOUBLE, self.Ki_V),
            Parameter('Kd_V',                      Parameter.Type.DOUBLE, self.Kd_V),
            Parameter('Kp_Omega',                  Parameter.Type.DOUBLE, self.Kp_Omega),
            Parameter('Ki_Omega',                  Parameter.Type.DOUBLE, self.Ki_Omega),
            Parameter('Kd_Omega',                  Parameter.Type.DOUBLE, self.Kd_Omega),
            Parameter('goal_tolerance',            Parameter.Type.DOUBLE, self.goal_tolerance),
            Parameter('heading_tolerance',         Parameter.Type.DOUBLE, self.heading_tolerance),
            Parameter('use_constant_linear_speed', Parameter.Type.BOOL,   self.use_constant_linear_speed),
            Parameter('constant_linear_speed',     Parameter.Type.DOUBLE, self.constant_linear_speed)
        ]

        result: SetParametersResult = self.parameter_callback(init_params)
        if not result.successful:
            raise RuntimeError(f"Parameter validation failed: {result.reason}")

        # Robot state
        self.current_pose = Pose2D()
        self.waypoint = Pose2D()

        # Time tracking (s)
        self.now_time = None
        self.last_time = None

        # PID internals 
        self.integral_e_d = 0.0
        self.integral_e_theta = 0.0
        self.last_signed_e_d = 0.0
        self.last_e_theta = 0.0
        self.stopped_pid = False

        # Publishers
        self.cmd_pub = self.create_publisher(
            Twist,
            'cmd_vel',
            qos.QoSProfile(depth=10, reliability=qos.ReliabilityPolicy.RELIABLE)
        )

        self.signed_e_d_pub   = self.create_publisher(Float32, 'sim_diff_drive/point_PID/signed_e_d', 10)
        self.abs_e_d_pub = self.create_publisher(Float32, 'sim_diff_drive/point_PID/abs_e_d', 10)
        self.e_theta_pub    = self.create_publisher(Float32, 'sim_diff_drive/point_PID/e_theta', 10)

        # Subscribers for odometry and goal waypoint
        self.create_subscription(
            Odometry,
            'sim_diff_drive/odom',
            self.odom_callback,
            qos.qos_profile_sensor_data
        )
        self.create_subscription(
            Float32MultiArray,
            'sim_diff_drive/point_PID/waypoint',
            self.waypoint_callback,
            qos.qos_profile_sensor_data
        )

        # Simulation time subscriber (Float32 message from CoppeliaSim)
        self.create_subscription(
            Float32,
            'simulationTime',
            self.sim_time_callback,
            qos.qos_profile_sensor_data
        )

        # Create a service server to allow external stopping/resuming of PID output
        self.create_service(SetBool, 'sim_diff_drive/point_PID/PID_stop', self.pid_stop_callback)
        self.pid_stop = False

        self.get_logger().info("PIDPointController Start.")
    
    def sim_time_callback(self, msg):
        # Update simulation time from the /simulationTime topic (s)
        self.now_time = msg.data

    def odom_callback(self, msg):
        # Update x, y
        self.current_pose.x = msg.pose.pose.position.x
        self.current_pose.y = msg.pose.pose.position.y

        # Convert the incoming orientation quaternion to Euler angles,
        # then store the yaw component
        q = msg.pose.pose.orientation
        roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.current_pose.theta = yaw
        
    def waypoint_callback(self, msg):
        # Update the target goal pose from the waypoint message
        self.waypoint.x = msg.data[0]
        self.waypoint.y = msg.data[1]
        # Automatically resume PID control if a new waypoint is received while stopped
        if self.pid_stop:
            self.get_logger().info("New waypoint received, resuming PID control.")
            self.pid_stop = False

    def pid_stop_callback(self, request, response):
        self.pid_stop = request.data
        response.success = True
        if self.pid_stop:
            response.message = "PID controller stopped."
            self.get_logger().info("Received PID stop command.")
        else:
            response.message = "PID controller resumed."
            self.get_logger().info("Received PID resume command.")
        return response

    def control_loop(self):
        # Ensure we have received simulation time
        if self.now_time is None:
            return

       # Initialization on first run
        if self.last_time is None:
            self.last_time = self.now_time
            return
        
        # Calculate the elapsed time 
        # since the last update and return early 
        # if it’s less than the interval dictated by update_rate,
        # to enforce a consistent loop frequency
        dt = self.now_time - self.last_time
        if dt < 1.0 / self.update_rate:
            return
        self.last_time = self.now_time

        # If the PID stop flag is set, publish a zero Twist and return immediately
        if self.pid_stop:
            self.cmd_pub.publish(Twist())
            self.get_logger().debug("PID stop flag active; publishing zero command.")
            self.last_time = self.now_time
            return

        # Compute position error components
        e_x = self.waypoint.x - self.current_pose.x
        e_y = self.waypoint.y - self.current_pose.y

        # Signed distance error along the robot's heading
        signed_e_d = e_x * math.cos(self.current_pose.theta) + e_y * math.sin(self.current_pose.theta)
    
        # Absolute distance error
        abs_e_d = math.hypot(e_x, e_y)

        # Compute angular error (wrapped to [-pi, pi])
        e_theta = wrap_to_pi(math.atan2(e_y, e_x) - self.current_pose.theta)

        # Publish the intermediate error signals
        self.signed_e_d_pub.publish(Float32(data=signed_e_d))
        self.abs_e_d_pub.publish(Float32(data=abs_e_d))
        self.e_theta_pub.publish(Float32(data=e_theta))
        
        # Auto-stop if both errors are below thresholds
        if abs_e_d < self.goal_tolerance and abs(e_theta) < self.heading_tolerance:
            self.cmd_pub.publish(Twist())
            if not self.stopped_pid:
                self.stopped_pid = True
                self.get_logger().info("Target reached. Stopping the robot.")
            self.last_time = self.now_time
            return 
        else:
            self.stopped_pid = False

        # Calculate linear velocity (V) based on mode
        if self.use_constant_linear_speed:
            # Use constant linear speed
            V = self.constant_linear_speed
        else:
            # PID control for linear velocity
            self.integral_e_d += signed_e_d * dt
            derivative_e_d = (signed_e_d - self.last_signed_e_d) / dt
            V = self.Kp_V * signed_e_d + self.Ki_V * self.integral_e_d + self.Kd_V * derivative_e_d

        # PID control for angular velocity
        self.integral_e_theta += e_theta * dt
        derivative_e_theta = (e_theta - self.last_e_theta) / dt
        Omega = self.Kp_Omega * e_theta + self.Ki_Omega * self.integral_e_theta + self.Kd_Omega * derivative_e_theta

        # Save errors and time for the next iteration
        self.last_signed_e_d = signed_e_d
        self.last_e_theta = e_theta
        self.last_time = self.now_time

        # Publish the computed command
        cmd = Twist()
        cmd.linear.x = V
        cmd.angular.z = Omega
        self.cmd_pub.publish(cmd)

        # Debug info
        self.get_logger().info(
            f"Control: dist_err={abs_e_d:.3f}, ang_err={e_theta:.3f}, V={V:.3f}, Omega={Omega:.3f}."
        )

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'update_rate':
                if not isinstance(param.value, (int, float)) or param.value <= 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason="update_rate must be > 0."
                    )
                self.update_rate = float(param.value)
                self.timer.cancel()
                self.timer = self.create_timer(1.0 / self.update_rate, self.control_loop)
                self.get_logger().info(f"Update rate changed: {self.update_rate} Hz.")

            elif param.name in (
                'Kp_V', 'Ki_V', 'Kd_V',
                'Kp_Omega', 'Ki_Omega', 'Kd_Omega'
            ):
                if not isinstance(param.value, (int, float)) or param.value < 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason=f"{param.name} must be a non-negative number."
                    )
                setattr(self, param.name, float(param.value))
                self.get_logger().info(f"{param.name} updated: {param.value}.")

            elif param.name == 'goal_tolerance':
                if not isinstance(param.value, (int, float)) or param.value < 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason="goal_tolerance must be a non-negative number."
                    )
                self.goal_tolerance = float(param.value)
                self.get_logger().info(f"Goal tolerance updated: {self.goal_tolerance}.")

            elif param.name == 'heading_tolerance':
                if not isinstance(param.value, (int, float)) or param.value < 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason="heading_tolerance must be a non-negative number."
                    )
                self.heading_tolerance = float(param.value)
                self.get_logger().info(f"Heading tolerance updated: {self.heading_tolerance}.")
            
            elif param.name == 'use_constant_linear_speed':
                if not isinstance(param.value, bool):
                    return SetParametersResult(
                        successful=False,
                        reason="use_constant_linear_speed must be a boolean value."
                    )
                self.use_constant_linear_speed = param.value

            elif param.name == 'constant_linear_speed':
                if not isinstance(param.value, (int, float)) or param.value < 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason="constant_linear_speed must be a non-negative number."
                    )
                self.constant_linear_speed = float(param.value)
                self.get_logger().info(f"Constant linear speed updated: {self.constant_linear_speed}.")
        
        return SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)

    try:
        node = PIDPointController()
    except Exception as e:
        print(f"[FATAL] PIDPointController failed to initialize: {e}.", file=sys.stderr)
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