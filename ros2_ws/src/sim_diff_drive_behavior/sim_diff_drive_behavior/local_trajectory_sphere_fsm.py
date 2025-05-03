#!/usr/bin/env python3
import sys

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32, Float32MultiArray
from geometry_msgs.msg import Twist

from custom_interfaces.msg import SphereDetection

from std_srvs.srv import SetBool

# FSM state constants
NO_SPHERE_IDLE = 0
GREEN_SPHERE_FORWARD = 1
ORANGE_SPHERE_RIGHT = 2
BLUE_SPHERE_LEFT = 3
YELLOW_SPHERE_PAUSE = 4

class LocalTrajectorySphereFSM(Node):
    def __init__(self):
        super().__init__('local_trajectory_sphere_fsm')
        
        # Declare parameters
        self.declare_parameter('update_rate', 20.0)  # Hz
        self.declare_parameter('angular_velocity_right', -1.0)  # rad/s for orange sphere
        self.declare_parameter('angular_velocity_left', 1.0)   # rad/s for blue sphere
        self.declare_parameter('pause_duration', 5.0)  # seconds for yellow sphere pause
        
        # Retrieve parameters
        self.update_rate = self.get_parameter('update_rate').value
        self.angular_velocity_right = self.get_parameter('angular_velocity_right').value
        self.angular_velocity_left = self.get_parameter('angular_velocity_left').value
        self.pause_duration = self.get_parameter('pause_duration').value
        
        # State register
        self.state = NO_SPHERE_IDLE
        self.previous_state = NO_SPHERE_IDLE
        
        # Last detection
        self.detection = SphereDetection()
        self.new_detection = False
        
        # Simulation time tracking
        self.now_time = None
        self.last_time = None
        
        # Flag for tracking PID controller status
        self.pid_stopped = True  # Assume PID is initially stopped
        
        # Publishers
        self.waypoint_pub = self.create_publisher(
            Float32MultiArray, 'sim_diff_drive/point_PID/waypoint', 10)
        self.cmd_pub = self.create_publisher(
            Twist, 'cmd_vel', 10)
            
        # Subscriber for sphere detection
        self.create_subscription(
            SphereDetection,
            'sphere_detection',
            self.sphere_callback,
            10)
            
        # Subscriber for simulation time
        self.create_subscription(
            Float32,
            'simulationTime',
            self.sim_time_callback,
            10)
        
        # Create a client for stopping/resuming the PID controller via a service
        self.pid_stop_client = self.create_client(SetBool, 'sim_diff_drive/point_PID/PID_stop')
        while not self.pid_stop_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("/sim_diff_drive/point_PID/PID_stop service not available, waiting...")
            
        # Timer for the control loop based on update_rate
        self.timer = self.create_timer(1.0 / self.update_rate, self.fsm_update_loop)
        
        # Yellow sphere pause timer
        self.pause_start_time = None
        
        self.get_logger().info('LocalTrajectorySphereFSM Start.')
        self.get_logger().info(f'Angular velocity right: {self.angular_velocity_right} rad/s')
        self.get_logger().info(f'Angular velocity left: {self.angular_velocity_left} rad/s')
        self.get_logger().info(f'Pause duration: {self.pause_duration} seconds')
    
    def sim_time_callback(self, msg):
        # Update simulation time from the /simulationTime topic (s)
        self.now_time = msg.data
    
    def sphere_callback(self, msg):
        # Just store the detection and set a flag
        self.detection = msg
        self.new_detection = True
    
    def call_pid_stop_service(self, stop: bool):
        # Prepare a service request for stopping (if stop==True) or resuming (if stop==False) the PID controller
        req = SetBool.Request()
        req.data = stop
        future = self.pid_stop_client.call_async(req)
        # Use asynchronous callback so that the timer thread is not blocked
        future.add_done_callback(lambda fut: self.pid_stop_response_callback(fut, stop))

    def pid_stop_response_callback(self, future, stop):
        try:
            result = future.result()
            self.pid_stopped = stop
            self.get_logger().info(f"PID stop service call successful: {'stopping' if stop else 'resuming'} PID.")
        except Exception as e:
            self.get_logger().error("PID stop service call failed: " + str(e))
    
    def fsm_update_loop(self):
        # Ensure we have received simulation time
        if self.now_time is None:
            return
            
        # Initialization on first run
        if self.last_time is None:
            self.last_time = self.now_time
            return
            
        # Calculate the elapsed time since the last update
        dt = self.now_time - self.last_time
        if dt < 1.0 / self.update_rate:
            return
        
        self.last_time = self.now_time
        
        # Process current state actions
        self.process_state_actions()
        
        # Check for state transitions only if we've received a new detection
        if self.new_detection:
            self.previous_state = self.state
            self.process_state_transitions()
            
            # Handle PID controller based on state changes
            self.handle_pid_controller()
            
            self.new_detection = False
    
    def handle_pid_controller(self):
        # If transitioning to or from GREEN_SPHERE_FORWARD, manage PID controller
        if self.state != self.previous_state:
            # If new state is GREEN_SPHERE_FORWARD and PID is stopped, resume it
            if self.state == GREEN_SPHERE_FORWARD and self.pid_stopped:
                self.call_pid_stop_service(False)
                self.get_logger().info("Entering GREEN_SPHERE_FORWARD state, resuming PID controller.")
            
            # If leaving GREEN_SPHERE_FORWARD and PID is not stopped, stop it
            elif self.previous_state == GREEN_SPHERE_FORWARD and not self.pid_stopped:
                self.call_pid_stop_service(True)
                self.get_logger().info("Leaving GREEN_SPHERE_FORWARD state, stopping PID controller.")
    
    def process_state_actions(self):
        if self.state == NO_SPHERE_IDLE:
            # IDLE → stop
            out_cmd = Twist()
            self.cmd_pub.publish(out_cmd)
            
        elif self.state == GREEN_SPHERE_FORWARD:
            # GREEN_SPHERE_FORWARD → send waypoint to PID controller
            # The waypoint is already calculated by SphereDetectionNode
            # and stored in the detection message
            wp = Float32MultiArray()
            wp.data = [
                self.detection.x,
                self.detection.y
            ]
            self.waypoint_pub.publish(wp)
            self.get_logger().debug(f'Moving to waypoint: x={self.detection.x:.2f}, y={self.detection.y:.2f}.')
            
        elif self.state == ORANGE_SPHERE_RIGHT:
            # ORANGE_SPHERE_RIGHT → direct control - rotate right
            out_cmd = Twist()
            out_cmd.linear.x = 0.0  # No forward motion
            out_cmd.angular.z = self.angular_velocity_right  # Turn right (negative angular velocity)
            self.cmd_pub.publish(out_cmd)
            
        elif self.state == BLUE_SPHERE_LEFT:
            # BLUE_SPHERE_LEFT → direct control - rotate left
            out_cmd = Twist()
            out_cmd.linear.x = 0.0  # No forward motion
            out_cmd.angular.z = self.angular_velocity_left  # Turn left (positive angular velocity)
            self.cmd_pub.publish(out_cmd)
            
        elif self.state == YELLOW_SPHERE_PAUSE:
            # YELLOW_SPHERE_PAUSE → zero cmd
            out_cmd = Twist()
            self.cmd_pub.publish(out_cmd)
            
            # Track pause timing if needed
            if self.pause_start_time is None:
                self.pause_start_time = self.now_time
                self.get_logger().info(f"Starting pause timer for {self.pause_duration} seconds.")
    
    def process_state_transitions(self):
        color = self.detection.color
        next_state = self.state
        
        if self.state == NO_SPHERE_IDLE:
            if color == SphereDetection.COLOR_GREEN: next_state = GREEN_SPHERE_FORWARD
            elif color == SphereDetection.COLOR_ORANGE: next_state = ORANGE_SPHERE_RIGHT
            elif color == SphereDetection.COLOR_BLUE: next_state = BLUE_SPHERE_LEFT
            elif color == SphereDetection.COLOR_YELLOW: next_state = YELLOW_SPHERE_PAUSE
            
        elif self.state == GREEN_SPHERE_FORWARD:
            if color == SphereDetection.COLOR_NONE: next_state = NO_SPHERE_IDLE
            elif color == SphereDetection.COLOR_ORANGE: next_state = ORANGE_SPHERE_RIGHT
            elif color == SphereDetection.COLOR_BLUE: next_state = BLUE_SPHERE_LEFT
            elif color == SphereDetection.COLOR_YELLOW: next_state = YELLOW_SPHERE_PAUSE
            
        elif self.state == ORANGE_SPHERE_RIGHT:
            if color == SphereDetection.COLOR_GREEN: next_state = GREEN_SPHERE_FORWARD
            elif color == SphereDetection.COLOR_NONE: next_state = NO_SPHERE_IDLE
            
        elif self.state == BLUE_SPHERE_LEFT:
            if color == SphereDetection.COLOR_GREEN: next_state = GREEN_SPHERE_FORWARD
            elif color == SphereDetection.COLOR_NONE: next_state = NO_SPHERE_IDLE
            
        elif self.state == YELLOW_SPHERE_PAUSE:
            # Check if pause duration has elapsed
            if self.pause_start_time is not None and (self.now_time - self.pause_start_time) > self.pause_duration:
                self.pause_start_time = None  # Reset timer
                if color == SphereDetection.COLOR_GREEN: next_state = GREEN_SPHERE_FORWARD
                elif color == SphereDetection.COLOR_NONE: next_state = NO_SPHERE_IDLE
                elif color == SphereDetection.COLOR_ORANGE: next_state = ORANGE_SPHERE_RIGHT
                elif color == SphereDetection.COLOR_BLUE: next_state = BLUE_SPHERE_LEFT
                else:  # Still yellow
                    next_state = YELLOW_SPHERE_PAUSE
        
        # Apply state transition if needed
        if next_state != self.state:
            self.get_logger().info(f'{self._state_name(self.state)} → {self._state_name(next_state)}.')
            self.state = next_state
    
    def _state_name(self, s):
        return {
            NO_SPHERE_IDLE: 'NO_SPHERE_IDLE',
            GREEN_SPHERE_FORWARD: 'GREEN_SPHERE_FORWARD',
            ORANGE_SPHERE_RIGHT: 'ORANGE_SPHERE_RIGHT',
            BLUE_SPHERE_LEFT: 'BLUE_SPHERE_LEFT',
            YELLOW_SPHERE_PAUSE: 'YELLOW_SPHERE_PAUSE'
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