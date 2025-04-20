# Coppelia Differential Drive

<p align="justify">
To run this simulation, you will need <a href="https://docs.ros.org/en/humble/Installation.html" target="_blank">ROS 2 Humble</a> and <a href="https://www.coppeliarobotics.com/" target="_blank">CoppeliaSim v4.9.0 rev6</a> installed on your system.
</p>

<p align="justify">
Once both are installed, use the <code>interface_setup</code> script located in the root of this repository to build the CoppeliaSim ROS2 interface.
</p>

## Sphere Following Using Position Control

<p align="justify">
First, this project implements a visual servoing system where a differential-drive robot tracks and follows a green sphere using position-based PID control. A camera mounted on the robot detects the sphere through HSV color segmentation. The centroid of the sphere is used to compute a heading correction, from which a target waypoint is generated a fixed distance ahead. This waypoint is sent to a PID controller that computes the linear and angular velocities needed to drive the robot toward the goal using odometry feedback. Here’s a quick breakdown of the involved files:
</p>

<ul>
  <li><strong><code>vision/visual_servoing.py</code></strong><br>
    Detects the green sphere in camera images and publishes a 2D waypoint ahead of the robot.
  </li>

  <li><strong><code>control/localization.py</code></strong><br>
    Computes the robot’s pose using dead reckoning from wheel encoder data and simulation time.
  </li>

  <li><strong><code>control/point_controller.py</code></strong><br>
    PID position controller that computes velocity commands (<code>cmd_vel</code>) based on the robot’s pose and goal.
  </li>
</ul>
