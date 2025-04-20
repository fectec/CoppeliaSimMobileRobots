# Coppelia Differential Drive

<p align="justify">
ROS2-based simulation of a differential-drive robot in CoppeliaSim with visual servoing. Developed as part of the undergraduate course <strong>"Intelligent Robotics Implementation."</strong>
</p>

<p align="justify">
To run this simulation, you will need <a href="https://docs.ros.org/en/humble/Installation.html" target="_blank">ROS 2 Humble</a> and <a href="https://www.coppeliarobotics.com/" target="_blank">CoppeliaSim v4.9.0 rev6</a> installed on your system.
</p>

<p align="justify">
Once both are installed, use the <code>interface_setup</code> script located in the root of this repository to build the CoppeliaSim ROS2 interface (<code>sim_ros2_interface</code> package).
</p>

<p align="justify">
Inside CoppeliaSim, go to <code>File → Open Scene</code> and load the <code>camera_car.ttt</code> file located in the <code>scenes</code> folder. This will load the full simulation environment, including the Lua and Python scripts attached to the objects, which you can also find in the <code>scripts</code> folder:
</p>

<ul>
  <li><strong><code>simulation_time_publisher.lua</code></strong><br>
    Publishes the current simulation time (in seconds) on the <code>/simulationTime</code> ROS 2 topic using a <code>std_msgs/msg/Float32</code> message. Helps synchronize ROS 2 nodes with the simulation clock.
  </li>

  <li><strong><code>differential_drive.lua</code></strong><br>
    Attached to the differential-drive robot. Implements inverse kinematics: it subscribes to <code>/cmd_vel</code> (linear and angular velocity commands), converts them into left and right wheel angular speeds, and applies them. It also publishes the actual measured wheel speeds to <code>/VelocityEncL</code> and <code>/VelocityEncR</code>.
  </li>

  <li><strong><code>image_publisher.lua</code></strong><br>
    Attached to the onboard vision sensor. Captures RGB images from the camera and publishes them to <code>/camera/image</code> as <code>sensor_msgs/msg/Image</code>.
  </li>

  <li><strong><code>sphere_movement.py</code></strong><br>
    Controls a green sphere in the simulation. Moves it back and forth along the Y-axis in a sinusoidal pattern. Used for visual servoing.
  </li>
</ul>

## Sphere Following Using Position Control

<p align="justify">
First, this project implements a visual servoing system where a differential-drive robot tracks and follows a green sphere using position-based PID control. A camera mounted on the robot detects the sphere through HSV color segmentation. The centroid of the sphere is used to compute a heading correction, from which a target waypoint is generated a fixed distance ahead. This waypoint is sent to a PID controller that computes the linear and angular velocities needed to drive the robot toward the goal using odometry feedback. Here’s a quick breakdown of the involved files:
</p>

<ul>
  <li><strong><code>vision/visual_servoing.py</code></strong><br>
    Detects the green sphere in camera images and publishes a 2D waypoint ahead of the robot.
  </li>

  <li><strong><code>control/localization.py</code></strong><br>
    Computes the robot’s pose using dead reckoning from wheel speed data and simulation time.
  </li>

  <li><strong><code>control/point_controller.py</code></strong><br>
    PID position controller that computes velocity commands (<code>cmd_vel</code>) based on the robot’s pose and goal.
  </li>
</ul>

![rqt_graph of Sphere Following Using Position Control](https://github.com/user-attachments/assets/71889e6f-da28-46a1-a3c6-3492d6153732)

<p align="justify"> Start the simulation in CoppeliaSim, and after building all packages (except <code>sim_ros2_interface</code>), run the ROS2 system using: </p> 

```bash
ros2 launch bringup bringup.launch.py
```
<p align="center"> 
  <img src="https://github.com/user-attachments/assets/f640f153-d37f-4a34-aa62-dc3051c473d1" alt="Sphere Following Using Position Control Demo" width="600"/> 
</p>
