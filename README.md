# Coppelia Differential Drive

<p align="justify">
ROS2-based simulation of a differential-drive robot in CoppeliaSim with visual servoing. Developed as part of the undergraduate course <strong>"Intelligent Robotics Implementation."</strong>
</p>

<p align="center">
  <img src="https://github.com/user-attachments/assets/5dd5a6e0-9a61-4b2f-b66c-d648c613e416" alt="CoppeliaSim Logo" width="40%"/>
</p>

<p align="justify">
To run this simulation, you will need <a href="https://docs.ros.org/en/humble/Installation.html" target="_blank">ROS 2 Humble</a> and <a href="https://www.coppeliarobotics.com/" target="_blank">CoppeliaSim v4.9.0 rev6</a> installed on your system.
</p>

<p align="justify">
Once both are installed, use the <code>interface_setup</code> script located in the root of this repository to build the CoppeliaSim ROS2 interface (<code>sim_ros2_interface</code> package).
</p>

<p align="justify">
Inside CoppeliaSim, go to <code>File → Open Scene</code> and load the desired <code>.ttt</code> file located in the <code>scenes</code> folder. This will load the full simulation environment, including the Lua and Python scripts attached to the objects, which you can also find in the <code>scripts</code> folder:
</p>

<ul>
  <li><strong><code>simulation_time_publisher.lua</code></strong><br>
    Publishes the current simulation time (in seconds) on the <code>/simulationTime</code> ROS2 topic using a <code>std_msgs/msg/Float32</code> message. Helps synchronize ROS2 nodes with the simulation clock.
  </li>

  <li><strong><code>differential_drive.lua</code></strong><br>
    Attached to the differential-drive robot. Implements inverse kinematics: it subscribes to <code>/cmd_vel</code> (linear and angular velocity commands), converts them into left and right wheel angular speeds, and applies them. It also publishes the actual measured wheel speeds to <code>/VelocityEncL</code> and <code>/VelocityEncR</code>.
  </li>

  <li><strong><code>image_publisher.lua</code></strong><br>
    Attached to the onboard vision sensor. Captures RGB images from the camera and publishes them to <code>/camera/image</code> as <code>sensor_msgs/msg/Image</code>.
  </li>

  <li><strong><code>green_sphere_movement.py</code></strong><br>
    Controls a green sphere in the simulation. Moves it back and forth along the Y-axis in a sinusoidal pattern. Used for visual servoing.
  </li>
</ul>

## Green Sphere Following Using Point-to-Point PID Control

<p align="justify">
First, this project implements a visual servoing system where a differential-drive robot tracks and follows a green sphere using point-to-point PID control. A camera mounted on the robot detects the sphere through HSV color segmentation. The centroid of the sphere is used to compute a heading correction, from which a target waypoint is generated a fixed distance ahead. This waypoint is sent to a PID controller that computes the linear and angular velocities needed to drive the robot toward the goal using odometry feedback. 
</p>

<p align="center"> 
  <img src="https://github.com/user-attachments/assets/f640f153-d37f-4a34-aa62-dc3051c473d1" alt="Green Sphere Following Using Point-to-Point Control Demo" width="600"/> 
</p>

<p align="justify"> 
Here’s a quick breakdown of the involved files:
</p>

<ul>
  <li><strong><code>sim_diff_drive_vision/green_sphere_following.py</code></strong><br>
    Detects the green sphere in camera images and publishes a 2D waypoint ahead of the robot.
  </li>

  <li><strong><code>sim_diff_drive_control/localization.py</code></strong><br>
    Computes the robot’s pose using dead reckoning from wheel speed data and simulation time.
  </li>

  <li><strong><code>sim_diff_drive_control/point_controller.py</code></strong><br>
    PID point-to-point controller that computes velocity commands (<code>cmd_vel</code>) based on the robot’s pose and goal.
  </li>
</ul>

<p align="center"> 
  <img src="https://github.com/user-attachments/assets/71889e6f-da28-46a1-a3c6-3492d6153732" alt="rqt_graph of Green Sphere Following Using Point-to-Point Control" width="600"/> 
</p>

<p align="justify">
Open the <code>green_sphere_following.ttt</code> file, start the simulation in CoppeliaSim, and after building all packages (except <code>sim_ros2_interface</code>), run the system using:
</p>

```bash
ros2 launch sim_diff_drive_bringup green_sphere_following.launch.py
```

## Finite State Machine for Local Trajectory Control

<p align="justify">
Next, a Finite State Machine (FSM) approach was developed to allow the robot to navigate its environment through visual servoing with different behaviors based on colored sphere detection, and can be found on <strong><code>sim_diff_drive_behavior/local_trajectory_sphere_fsm.py</code></strong>.
</p>

<p align="center">
  <img src="https://github.com/user-attachments/assets/2ba39ad2-a956-452c-8a68-8b5c23c6171d" alt="Finite State Machine for Local Trajectory Control Demo" width="600"/>
</p>

<ul>
  <li><strong>No Sphere:</strong> Robot stops and remains idle.</li>
  <li><strong>Green Sphere:</strong> Robot moves forward toward the sphere.</li>
  <li><strong>Orange Sphere:</strong> Robot turns right until it sees a green sphere again.</li>
  <li><strong>Blue Sphere:</strong> Robot turns left until it sees a green sphere again.</li>
  <li><strong>Yellow Sphere:</strong> Robot pauses all movement for 5 seconds.</li>
</ul>

<p align="justify">
The FSM implements the following states and actions:
</p>

<ul>
  <li><p align="justify"><strong>NO_SPHERE_IDLE:</strong> Publishes an empty Twist message to stop the robot.</p></li>
  <li><p align="justify"><strong>GREEN_SPHERE_FORWARD:</strong> Sends waypoint coordinates to the Point-to-Point PID controller, similar to the earlier green sphere following approach.</p></li>
  <li><p align="justify"><strong>ORANGE_SPHERE_RIGHT:</strong> Directly publishes a Twist message with negative angular velocity to turn right.</p></li>
  <li><p align="justify"><strong>BLUE_SPHERE_LEFT:</strong> Directly publishes a Twist message with positive angular velocity to turn left.</p></li>
  <li><p align="justify"><strong>YELLOW_SPHERE_PAUSE:</strong> Publishes an empty Twist message and starts a timer for the pause duration.</p></li>
</ul>

<p align="justify">
The state transitions follow these rules:
</p>

<ul>
  <li><p align="justify">From <strong>NO_SPHERE_IDLE</strong>: Transitions to any other state if the corresponding colored sphere is detected.</p></li>
  <li><p align="justify">From <strong>GREEN_SPHERE_FORWARD</strong>: Can transition to any other state based on sphere detection.</p></li>
  <li><p align="justify">From <strong>ORANGE_SPHERE_RIGHT</strong> or <strong>BLUE_SPHERE_LEFT</strong>: Transitions to GREEN_SPHERE_FORWARD if a green sphere is detected, or to NO_SPHERE_IDLE if no sphere is detected.</p></li>
  <li><p align="justify">From <strong>YELLOW_SPHERE_PAUSE</strong>: After the pause duration (5 seconds), transitions to any state based on sphere detection.</p></li>
</ul>

<p align="justify">
This system utilizes a general sphere detection node <strong><code>sim_diff_drive_vision/sphere_detection.py</code></strong> that can recognize all configured color spheres using HSV thresholding, and publishes both the detected color and a 2D waypoint ahead of the robot in a custom <code>SphereDetection.msg</code> message.</p> 

<p align="justify">
If multiple colors are on the scene, the detection algorithm prioritizes green spheres for waypoint calculation, and if no green sphere is present, it uses the color with the largest area.</p> 

<p align="justify">
The color attribute of the message is used for state transitions by the FSM node.
</p>

<p align="justify">
For creating the local route, special spheres in the scene can change colors when the robot triggers proximity sensors. These interactive spheres use scripts to listen for sensor detection events and modify their color properties accordingly, allowing the robot to respond dynamically to its environment.
</p>

![Finite State Machine for Local Trajectory Control rqt_graph](https://github.com/user-attachments/assets/fe7f0671-f155-4162-b13b-f7cf99486d57)

<p align="justify">
Open the <code>local_path_FSM.ttt</code> file, start the simulation in CoppeliaSim, and after building all packages (except <code>sim_ros2_interface</code>), run the setup using:
</p>

```bash
ros2 launch sim_diff_drive_bringup local_trajectory_sphere_fsm.launch.py
```
