sim = require 'sim'
simROS2 = require 'simROS2'

-- Global variables
velSub = nil
leftJointDynamic = nil
rightJointDynamic = nil

-- Teleoperation parameters
wheel_radius = 0.195/2    -- Wheel radius in meters
robot_width  = 0.33       -- Distance between wheels in meters

-- Initialization function
function sysCall_init()
    -- Retrieve handles for the dynamic joints.
    -- Update these paths to match your scene's hierarchy.
    leftJointDynamic = sim.getObject("../DynamicLeftJoint")
    rightJointDynamic = sim.getObject("../DynamicRightJoint")
    
    if leftJointDynamic == -1 then
        sim.addLog(sim.verbosity_errors, "DynamicLeftJoint not found! Check the object path.")
    else
        sim.addLog(sim.verbosity_scriptinfos, "Found DynamicLeftJoint: " .. leftJointDynamic)
    end
    
    if rightJointDynamic == -1 then
        sim.addLog(sim.verbosity_errors, "DynamicRightJoint not found! Check the object path.")
    else
        sim.addLog(sim.verbosity_scriptinfos, "Found DynamicRightJoint: " .. rightJointDynamic)
    end

    -- Subscribe to the /cmd_vel topic to receive Twist messages
    velSub = simROS2.createSubscription('/cmd_vel', 'geometry_msgs/msg/Twist', 'setVelocity_cb')
    sim.addLog(sim.verbosity_scriptinfos, "Subscribed to /cmd_vel for teleop control")
end

-- Callback to process incoming Twist messages from teleoperation
function setVelocity_cb(msg)
    local linear = msg.linear.x   -- Desired forward speed (m/s)
    local angular = msg.angular.z -- Desired rotational speed (rad/s)

    -- Compute individual wheel linear velocities (m/s)
    local v_left  = linear - (angular * robot_width / 2)
    local v_right = linear + (angular * robot_width / 2)
    
    -- Convert linear velocities to wheel angular velocities (rad/s)
    local leftSpeed  = v_left / wheel_radius
    local rightSpeed = v_right / wheel_radius

    -- Set the target velocities for the wheel joints
    sim.setJointTargetVelocity(leftJointDynamic, leftSpeed)
    sim.setJointTargetVelocity(rightJointDynamic, rightSpeed)
    
    sim.addLog(sim.verbosity_scriptinfos, string.format("setVelocity_cb: linear=%.3f, angular=%.3f -> leftSpeed=%.3f, rightSpeed=%.3f", 
        linear, angular, leftSpeed, rightSpeed))
end

-- Actuation function (called each simulation step)
function sysCall_actuation()
    -- No per-step actions needed here for teleoperation;
    -- velocity updates occur in the ROS2 callback.
end

-- Cleanup function (called at simulation end)
function sysCall_cleanup()
    simROS2.shutdownSubscription('/cmd_vel')
    sim.addLog(sim.verbosity_scriptinfos, "ROS2 subscription shutdown")
end
