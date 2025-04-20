sim = require 'sim'
simROS2 = require 'simROS2'

-- ============================================================================
-- This script is attached to a differential‐drive robot in CoppeliaSim.
-- It performs two main functions:
-- 1. It subscribes to the /cmd_vel topic (geometry_msgs/msg/Twist)
--    to command the robot’s motion.
-- 2. It reads the dynamic joints’ actual velocities
--    (using sim.getJointVelocity) and publishes these values to
--    /VelocityEncL and /VelocityEncR as std_msgs/msg/Float32 messages.
--    These measured values can then be used for dead-reckoning (localization).
-- ============================================================================
 
-- Global variables
velSub = nil
velEncLPub = nil
velEncRPub = nil
leftJointDynamic = nil
rightJointDynamic = nil

-- Set parameters (update these as needed to match your robot)
wheel_radius = 0.027      -- Wheel radius in meters
wheel_base   = 0.119      -- Distance between wheels in meters

function sysCall_init()
    -- Retrieve handles for the dynamic joints
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

    -- Subscribe to /cmd_vel (geometry_msgs/msg/Twist)
    velSub = simROS2.createSubscription('/cmd_vel', 'geometry_msgs/msg/Twist', 'setVelocity_cb')
    sim.addLog(sim.verbosity_scriptinfos, "Subscribed to /cmd_vel for teleop control")

    -- Create publishers for the actual (measured) wheel speeds
    velEncLPub = simROS2.createPublisher('/VelocityEncL', 'std_msgs/msg/Float32')
    velEncRPub = simROS2.createPublisher('/VelocityEncR', 'std_msgs/msg/Float32')
    sim.addLog(sim.verbosity_scriptinfos, "Publishing wheel encoders on /VelocityEncL and /VelocityEncR")
end

-- Callback to process incoming Twist messages
function setVelocity_cb(msg)
    local linear = msg.linear.x    -- Forward speed (m/s)
    local angular = msg.angular.z  -- Rotational speed (rad/s)

    -- Compute desired wheel linear velocities using differential-drive kinematics:
    local v_left  = linear - (angular * wheel_base / 2)
    local v_right = linear + (angular * wheel_base / 2)
    
    -- Convert to desired wheel angular velocities (rad/s)
    local leftTargetSpeed  = v_left / wheel_radius
    local rightTargetSpeed = v_right / wheel_radius

    -- Set the target velocities on the dynamic joints
    sim.setJointTargetVelocity(leftJointDynamic, leftTargetSpeed)
    sim.setJointTargetVelocity(rightJointDynamic, rightTargetSpeed)
    
    sim.addLog(sim.verbosity_scriptinfos,
        string.format("setVelocity_cb: linear=%.3f, angular=%.3f -> leftTargetSpeed=%.3f, rightTargetSpeed=%.3f", 
        linear, angular, leftTargetSpeed, rightTargetSpeed)
    )
end

-- In sysCall_actuation, we read the actual joint velocities and publish them.
function sysCall_actuation()
    local leftSpeedMeasured = sim.getJointVelocity(leftJointDynamic)
    local rightSpeedMeasured = sim.getJointVelocity(rightJointDynamic)

    simROS2.publish(velEncLPub, {data = leftSpeedMeasured})
    simROS2.publish(velEncRPub, {data = rightSpeedMeasured})
end

function sysCall_cleanup()
    simROS2.shutdownSubscription(velSub)
    simROS2.shutdownPublisher(velEncLPub)
    simROS2.shutdownPublisher(velEncRPub)
end