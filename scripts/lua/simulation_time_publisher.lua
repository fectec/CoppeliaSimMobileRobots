sim = require 'sim'
simROS2 = require 'simROS2'

-- ============================================================================
-- This script can be attached to a simulation object in CoppeliaSim.
-- It performs one main function:
-- 1. Publishes the current simulation time (in seconds) on the '/simulationTime'
--    topic using std_msgs/msg/Float32.
--
-- This can be useful for synchronizing ROS2 nodes with the simulation clock.
-- ============================================================================
 
-- Global variable for the publisher
publisher = nil

function sysCall_init()
    -- Create a Float32 publisher for the '/simulationTime' topic
    publisher = simROS2.createPublisher('/simulationTime', 'std_msgs/msg/Float32')
    
    sim.addLog(sim.verbosity_scriptinfos, "Initialized simulationTime publisher.")
end

function sysCall_actuation()
    -- Publish the current simulation time (in seconds)
    local simulationTime = sim.getSimulationTime()
    simROS2.publish(publisher, {data = simulationTime})
end

function sysCall_cleanup()
    -- Shutdown the publisher
    simROS2.shutdownPublisher(publisher)
end