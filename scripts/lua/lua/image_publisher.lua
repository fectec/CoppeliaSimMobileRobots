sim = require 'sim'
simROS2 = require('simROS2')

-- ============================================================================
-- This script is attached to a vision sensor mounted on a mobile robot
-- in CoppeliaSim. It captures RGB images from the sensor and publishes them
-- as ROS2 messages (sensor_msgs/Image) on the topic /camera/image.
-- ============================================================================

-- Get the camera handle
visionSensor = sim.getObject('/visionSensor')

-- Create a publisher for the image topic
imagePub = nil

function sysCall_init()
    imagePub = simROS2.createPublisher('/camera/image', 'sensor_msgs/msg/Image')
end

function sysCall_actuation()
    -- Get the sensor resolution (width, height)
    local resolution = sim.getVisionSensorResolution(visionSensor)
    local width = resolution[1]
    local height = resolution[2]
    
    -- Capture the image, returned as an array of floats
    local image = sim.getVisionSensorImage(visionSensor)
    
    -- Create the sensor_msgs/Image message
    local msg = {}
    msg.header = {}                 -- If needed, set the header (e.g., timestamp, frame_id)
    msg.height = height
    msg.width = width
    msg.encoding = "rgb8"           -- Assuming an 8-bit RGB image
    msg.is_bigendian = 1
    msg.step = width * 3            -- Number of bytes per line: width * 3 channels
    msg.data = {}
    
    -- Convert the image by multiplying each value by 255 
    -- (CoppeliaSim image values range from [0,1])
    for i = 1, #image do
        msg.data[i] = math.floor(image[i] * 255)
    end
    
    -- Publish the message
    simROS2.publish(imagePub, msg)
end

function sysCall_cleanup()
    -- Clean up publisher
    simROS2.shutdownPublisher(imagePub)
end