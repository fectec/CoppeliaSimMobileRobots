-- IMPORTAR APIs
local sim      = require('sim')      -- API base de CoppeliaSim
local simROS2  = require('simROS2')  -- Plugin ROS 2

-- VARIABLES
local publisher
local sensorHandle
local msg = { data = false }

function sysCall_init()
    -- Publicador de bool en /proximity_detection
    publisher = simROS2.createPublisher(
      '/proximity_detection_0',          -- tópico
      'std_msgs/msg/Bool'              -- tipo completo ROS 2
    )
    -- Handle del sensor de tipo Ray
    sensorHandle = sim.getObjectHandle('proximitySensor_0')
end

function sysCall_actuation()
    -- Leer el sensor (1 si detecta, 0 si no)
    local detected = sim.readProximitySensor(sensorHandle)
    msg.data = (detected == 1)
    -- Publicar detección
    simROS2.publish(publisher, msg)
    -- Procesar callbacks internos
    simROS2.spinSome()
end

function sysCall_cleanup()
    -- Cerrar publicador
    simROS2.shutdownPublisher(publisher)
end
