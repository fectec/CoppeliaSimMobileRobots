-- importar APIs
local sim     = require('sim')
local simROS2 = require('simROS2')

-- número de sensores a manejar
local N = 3  -- cámbialo al número de proximity sensors que tengas

-- tablas para almacenar handles y publishers
local sensorHandles = {}
local publishers    = {}

function sysCall_init()
    -- crear un publisher por cada sensor y obtener su handle
    for i = 0, N-1 do
        -- get handle del objeto
        local h = sim.getObjectHandle('proximitySensor_' .. i)
        sensorHandles[i] = h

        -- crear publisher en /proximity_detection_i
        local topic = '/proximity_detection_' .. i
        publishers[i] = simROS2.createPublisher(topic, 'std_msgs/msg/Bool')

        sim.addLog(sim.verbosity_scriptinfos,
          string.format('Publisher creado en %s para sensor %d', topic, i)
        )
    end
end

function sysCall_actuation()
    -- por cada sensor: leer y publicar
    for i = 0, N-1 do
        local detected = sim.readProximitySensor(sensorHandles[i])
        local msg = { data = (detected == 1) }
        simROS2.publish(publishers[i], msg)
    end
    -- opcional: procesar callbacks
    simROS2.spinSome()
end

function sysCall_cleanup()
    -- cerrar todos los publishers
    for i = 0, N-1 do
        simROS2.shutdownPublisher(publishers[i])
    end
end
