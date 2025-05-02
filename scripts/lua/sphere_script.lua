sim      = require('sim')
simROS2  = require('simROS2')

-- Variables de ámbito superior
local detectionSub
local sphereHandle_0
local colorChanged = false

-- Callback de la suscripción
local function setColorCallback(msg)
    if msg.data then
        colorChanged = true
    end
end

-- Aplicar color a la esfera
local function applyColor(rgb)
    sim.setShapeColor(
      sphereHandle_0, 
      nil, 
      sim.colorcomponent_ambient_diffuse, 
      rgb
    )
end

function sysCall_init()
    -- Obtener handle
    sphereHandle_0 = sim.getObjectHandle('Sphere_0')  -- :contentReference[oaicite:2]{index=2}

    -- Crear suscripción 
    detectionSub = simROS2.createSubscription(
      '/proximity_detection_0',
      'std_msgs/msg/Bool',
      setColorCallback 
    )
    sim.addLog(sim.verbosity_scriptinfos, "Suscripción a /proximity_detection_0 inicializada.")

    -- Color inicial: verde
    applyColor({0,1,0})
end

function sysCall_actuation()
    -- Procesar la cola de ROS2
    simROS2.spinSome() 

    -- 7) Si recibimos true, cambiamos a azul (una sola vez)
    if colorChanged then
        applyColor({0,0,1})
        colorChanged = false
        sim.addLog(sim.verbosity_scriptinfos, "Color cambiado a azul.")
    end
end

function sysCall_cleanup()
    simROS2.shutdownSubscription(detectionSub)
end
