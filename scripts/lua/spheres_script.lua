sim     = require('sim')
simROS2 = require('simROS2')

-- Número de esferas / suscriptores que queremos
local N = 3  -- por ejemplo, 5 esferas nombradas "Sphere_0" .. "Sphere_4"

-- Tablas de ámbito superior
local sphereHandles = {}
local detectionSubs  = {}
local colorChanged   = {}

-- Función que genera el callback para la i-ésima suscripción
local function makeCallback(i)
    return function(msg)
        if msg.data then
            colorChanged[i] = true
        end
    end
end

-- Función para aplicar color a la i-ésima esfera
local function applyColor(i, rgb)
    sim.setShapeColor(
      sphereHandles[i],
      nil,
      sim.colorcomponent_ambient_diffuse,
      rgb
    )
end

function sysCall_init()
    -- 1) Obtener handles y inicializar flags
    for i = 0, N-1 do
        -- nota: obtenemos Sphere_i
        sphereHandles[i] = sim.getObjectHandle('Sphere_' .. i)
        colorChanged[i]  = false

        -- color inicial verde
        applyColor(i, {0,1,0})
    end

    -- 2) Crear N suscripciones dinámicamente
    for i = 0, N-1 do
        local topic = '/proximity_detection_' .. i
        detectionSubs[i] = simROS2.createSubscription(
          topic,
          'std_msgs/msg/Bool',
          makeCallback(i)
        )
        sim.addLog(
          sim.verbosity_scriptinfos,
          string.format('Subscribed to %s for Sphere_%d', topic, i)
        )
    end
end

function sysCall_actuation()
    -- 3) Procesar cola ROS2
    simROS2.spinSome()

    -- 4) Para cada esfera cuyo flag sea true, cambiar a azul
    for i = 0, N-1 do
        if colorChanged[i] then
            applyColor(i, {0,0,1})
            colorChanged[i] = false  -- opcionally reset if solo quieres un cambio único
            sim.addLog(
              sim.verbosity_scriptinfos,
              string.format('Sphere_%d turned blue', i)
            )
        end
    end
end

function sysCall_cleanup()
    -- 5) Cerrar todas las suscripciones
    for i = 0, N-1 do
        simROS2.shutdownSubscription(detectionSubs[i])
    end
end
