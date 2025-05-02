sim     = require('sim')
simROS2 = require('simROS2')

-- Número de esferas
local N = 5

-- Tablas para guardar handles y subscriptions
local sphereHandles = {}
local colorSubs     = {}

-- Generador de callbacks: captura el índice i
local function makeColorCallback(i)
  return function(msg)
    -- Extraer RGB del mensaje
    local r, g, b = msg.r, msg.g, msg.b
    -- Log para depuración
    sim.addLog(sim.verbosity_scriptinfos,
      string.format('Received color for sphere %d: {%.3f, %.3f, %.3f}', i, r, g, b)
    )
    -- Aplicar al sphere i
    sim.setShapeColor(
      sphereHandles[i],
      nil,
      sim.colorcomponent_ambient_diffuse,
      {r, g, b}
    )
  end
end

function sysCall_init()
  -- 1) Obtener handles y suscribirse
  for i = 1, N-1 do
    -- Asume objetos nombrados "Sphere_0", "Sphere_1", ..., "Sphere_N-1"
    sphereHandles[i] = sim.getObjectHandle('Sphere_' .. i)
    
    local topic = string.format('/sphere%d/color', i)
    colorSubs[i] = simROS2.createSubscription(
      topic,
      'std_msgs/msg/ColorRGBA',
      makeColorCallback(i)
    )
    
    sim.addLog(sim.verbosity_scriptinfos,
      string.format('Subscribed to %s', topic)
    )
  end
end

function sysCall_actuation()
  -- 2) Procesar cola ROS2 para disparar los callbacks
  simROS2.spinSome()
end

function sysCall_cleanup()
  -- 3) Cerrar todas las suscripciones
  for i = 0, N-1 do
    simROS2.shutdownSubscription(colorSubs[i])
  end
end
