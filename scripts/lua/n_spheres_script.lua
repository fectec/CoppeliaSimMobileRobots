sim      = require('sim')
simROS2  = require('simROS2')

local N = 3
local sphereHandles = {}

-- Colores en el mismo orden que tu nodo Python
local sphere_colors = {'amarilla','azul','naranja'}
local color_map = {
  amarilla = {1.0, 1.0, 0.0},
  azul     = {0.0, 0.0, 1.0},
  naranja  = {1.0, 0.65, 0.0},
  verde    = {0.0, 1.0, 0.0}  -- fallback
}

function sysCall_init()
  for j = 1, N do
    local idx    = j - 1
    local handle = sim.getObjectHandle('Sphere_' .. idx)
    sphereHandles[j] = handle
    -- color inicial verde
    sim.setShapeColor(handle, nil, sim.colorcomponent_ambient_diffuse, color_map.verde)

    -- crear servicio con callback de un solo parámetro
    simROS2.createService(
      '/sphere' .. idx .. '/set_color',
      'std_srvs/srv/Trigger',
      function(req)
        -- determinar color y aplicar
        local cname = sphere_colors[j]
        local col   = color_map[cname] or color_map.verde
        sim.setShapeColor(handle, nil, sim.colorcomponent_ambient_diffuse, col)
        -- devolver tabla con success y message
        return {
          success = true,
          message = string.format('Sphere %d ? %s', idx, cname)
        }
      end
    )
    sim.addLog(sim.verbosity_scriptinfos,
      string.format('Service /sphere%d/set_color ready', idx)
    )
  end
end

function sysCall_actuation()
  simROS2.spinSome()
end

function sysCall_cleanup()
  -- simROS2.shutdownService if deseas limpiar manualmente
end
