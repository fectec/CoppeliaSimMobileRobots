sim = require 'sim'
simROS2 = require('simROS2')

-- Obtén el handle de la cámara (asegúrate de usar la ruta o nombre correcto)
visionSensor = sim.getObject('/visionSensor')

-- Crea el publicador para el tópico de la imagen
imagePub = nil

function sysCall_init()
    imagePub = simROS2.createPublisher('/camera/image', 'sensor_msgs/msg/Image')
end

function sysCall_actuation()
    -- Obtén la resolución del sensor (ancho, alto)
    local resolution = sim.getVisionSensorResolution(visionSensor)  -- {ancho, alto}
    local width = resolution[1]
    local height = resolution[2]
    
    -- Captura la imagen. La imagen se devuelve como un vector (tabla) de valores flotantes
    local image = sim.getVisionSensorImage(visionSensor)
    
    -- Crea el mensaje de tipo sensor_msgs/Image
    local msg = {}
    msg.header = {}                  -- Si fuera necesario, asigna la cabecera (p. ej. timestamp, frame_id)
    msg.height = height
    msg.width = width
    msg.encoding = "rgb8"            -- Asumiendo que la imagen es RGB de 8 bits
    msg.is_bigendian = 0
    msg.step = width * 3             -- Número de bytes en una línea: ancho*3 canales
    msg.data = {}
    
    -- Convierte la imagen: cada valor multiplicado por 255 (la imagen de CoppeliaSim es un vector de [0,1])
    for i = 1, #image do
        msg.data[i] = math.floor(image[i] * 255)
    end
    
    -- Publica el mensaje
    simROS2.publish(imagePub, msg)
end

function sysCall_cleanup()
    simROS2.shutdownPublisher(imagePub)
end
sim = require 'sim'
simROS2 = require('simROS2')

-- Obtén el handle de la cámara (asegúrate de usar la ruta o nombre correcto)
visionSensor = sim.getObject('/visionSensor')

-- Crea el publicador para el tópico de la imagen
imagePub = nil

function sysCall_init()
    imagePub = simROS2.createPublisher('/camera/image', 'sensor_msgs/msg/Image')
end

function sysCall_actuation()
    -- Obtén la resolución del sensor (ancho, alto)
    local resolution = sim.getVisionSensorResolution(visionSensor)  -- {ancho, alto}
    local width = resolution[1]
    local height = resolution[2]
    
    -- Captura la imagen. La imagen se devuelve como un vector (tabla) de valores flotantes
    local image = sim.getVisionSensorImage(visionSensor)
    
    -- Crea el mensaje de tipo sensor_msgs/Image
    local msg = {}
    msg.header = {}                  -- Si fuera necesario, asigna la cabecera (p. ej. timestamp, frame_id)
    msg.height = height
    msg.width = width
    msg.encoding = "rgb8"            -- Asumiendo que la imagen es RGB de 8 bits
    msg.is_bigendian = 0
    msg.step = width * 3             -- Número de bytes en una línea: ancho*3 canales
    msg.data = {}
    
    -- Convierte la imagen: cada valor multiplicado por 255 (la imagen de CoppeliaSim es un vector de [0,1])
    for i = 1, #image do
        msg.data[i] = math.floor(image[i] * 255)
    end
    
    -- Publica el mensaje
    simROS2.publish(imagePub, msg)
end

function sysCall_cleanup()
    simROS2.shutdownPublisher(imagePub)
end
