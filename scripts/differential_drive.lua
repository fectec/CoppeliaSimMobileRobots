sim = require 'sim'
simROS2 = require 'simROS2'
simUI = require 'simUI'

-- Declaración de variables globales
ui = nil
objHandle = nil
lineTracerBase = nil
leftSensor = nil
middleSensor = nil
rightSensor = nil
leftJoint = nil
rightJoint = nil
leftJointDynamic = nil
rightJointDynamic = nil
velSub = nil

-- Parámetros para el control teleop
wheel_radius = 0.195/2    -- Radio de la rueda (en metros)
robot_width = 0.33        -- Distancia entre ruedas (en metros)

-- Función para actualizar la interfaz (LEDs) de los sensores
setLeds = function(elHandle, left, middle, right)
    simUI.setCheckboxValue(ui, 1, left and 2 or 0)
    simUI.setCheckboxValue(ui, 2, middle and 2 or 0)
    simUI.setCheckboxValue(ui, 3, right and 2 or 0)
end

-- Función de inicialización del script
function sysCall_init()
    local xml = [[
    <ui title="Line tracer, sensor display" closeable="false" placement="relative" position="50,-50" layout="grid">
        <label text=""/>
        <checkbox id="1" enabled="false" text="" style="* {font-size: 20px; font-weight: bold; margin-left: 20px; margin-right: 20px;}"/>
        <checkbox id="2" enabled="false" text="" style="* {font-size: 20px; font-weight: bold; margin-left: 20px; margin-right: 20px;}"/>
        <checkbox id="3" enabled="false" text="" style="* {font-size: 20px; font-weight: bold; margin-left: 20px; margin-right: 20px;}"/>
    </ui>
    ]]
    ui = simUI.create(xml)

    -- Obtención de handles de los objetos y sensores
    objHandle = sim.getObject('..')
    lineTracerBase = sim.getObject("../LineTracerBase")
    leftSensor = sim.getObject("../LeftSensor")
    middleSensor = sim.getObject("../MiddleSensor")
    rightSensor = sim.getObject("../RightSensor")
    leftJoint = sim.getObject("../LeftJoint")
    rightJoint = sim.getObject("../RightJoint")
    leftJointDynamic = sim.getObject("../DynamicLeftJoint")
    rightJointDynamic = sim.getObject("../DynamicRightJoint")

    -- Se activa la suscripción al tópico '/cmd_vel' para recibir Twist
    velSub = simROS2.createSubscription('/cmd_vel', 'geometry_msgs/msg/Twist', 'setVelocity_cb')
end

-- Callback que se invoca cuando se recibe un mensaje en /cmd_vel
function setVelocity_cb(msg)
    -- Extraemos la velocidad lineal (m/s) y angular (rad/s)
    local linear = msg.linear.x
    local angular = msg.angular.z

    -- Cálculo de la velocidad de cada rueda utilizando la cinemática diferencial:
    local vl = linear - (angular * robot_width) / 2
    local vr = linear + (angular * robot_width) / 2

    -- Conversión de velocidad lineal (m/s) a velocidad angular (rad/s):
    sim.setJointTargetVelocity(leftJointDynamic, vl / wheel_radius)
    sim.setJointTargetVelocity(rightJointDynamic, vr / wheel_radius)
end

-- Función de actuation que se llama cada ciclo de simulación
function sysCall_actuation()
    -- Aunque el control teleop actualiza las velocidades en el callback,
    -- aquí se pueden agregar lecturas de sensores o actualizaciones visuales.
    local sensorLeft = sim.readVisionSensor(leftSensor)
    local sensorMiddle = sim.readVisionSensor(middleSensor)
    local sensorRight = sim.readVisionSensor(rightSensor)
    
    -- Actualización de los indicadores (LEDs) en la UI:
    setLeds(nil, sensorLeft==1, sensorMiddle==1, sensorRight==1)
end

-- Función de limpieza (al finalizar la simulación)
function sysCall_cleanup()
    simROS2.shutdownSubscription(velSub)
end
