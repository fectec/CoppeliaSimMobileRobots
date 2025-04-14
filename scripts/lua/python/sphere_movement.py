import math

# ============================================================================
# This script is attached to a sphere in CoppeliaSim.
# It causes the sphere to move back and forth along the Y-axis in a sine wave
# pattern, simulating oscillatory motion. The motion is defined by adjustable
# amplitude and frequency values.
# ============================================================================

def sysCall_init():
    sim = require('sim')
    
    # Get the handle for the sphere
    self.sphereHandle = sim.getObject('/Sphere')
    
    # Record the current simulation time
    self.startTime = sim.getSimulationTime()
    
    # Customize the amplitude/frequency of the sine movement
    self.amplitude = 0.5   # Movement range along Y-axis
    self.frequency = 0.2   # Oscillation frequency in Hz

def sysCall_actuation():
    # Calculate elapsed time
    currentTime = sim.getSimulationTime()
    elapsed = currentTime - self.startTime
    
    # Sine-based displacement    
    displacement = self.amplitude * math.sin(2 * math.pi * self.frequency * elapsed)
    
    # Get current position in the world frame (-1)
    position = sim.getObjectPosition(self.sphereHandle, -1)
    
    # Update the Y component
    position[1] = displacement
    
    # Apply the new position
    sim.setObjectPosition(self.sphereHandle, -1, position)