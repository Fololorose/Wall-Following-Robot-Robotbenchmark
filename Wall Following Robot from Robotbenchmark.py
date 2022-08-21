"""Sample Webots controller for the wall following benchmark."""

from controller import Robot


def getDistance(sensor):
    """
    Return the distance of an obstacle for a sensor.

    The value returned by the getValue() method of the distance sensors
    corresponds to a physical value (here we have a sonar, so it is the
    strength of the sonar ray). This function makes a conversion to a
    distance value in meters.
    """
    return ((1000 - sensor.getValue()) / 1000) * 10


# Maximum speed for the velocity value of the wheels.
# Don't change this value.
MAX_SPEED = 5.24

# Get pointer to the robot.
robot = Robot()

# Get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# Get pointer to the robot wheels motors.
leftWheel = robot.getMotor('left wheel')
rightWheel = robot.getMotor('right wheel')

# We will use the velocity parameter of the wheels, so we need to
# set the target position to infinity.
leftWheel.setPosition(float('inf'))
rightWheel.setPosition(float('inf'))

# Get and enable the distance sensors.
Sensor0 = robot.getDistanceSensor("so0")
Sensor0.enable(timestep)
Sensor1 = robot.getDistanceSensor("so1")
Sensor1.enable(timestep)
Sensor2 = robot.getDistanceSensor("so2")
Sensor2.enable(timestep)
Sensor3 = robot.getDistanceSensor("so3")
Sensor3.enable(timestep)
Sensor4 = robot.getDistanceSensor("so4")
Sensor4.enable(timestep)
Sensor15 = robot.getDistanceSensor("so15")
Sensor15.enable(timestep)

# Move forward until we are 50 cm away from the wall.
while robot.step(timestep) != -1:
    leftWheel.setVelocity(MAX_SPEED)
    rightWheel.setVelocity(MAX_SPEED)
    if getDistance(Sensor3) < 0.45:
        break

# Rotate clockwise until the wall is to our left.
while robot.step(timestep) != -1:
    # Rotate until there is a wall to our left, and nothing in front of us.
    leftWheel.setVelocity(MAX_SPEED)
    rightWheel.setVelocity(-MAX_SPEED)
    if getDistance(Sensor0) < 1:
        break

# Main loop.
while robot.step(timestep) != -1:
        
    # Wall is in front, we need to make sharp turn to right
    if getDistance(Sensor4) < 0.7:
        leftWheel.setVelocity(MAX_SPEED)
        rightWheel.setVelocity(-MAX_SPEED)
        if getDistance(Sensor4) == 0.7:
            continue
        
    elif getDistance(Sensor3) < 0.7:
        leftWheel.setVelocity(MAX_SPEED)
        rightWheel.setVelocity(-MAX_SPEED)
        if getDistance(Sensor3) == 0.7:
            continue
            
    elif getDistance(Sensor2) < 0.7:
        leftWheel.setVelocity(MAX_SPEED)
        rightWheel.setVelocity(MAX_SPEED * 0.1)
        if getDistance(Sensor2) == 0.7:
            continue
        
    elif getDistance(Sensor1) < 0.7:
        leftWheel.setVelocity(MAX_SPEED)
        rightWheel.setVelocity(MAX_SPEED * 0.2)
        if getDistance(Sensor1) == 0.7:
            continue
        
    # Too close to the wall, we need to turn right.    
    elif getDistance(Sensor0) < 0.7:
        
        # Slightly right turn
        if getDistance(Sensor0) < 0.3:
            leftWheel.setVelocity(MAX_SPEED)
            rightWheel.setVelocity(MAX_SPEED * 0.7)
            if getDistance(Sensor0) == 0.7:
                continue
                
        elif getDistance(Sensor0) < 0.5:
            leftWheel.setVelocity(MAX_SPEED)
            rightWheel.setVelocity(MAX_SPEED * 0.8)
            if getDistance(Sensor0) == 0.7:
                continue
                    
        else:
            leftWheel.setVelocity(MAX_SPEED)
            rightWheel.setVelocity(MAX_SPEED * 0.9)
            if getDistance(Sensor0) == 0.7:
                continue
    
    # Too far from the wall, we need to turn left.
    elif getDistance(Sensor0) > 0.7:
        
        # Sharp turn to left
        if getDistance(Sensor15) > 0.4:
            leftWheel.setVelocity(MAX_SPEED * 0.5)
            rightWheel.setVelocity(MAX_SPEED)
            if getDistance(Sensor0) == 0.7:
                continue
                    
        # Slightly left turn
        else:
            if getDistance(Sensor0) > 1.1:
                leftWheel.setVelocity(MAX_SPEED * 0.7)
                rightWheel.setVelocity(MAX_SPEED)
                if getDistance(Sensor0) == 0.7:
                    continue
                    
            elif getDistance(Sensor0) > 0.9:
                leftWheel.setVelocity(MAX_SPEED * 0.8)
                rightWheel.setVelocity(MAX_SPEED)
                if getDistance(Sensor0) == 0.7:
                    continue
                
            else:
                leftWheel.setVelocity(MAX_SPEED * 0.9)
                rightWheel.setVelocity(MAX_SPEED)
                if getDistance(Sensor0) == 0.7:
                    continue
                
    # We are in the right direction.
    else:
        leftWheel.setVelocity(MAX_SPEED)
        rightWheel.setVelocity(MAX_SPEED)

# Stop the robot when we are done.
leftWheel.setVelocity(0)
rightWheel.setVelocity(0)
