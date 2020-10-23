"""panda_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import tinyik

arm_joints = [
  'panda_joint1',
  'panda_joint2',
  'panda_joint3',
  'panda_joint4',
  'panda_joint5',
  'panda_joint6',
  'panda_joint7'
]

hand_joints = [
  'panda_finger_joint1',
  'panda_finger_joint2',
]

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getMotor('motorname')
#  ds = robot.getDistanceSensor('dsname')
#  ds.enable(timestep)
arm_motors = []
for name in arm_joints:
  arm_motors.append(robot.getMotor(name))

hand_motors = []
for name in hand_joints:
  hand_motors.append(robot.getMotor(name))

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.
