import json
import numpy as np
from math import ceil
import roboticstoolbox as rtb
from spatialmath import SE3
from controller import Supervisor

TIME_STEP = 32
POSITION_THRESHOLD = 0.002

panda = rtb.models.DH.Panda()           # Panda model for inverse kinematics
arm_node = Supervisor()                 # Webots' arm node
arm_base = arm_node.getFromDef("PANDA") # Arm's base node for basing translations and rotations of the end-effector
motors = []                             # Motor's nodes
sensors = []                            # Positional sensors of the different motors

# Initialization of the Panda joints and sensors
def init_arm_components():

    # Recover joints and sensors for moving the arm
    for i in range(7):
        motor = arm_node.getDevice("panda_joint" + str(i+1))
        motor.setVelocity(1)
        motors.append(motor)

        sensor = motor.getPositionSensor()
        sensor.enable(TIME_STEP)
        sensors.append(sensor)
    
    # Recover hand joints
    lfinger = arm_node.getDevice("panda_finger::left")
    lfinger.setVelocity(1)
    motors.append(lfinger)

    rfinger = arm_node.getDevice("panda_finger::right")
    rfinger.setVelocity(1)
    motors.append(rfinger)
    return


# Move the arm to the given position
def move_arm(final_position):
    arm_pos = []

    """ Before moving the arm to the pointed location, the exact angle of each joint
        has to be computed. To do this, the model of the Panda robotic arm from the roboticstoolbox
        library is harnessed. For the values to be corret, the current positions of the joints have to be provided though. """
    for i in range(7):
        arm_pos.append(sensors[i].getValue())

    panda.q = np.array(arm_pos)
    x = final_position[0] - arm_base.getPosition()[0]
    y = final_position[1] - arm_base.getPosition()[1]
    z = final_position[2] # Since the block and the arm's base are at the same height

    # Compute transformation matrix
    T_matrix = SE3.Trans(x, y, z) * SE3.OA([0, 1, 0], [0, 0, -1])
    
    # Search for a valid solution to the inverse kinematic problem
    not_valid = True
    while not_valid == True:
        not_valid = False
        inverse_kin_sol = (panda.ikine_LM(T_matrix)).q
        for i in range(7):
            if not (motors[i].getMinPosition() < inverse_kin_sol[i] and motors[i].getMaxPosition() > inverse_kin_sol[i]):
                not_valid = True

    for i in range(7):
        motors[i].setPosition(inverse_kin_sol[i])

    # Perform the movement until it is finished
    for i in range(7):
        while (is_position_reached(motors[i], inverse_kin_sol[i]) == False):
            arm_node.step(TIME_STEP)

    return 0


# Check if a joint has reached the requested position
def is_position_reached(motor, pos):
    return True if ((abs(motor.getPositionSensor().getValue() - pos)) <= POSITION_THRESHOLD) else False


# Check if the block position has changed
def is_pos_new(old, current):
    is_new = False

    for i in range(3):
        if (ceil(old[i] * 1000.0) / 1000.0) != (ceil(current[i] * 1000.0) / 1000.0):
            is_new = True
            break

    return is_new


def main():
    # Initialize the components
    print("Initializing Panda motors and sensors...")
    init_arm_components()
    red_block = arm_node.getFromDef("REDBLOCK")

    # Run
    old_pos = [0, 0, 0]
    while arm_node.step(TIME_STEP) != -1:
        # DEFINE POSITION
        pos = red_block.getPosition()
        pos[2] = 0.12                    # Position the EE above the block, without moving it

        # MOVE TO POSITION
        if is_pos_new(old_pos, pos) == True:
            old_pos = pos
            move_arm(pos)

    return


if __name__ == "__main__":
    main()