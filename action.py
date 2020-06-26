
#% Basic Actions
def stop(robot):
    robot.simSetVel(0,0)

def sweepBall(robot):
    if robot.yPos > 65:
        robot.simSetVel(0,robot.rMax)
    else:
        robot.simSetVel(0,-robot.rMax)