from numpy import pi,cos,sin,arctan2
from execution import univec

#% Basic Actions
def stop(robot):
    robot.simSetVel(0,0)

def sweepBall(robot,leftSide=True):
    if leftSide:
        w=-0.5*robot.vMax*robot.R/robot.L
    else:
        w=0.5*robot.vMax*robot.R/robot.L
    if robot.yPos > 65:
        robot.simSetVel(0,w)
    else:
        robot.simSetVel(0,-w)

def positionToSweep(robot,ball,leftSide=True):
    if leftSide:
        robot.target.update(ball.xPos,ball.yPos,0)
        v,w=univec(robot,robot.target,None,False)
        robot.simSetVel(v,w)
    else:
        robot.target.update(ball.xPos,ball.yPos,pi)
        v,w=univec(robot,robot.target,None,False)
        robot.simSetVel(v,w)

def avoidBound(robot):
    #% Verify if the dot product between the robot and the point (135,65) is positive
    #% It means the angle resides in ]-pi/2,pi/2[
    dotProd=(cos(robot.theta))*(135-robot.xPos)+(sin(robot.theta))*(65-robot.yPos)
    if dotProd >= 0:
        arrivalTheta=arctan2(65-robot.yPos,135-robot.xPos)
        robot.target.update(135,65,arrivalTheta)
        v,w=univec(robot,robot.target,None,False)
        robot.simSetVel(v,w)
    else:
        arrivalTheta=arctan2(65-robot.yPos,15-robot.xPos)
        robot.target.update(15,65,arrivalTheta)
        v,w=univec(robot,robot.target,None,False)
        robot.simSetVel(v,w)

def holdPosition(robot,xg,yg,desTheta):
    robot.target.update(xg,yg,desTheta)
    v,w=univec(robot,robot.target,None,False,8,2,True)
    robot.simSetVel(v,w)

#% Attacker Actions
def shoot(robot,ball,leftSide=True):
    if leftSide:
        arrivalTheta=arctan2(65-ball.yPos,150-ball.xPos) #? Angle between the ball and point (150,65)
        robot.target.update(ball.xPos,ball.yPos,arrivalTheta)
        v,w=univec(robot,robot.target,None,False,16,2)
        robot.simSetVel(v,w)
    else:
        arrivalTheta=arctan2(65-ball.yPos,-ball.xPos) #? Angle between the ball and point (0,65)
        robot.target.update(ball.xPos,ball.yPos,arrivalTheta)
        v,w=univec(robot,robot.target,None,False,16,2)
        robot.simSetVel(v,w)