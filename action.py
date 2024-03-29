from numpy import pi,cos,sin,arctan2,sqrt
from execution import univecController

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

def positionToSweep(robot,ball,leftSide=True,friend1=None,friend2=None):
    if leftSide:
        robot.target.update(ball.xPos,ball.yPos,0)
    else:
        robot.target.update(ball.xPos,ball.yPos,pi)

    if friend1 is None and friend2 is None: #? No friends to avoid
        v,w=univecController(robot,robot.target,avoidObst=False)
    else: #? Both friends to avoid
        robot.obst.update(robot,friend1,friend2)
        v,w=univecController(robot,robot.target,True,robot.obst)
    
    robot.simSetVel(v,w)

def avoidBound(robot,friend1=None,friend2=None):
    #% Verify if the dot product between the robot and the point (135,65) is positive
    #% It means the angle resides in ]-pi/2,pi/2[
    dotProd=(cos(robot.theta))*(135-robot.xPos)+(sin(robot.theta))*(65-robot.yPos)

    if dotProd >= 0:
        arrivalTheta=arctan2(65-robot.yPos,135-robot.xPos)
        robot.target.update(135,65,arrivalTheta)
    else:
        arrivalTheta=arctan2(65-robot.yPos,15-robot.xPos)
        robot.target.update(15,65,arrivalTheta)
    
    if friend1 is None and friend2 is None: #? No friends to avoid
        v,w=univecController(robot,robot.target,avoidObst=False)
    else: #? Both friends to avoid
        robot.obst.update(robot,friend1,friend2)
        v,w=univecController(robot,robot.target,True,robot.obst)

    robot.simSetVel(v,w)

def holdPosition(robot,xg,yg,desTheta,friend1=None,friend2=None):
    robot.target.update(xg,yg,desTheta)

    if friend1 is None and friend2 is None: #? No friends to avoid
        v,w=univecController(robot,robot.target,avoidObst=False,stopWhenArrive=True)
    else: #? Both friends to avoid
        robot.obst.update(robot,friend1,friend2)
        v,w=univecController(robot,robot.target,True,robot.obst,stopWhenArrive=True)

    robot.simSetVel(v,w)

#% Attacker Actions
def shoot(robot,ball,leftSide=True,friend1=None,friend2=None):
    if leftSide:
        arrivalTheta=arctan2(65-ball.yPos,150-ball.xPos) #? Angle between the ball and point (150,65)
    else:
        arrivalTheta=arctan2(65-ball.yPos,-ball.xPos) #? Angle between the ball and point (0,65)
    robot.target.update(ball.xPos,ball.yPos,arrivalTheta)

    if friend1 is None and friend2 is None: #? No friends to avoid
        v,w=univecController(robot,robot.target,avoidObst=False,n=16)
    else: #? Both friends to avoid
        robot.obst.update(robot,friend1,friend2)
        v,w=univecController(robot,robot.target,True,robot.obst,16)

    robot.simSetVel(v,w)

#% Defender Actions
def pushBall(robot,ball,friend1=None,friend2=None):
    dSup=sqrt((75-ball.xPos)**2+(130-ball.yPos)**2) #? Distance between the ball and point (75,130)
    dInf=sqrt((75-ball.xPos)**2+(0-ball.yPos)**2)   #? Distance between the ball and point (75,0)

    if dSup<=dInf:
        arrivalTheta=arctan2(130-ball.yPos,75-ball.xPos) #? Angle between the ball and point (75,130)
    else:
        arrivalTheta=arctan2(-ball.yPos,75-ball.xPos) #? Angle between the ball and point (75,0)
    robot.target.update(ball.xPos,ball.yPos,arrivalTheta)
    
    if friend1 is None and friend2 is None: #? No friends to avoid
        v,w=univecController(robot,robot.target,avoidObst=False)
    else: #? Both friends to avoid
        robot.obst.update(robot,friend1,friend2)
        v,w=univecController(robot,robot.target,True,robot.obst)

    robot.simSetVel(v,w)

#TODO #2 Need more speed to reach the ball faster than our enemy
def screenOutBall(robot,ball,leftSide=True,friend1=None,friend2=None):
    if leftSide:
        if robot.yPos <= ball.yPos:
            arrivalTheta=pi/2
        else:
            arrivalTheta=-pi/2
        robot.target.update(30,ball.yPos,arrivalTheta)
    else:
        if robot.yPos <= ball.yPos:
            arrivalTheta=pi/2
        else:
            arrivalTheta=-pi/2
        robot.target.update(120,ball.yPos,arrivalTheta)

    if friend1 is None and friend2 is None: #? No friends to avoid
        v,w=univecController(robot,robot.target,avoidObst=False)
    else: #? Both friends to avoid
        robot.obst.update(robot,friend1,friend2)
        v,w=univecController(robot,robot.target,True,robot.obst)

    robot.simSetVel(v,w)  

#% Goalkeeper Actions
#TODO #1 More effective way to predict the ball position
def blockBall(robot,ball,leftSide=True):
    ballVec=(ball.pastPose[:,1]-ball.pastPose[:,0]).reshape(2,1) #? Building a vector between current and past position of the ball
    if leftSide: 
        alpha=(9-ball.xPos)/(ballVec[0]+0.000000001)
        desY=ball.yPos+alpha*ballVec[1]
        if desY <= 82 and desY >= 48: #? If the projection of the ball is inside of our goal, we manage the goalkeeper to the
            if robot.yPos <= desY:   #? point (9,y_projected)
                arrivalTheta=pi/2
            else:
                arrivalTheta=-pi/2
            robot.target.update(9,float(desY),arrivalTheta)
        else:                       #? Else we manage the goalkeeper to the center of the goal, at point (9,65)
            if robot.yPos <= 65:
                arrivalTheta=pi/2
            else:
                arrivalTheta=-pi/2
            robot.target.update(9,65,arrivalTheta)
        v,w=univecController(robot,robot.target,None,False,stopWhenArrive=True)
        robot.simSetVel(v,w)
    else:
        alpha=(141-ball.xPos)/(ballVec[0]+0.000000001)
        desY=ball.yPos+alpha*ballVec[1]
        if desY <= 82 and desY >= 48: #? If the projection of the ball is inside of our goal, we manage the goalkeeper to the
            if robot.yPos <= desY:   #? point (141,y_projected)
                arrivalTheta=pi/2
            else:
                arrivalTheta=-pi/2
            robot.target.update(141,float(desY),arrivalTheta)
        else:                       #? Else we manage the goalkeeper to the center of the goal, at point (141,65)
            if robot.yPos <= 65:
                arrivalTheta=pi/2
            else:
                arrivalTheta=-pi/2
            robot.target.update(141,65,arrivalTheta)
        v,w=univecController(robot,robot.target,None,False,stopWhenArrive=True)
        robot.simSetVel(v,w)