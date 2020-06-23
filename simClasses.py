from numpy import arctan2,pi,sqrt,cos,sin,array,matmul
import sim,simConst

#? Operation modes for API
opmblock=sim.simx_opmode_blocking
opmstream=sim.simx_opmode_streaming
opmbuffer=sim.simx_opmode_buffer
opmoneshot=sim.simx_opmode_oneshot

#! Units: cm, rad, s
class Ball:
    def __init__(self):
        self.simStream=False
        self.xPos=0
        self.yPos=0
    
    def simConnect(self,clientID,center):
        self.clientID=clientID
        self.resC,self.center=sim.simxGetObjectHandle(self.clientID,center,opmblock) #? Receiving the ball in the simulation

    def simCheckConnection(self):
        if (self.resC!=0):
            return False
        else:
            return True

    def simGetPose(self,refPoint):
        if (not self.simStream):
            resRP,self.refPoint=sim.simxGetObjectHandle(self.clientID,refPoint,opmblock)    #? Reference point
            if resRP!=0:
                print('Error while setting the reference point!\nTurning off the simulation')
                sim.simxFinish(self.clientID)
                exit()
            self.resC,self.centerPos=sim.simxGetObjectPosition(self.clientID,self.center,self.refPoint,opmstream)
            self.simStream=True
        else:
            self.resC,self.centerPos=sim.simxGetObjectPosition(self.clientID,self.center,self.refPoint,opmbuffer)
            self.xPos=100*self.centerPos[0]
            self.yPos=100*self.centerPos[1]

    def showInfo(self):
        print('xPos: {:.2f} | yPos: {:.2f}'.format(self.xPos,self.yPos))

class Robot:
    def __init__(self):
        self.simStream=False
        self.xPos=0             #? X position
        self.yPos=0             #? Y position
        self.theta=0            #? Orientation
        self.rightMotor=0       #? Right motor handle
        self.leftMotor=0        #? Left motor handle
        self.v=0                #? Velocity (cm/s)
        self.vMax=30            #! Robot max velocity (cm/s)
        self.rMax=3*self.vMax   #! Robot max rotation velocity (rad*cm/s)
        self.L=8                #? Base length of the robot (cm)
        self.R=3.4              #? Wheel radius (cm)

    def simConnect(self,clientID,center,teamMarker,idMarker,leftMotor,rightMotor):
        self.clientID=clientID
        self.resC,self.center=sim.simxGetObjectHandle(self.clientID,center,opmblock)           #? Receiving robot parts in the simulation
        self.resTM,self.teamMarker=sim.simxGetObjectHandle(self.clientID,teamMarker,opmblock)
        self.resIDM,self.IDMarker=sim.simxGetObjectHandle(self.clientID,idMarker,opmblock)
        self.resLM,self.leftMotor=sim.simxGetObjectHandle(self.clientID,leftMotor,opmblock)
        self.resRM,self.rightMotor=sim.simxGetObjectHandle(self.clientID,rightMotor,opmblock) 

    def simCheckConnection(self):
        if (self.resC!=0 or self.resTM!=0 or self.resIDM!=0 or self.resLM!=0 or self.resRM!=0):
            return False
        else:
            return True

    def simGetPose(self,refPoint):
        if (not self.simStream):
            resRP,self.refPoint=sim.simxGetObjectHandle(self.clientID,refPoint,opmblock)    #? Reference point
            if resRP!=0:
                print('Error while setting the reference point!\nTurning off the simulation')
                sim.simxFinish(self.clientID)
                exit()
            self.resC,self.centerPos=sim.simxGetObjectPosition(self.clientID,self.center,self.refPoint,opmstream)
            self.resTM,self.teamMarkerPos=sim.simxGetObjectPosition(self.clientID,self.teamMarker,self.refPoint,opmstream)
            self.resIDM,self.idMarkerPos=sim.simxGetObjectPosition(self.clientID,self.IDMarker,self.refPoint,opmstream)
            self.simStream=True
        else:
            self.resC,self.centerPos=sim.simxGetObjectPosition(self.clientID,self.center,self.refPoint,opmbuffer)
            self.resTM,self.teamMarkerPos=sim.simxGetObjectPosition(self.clientID,self.teamMarker,self.refPoint,opmbuffer)
            self.resIDM,self.idMarkerPos=sim.simxGetObjectPosition(self.clientID,self.IDMarker,self.refPoint,opmbuffer)
            self.xPos=100*self.centerPos[0]
            self.yPos=100*self.centerPos[1]
            rotMatrix=array(((cos(-pi/4),-sin(-pi/4)),(sin(-pi/4),cos(-pi/4))))
            posVec=array(((self.idMarkerPos[0]-self.teamMarkerPos[0]),(self.idMarkerPos[1]-self.teamMarkerPos[1]))).reshape(2,1)
            rotVec=matmul(rotMatrix,posVec)
            self.theta=arctan2(rotVec[1],rotVec[0])
        
    def simSetVel(self,v,w):
        self.v=v
        self.resRM=sim.simxSetJointTargetVelocity(self.clientID,self.rightMotor,(v+0.5*self.L*w)/self.R,opmoneshot)
        self.resLM=sim.simxSetJointTargetVelocity(self.clientID,self.leftMotor,(v-0.5*self.L*w)/self.R,opmoneshot)
    
    def showInfo(self):
        print('xPos: {:.2f} | yPos: {:.2f} | theta: {:.2f} | velocity: {:.2f}'.format(self.xPos,self.yPos,float(self.theta),self.v))

class Target:
    def __init__(self):
        self.xPos=0         #? Desired x position
        self.yPos=0         #? Desired y position
        self.theta=0        #? Orientation at the desired point (x,y)

    def setTarget(self,x,y,theta):
        self.xPos=x
        self.yPos=y
        self.theta=theta

    def showInfo(self):
        print('xPos: {:.2f} | yPos: {:.2f} | theta: {:.2f}'.format(self.xPos,self.yPos,float(self.theta)))

class Obstacle:
    def __init__(self):
        self.xPos=0         #? Obstacle x position
        self.yPos=0         #? Obstacle y position
        self.v=0            #? Obstacle velocity (cm/s)
        self.theta=0        #? OBstacle orientation

    def setObst(self,x,y,v,theta):
        self.xPos=x
        self.yPos=y
        self.v=v
        self.theta=theta

    def showInfo(self):
        print('xPos: {:.2f} | yPos: {:.2f} | theta: {:.2f} | velocity: {:.2f}'.format(self.xPos,self.yPos,float(self.theta),self.v))