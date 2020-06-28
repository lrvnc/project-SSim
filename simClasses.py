from numpy import arctan2,pi,sqrt,cos,sin,array,matmul,amin,where,zeros,delete,append
import sim,simConst

#? Operation modes for API
opmblock=sim.simx_opmode_blocking
opmstream=sim.simx_opmode_streaming
opmbuffer=sim.simx_opmode_buffer
opmoneshot=sim.simx_opmode_oneshot

#! Units: cm, rad, s

#% Class to set the targets of each robot in game
class Target:
    def __init__(self):
        self.xPos=0         #? Desired x position
        self.yPos=0         #? Desired y position
        self.theta=0        #? Orientation at the desired point (x,y)

    #% Setter
    def update(self,x,y,theta):
        self.xPos=x
        self.yPos=y
        self.theta=theta

    #% This method print a little log on console
    def showInfo(self):
        print('xPos: {:.2f} | yPos: {:.2f} | theta: {:.2f}'.format(self.xPos,self.yPos,float(self.theta)))

#% Class to set the obstacle of each robot
class Obstacle:
    def __init__(self):
        self.xPos=0         #? Obstacle x position
        self.yPos=0         #? Obstacle y position
        self.v=0            #? Obstacle velocity (cm/s)
        self.theta=0        #? Obstacle orientation

    #% Setter
    def setObst(self,x,y,v,theta):
        self.xPos=x
        self.yPos=y
        self.v=v
        self.theta=theta

    #% This method verify which is the closest obstacle and sets it as the current obstacle to avoid
    def update(self,robot,friend1,friend2,enemy1,enemy2,enemy3):
        d=array([[robot.dist(friend1)],
                 [robot.dist(friend2)],
                 [robot.dist(enemy1)],
                 [robot.dist(enemy2)],
                 [robot.dist(enemy3)]])
        index=where(d==amin(d))
        if index[0][0]==0:
            self.setObst(friend1.xPos,friend1.yPos,0,0)
        elif index[0][0]==1:
            self.setObst(friend2.xPos,friend2.yPos,0,0)
        elif index[0][0]==2:
            self.setObst(enemy1.xPos,enemy1.yPos,0,0)
        elif index[0][0]==3:
            self.setObst(enemy2.xPos,enemy2.yPos,0,0)
        else:
            self.setObst(enemy3.xPos,enemy3.yPos,0,0)

    #% This method print a little log on console
    def showInfo(self):
        print('xPos: {:.2f} | yPos: {:.2f} | theta: {:.2f} | velocity: {:.2f}'.format(self.xPos,self.yPos,float(self.theta),self.v))

#% Class to create the ball in game
class Ball:
    def __init__(self):
        self.simStream=False
        self.xPos=0
        self.yPos=0

    #% This method connects the ball with CoppeliaSim
    def simConnect(self,clientID,center):
        self.clientID=clientID
        self.resC,self.center=sim.simxGetObjectHandle(self.clientID,center,opmblock) #? Receiving the ball in the simulation

    #% This method verify the connection between the ball and the simulation
    def simCheckConnection(self):
        if (self.resC!=0):
            return False
        else:
            return True

    #% This method gets the position of the ball in CoppeliaSim
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

    #% This method print a little log on console
    def showInfo(self):
        print('xPos: {:.2f} | yPos: {:.2f}'.format(self.xPos,self.yPos))

#% Class to create the robots in game
class Robot:
    def __init__(self):
        self.simStream=False
        self.xPos=0                         #? X position
        self.yPos=0                         #? Y position
        self.theta=0                        #? Orientation
        self.rightMotor=0                   #? Right motor handle
        self.leftMotor=0                    #? Left motor handle
        self.v=0                            #? Velocity (cm/s)
        self.vMax=30                        #! Robot max velocity (cm/s)
        self.rMax=3*self.vMax               #! Robot max rotation velocity (rad*cm/s)
        self.L=8                            #? Base length of the robot (cm)
        self.R=3.4                          #? Wheel radius (cm)
        self.obst=Obstacle()                #? Defines the robot obstacle
        self.target=Target()                #? Defines the robot target
        self.pastPose=zeros(9).reshape(3,3) #? Stores the last 3 positions (x,y) and orientation

    #% This method calculate the distance between the robot and an object
    def dist(self,obj):
        return sqrt((self.xPos-obj.xPos)**2+(self.yPos-obj.yPos)**2)

    #% This method connects the robot with CoppeliaSim
    def simConnect(self,clientID,center,teamMarker,idMarker,leftMotor,rightMotor):
        self.clientID=clientID
        self.resC,self.center=sim.simxGetObjectHandle(self.clientID,center,opmblock)           #? Receiving robot parts in the simulation
        self.resTM,self.teamMarker=sim.simxGetObjectHandle(self.clientID,teamMarker,opmblock)
        self.resIDM,self.IDMarker=sim.simxGetObjectHandle(self.clientID,idMarker,opmblock)
        self.resLM,self.leftMotor=sim.simxGetObjectHandle(self.clientID,leftMotor,opmblock)
        self.resRM,self.rightMotor=sim.simxGetObjectHandle(self.clientID,rightMotor,opmblock)

    #% This method verify the connection between the robot and the simulation
    def simCheckConnection(self):
        if (self.resC!=0 or self.resTM!=0 or self.resIDM!=0 or self.resLM!=0 or self.resRM!=0):
            return False
        else:
            return True

    #% This method gets both position and orientation of the robot in CoppeliaSim
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
            rotM=array(((cos(-pi/4),-sin(-pi/4)),(sin(-pi/4),cos(-pi/4))))
            mrkVec=array(((self.idMarkerPos[0]-self.teamMarkerPos[0]),(self.idMarkerPos[1]-self.teamMarkerPos[1]))).reshape(2,1)
            headVec=100*matmul(rotM,mrkVec)
            self.theta=arctan2(headVec[1],headVec[0])
            #? Code to store the past Pose:
            self.pastPose=delete(self.pastPose,0,1) #? Deleting the first column
            self.pastPose=append(self.pastPose,array([[round(self.xPos)],[round(self.yPos)],[round(float(self.theta))]]),1) #? Updating the last Pose
            

    #% This method sets the velocity of the robot
    def simSetVel(self,v,w):
        self.v=v
        self.resRM=sim.simxSetJointTargetVelocity(self.clientID,self.rightMotor,(v+0.5*self.L*w)/self.R,opmoneshot)
        self.resLM=sim.simxSetJointTargetVelocity(self.clientID,self.leftMotor,(v-0.5*self.L*w)/self.R,opmoneshot)

    #% This method print a little log on console
    def showInfo(self):
        print('xPos: {:.2f} | yPos: {:.2f} | theta: {:.2f} | velocity: {:.2f}'.format(self.xPos,self.yPos,float(self.theta),float(self.v)))