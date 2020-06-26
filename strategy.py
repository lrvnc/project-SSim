from numpy import pi
from simClasses import Ball,Robot,Target,Obstacle
from execution import control

class SoloFollowBall:
    def __init__(self):
        self.ball=Ball()
        self.redRob=Robot()

    def simConnect(self,clientID):
        self.clientID=clientID
        self.ball.simConnect(self.clientID,'ball')
        self.redRob.simConnect(self.clientID,'soccerRob_pos','soccerRob_teamMarker','soccerRob_IDMarker','leftMotor','rightMotor')
        if self.redRob.simCheckConnection():
            print('Robot ready to play!')
        else:
            print('Robot not found...')
        if self.ball.simCheckConnection:
            print('Ball ready to play!')
        else:
            print('Ball not found...')

    def play(self):
        self.redRob.simGetPose('infLeft_cornor')
        self.ball.simGetPose('infLeft_cornor')
        self.redRob.target.update(self.ball.xPos,self.ball.yPos,-pi/2)
        v,w=self.ctrl.gtgN_controller(self.redRob,self.redRob.target)
        self.redRob.simSetVel(v,w)

class SoloStaticObstacles:
    def __init__(self):
        self.ball=Ball()
        self.redRob=Robot()
        self.cylinder1=Ball()
        self.cylinder2=Ball()
        self.cylinder3=Ball()
        self.cylinder4=Ball()
        self.cylinder5=Ball()

    def simConnect(self,clientID):
        self.clientID=clientID
        self.ball.simConnect(self.clientID,'ball')
        self.cylinder1.simConnect(self.clientID,'Cylinder1')
        self.cylinder2.simConnect(self.clientID,'Cylinder2')
        self.cylinder3.simConnect(self.clientID,'Cylinder3')
        self.cylinder4.simConnect(self.clientID,'Cylinder4')
        self.cylinder5.simConnect(self.clientID,'Cylinder5')
        self.redRob.simConnect(self.clientID,'soccerRob_pos','soccerRob_teamMarker','soccerRob_IDMarker','leftMotor','rightMotor')
        if self.redRob.simCheckConnection():
            print('Robot ready to play!')
        else:
            print('Robot not found...')
        if self.ball.simCheckConnection:
            print('Ball ready to play!')
        else:
            print('Ball not found...')
        if self.cylinder1.simCheckConnection:
            print('Cyl1 ready to play!')
        else:
            print('Cyl1 not found...')
        if self.cylinder2.simCheckConnection:
            print('Cyl2 ready to play!')
        else:
            print('Cyl2 not found...')
        if self.cylinder3.simCheckConnection:
            print('Cyl3 ready to play!')
        else:
            print('Cyl3 not found...')
        if self.cylinder4.simCheckConnection:
            print('Cyl4 ready to play!')
        else:
            print('Cyl4 not found...')
        if self.cylinder5.simCheckConnection:
            print('Cyl5 ready to play!')
        else:
            print('Cyl5 not found...')

    def play(self):
        self.redRob.simGetPose('infLeft_cornor')
        self.ball.simGetPose('infLeft_cornor')
        self.cylinder1.simGetPose('infLeft_cornor')
        self.cylinder2.simGetPose('infLeft_cornor')
        self.cylinder3.simGetPose('infLeft_cornor')
        self.cylinder4.simGetPose('infLeft_cornor')
        self.cylinder5.simGetPose('infLeft_cornor')
        self.redRob.target.update(self.ball.xPos,self.ball.yPos,pi/2)
        self.redRob.obst.update(self.redRob,self.cylinder1,self.cylinder2,self.cylinder3,self.cylinder4,self.cylinder5)
        v,w=control(self.redRob,self.redRob.target,self.redRob.obst,True)
        self.redRob.simSetVel(v,w)
