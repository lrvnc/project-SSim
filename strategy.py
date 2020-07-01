from numpy import pi,array_equal
from simClasses import Ball,Robot,Target,Obstacle
import action
from time import sleep

class StrategyTesting:
    def __init__(self):
        self.ball=Ball()
        self.redRob=Robot()
        self.greenRob=Robot()
        self.pinkRob=Robot()

    def simConnect(self,clientID):
        self.clientID=clientID
        self.ball.simConnect(self.clientID,'ball')
        self.redRob.simConnect(self.clientID,'soccerRob_pos','soccerRob_teamMarker','soccerRob_IDMarker','leftMotor','rightMotor')
        self.greenRob.simConnect(self.clientID,'soccerRob_pos#0','soccerRob_teamMarker#0','soccerRob_IDMarker#0','leftMotor#0','rightMotor#0')
        self.pinkRob.simConnect(self.clientID,'soccerRob_pos#1','soccerRob_teamMarker#1','soccerRob_IDMarker#1','leftMotor#1','rightMotor#1')

        if self.redRob.simCheckConnection():
            print('RedRob ready to play!')
        else:
            print('RedRob not found...')
        if self.greenRob.simCheckConnection():
            print('GreenRob ready to play!')
        else:
            print('GreenRob not found...')
        if self.pinkRob.simCheckConnection():
            print('pinkRob ready to play!')
        else:
            print('pinkRob not found...')
        if self.ball.simCheckConnection():
            print('Ball ready to play!\n')
        else:
            print('Ball not found...\n')
        self.redRob.simGetPose('infLeft_cornor')
        self.greenRob.simGetPose('infLeft_cornor')
        self.pinkRob.simGetPose('infLeft_cornor')
        self.ball.simGetPose('infLeft_cornor')

        i=0
        while self.redRob.xPos == 0 or self.greenRob.xPos == 0 or self.pinkRob.xPos == 0 or self.ball.xPos == 0 :
            self.redRob.simGetPose('infLeft_cornor')
            self.greenRob.simGetPose('infLeft_cornor')
            self.pinkRob.simGetPose('infLeft_cornor')
            self.ball.simGetPose('infLeft_cornor')
            i+=1
        print(i,'tentativas até pegar as posições\n')


    def play(self):
        self.redRob.simGetPose('infLeft_cornor')
        self.greenRob.simGetPose('infLeft_cornor')
        self.pinkRob.simGetPose('infLeft_cornor')
        self.ball.simGetPose('infLeft_cornor')
        action.holdPosition(self.redRob,60,75,0,self.pinkRob,self.greenRob)
        action.screenOutBall(self.pinkRob,self.ball,True)
        if self.greenRob.yPos < 10:
            action.avoidBound(self.greenRob)
        else:
            action.shoot(self.greenRob,self.ball,False,self.redRob,self.pinkRob)