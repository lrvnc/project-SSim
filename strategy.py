from simClasses import Ball,Robot,Target
from execution import Controllers

class SoloFollowBall:
    def __init__(self):
        self.ball=Ball()
        self.redRob=Robot()
        self.target=Target()
        self.ctrl=Controllers()
    
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
        self.target.setTarget(self.ball.xPos,self.ball.yPos,0)
        v,w=self.ctrl.gtg_controller(self.redRob,self.target)
        self.target.showInfo()