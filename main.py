import sim,simConst
from simClasses import Ball,Robot
from numpy import pi

clientID=sim.simxStart('127.0.0.1',20001,True,True,5000,1)
if clientID==-1:
    print('Server not found!')
    exit()
print('Server connected!')

ball=Ball()
attacker=Robot()

ball.simConnect(clientID,'ball')
attacker.simConnect(clientID,'soccerRob_pos','soccerRob_teamMarker','soccerRob_IDMarker','leftMotor','rightMotor')

ball.simStreamPose('infLeft_cornor')
attacker.simStreamPose('infLeft_cornor')

while True:
    ball.simGetPose()
    attacker.simGetPose()

sim.simxFinish(clientID)