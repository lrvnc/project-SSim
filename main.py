from simClasses import Ball,Robot,Target
from strategy import SoloFollowBall
import sim,simConst

clientID=sim.simxStart('127.0.0.1',20001,True,True,5000,1)
if clientID==-1:
    print('Server not found!')
    exit()
print('Server connected!')

game=SoloFollowBall()
game.simConnect(clientID)

for i in range(50):
    game.play()

sim.simxFinish(clientID)