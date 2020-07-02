from simClasses import Ball,Robot,Target
from strategy import StrategyTesting,DrawRedDragons
import sim,simConst

clientID=sim.simxStart('127.0.0.1',20001,True,True,5000,1)
if clientID==-1:
    print('Server not found!')
    exit()
print('Server connected!')

game=StrategyTesting()
game.simConnect(clientID)

while True:
    game.play()

sim.simxFinish(clientID)