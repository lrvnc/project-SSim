from numpy import array,arctan2,cos,sin,pi,sqrt,matmul
import sim,simConst

#? Operation modes for API
opmblock=sim.simx_opmode_blocking
opmstream=sim.simx_opmode_streaming
opmbuffer=sim.simx_opmode_buffer
opmoneshot=sim.simx_opmode_oneshot

def navigate(theta,uX,uY):
    u=array([[uX],
             [uY]])
    R=array(((cos(-theta),-sin(-theta)),(sin(-theta),cos(-theta)))).reshape(2,2)
    L=array([[1,0],
             [0,15]])
    v,w=L.dot(R).dot(u)
    return 10*v,w

def simSetVel(robot,v,w):
    vR=v+w
    vL=v-w
    resRM=sim.simxSetJointTargetVelocity(robot.clientID,robot.rightMotor,vR,opmoneshot)
    resLM=sim.simxSetJointTargetVelocity(robot.clientID,robot.leftMotor,vL,opmoneshot)