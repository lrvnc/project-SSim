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

class test:
    def __init__(self,xPos,yPos,theta):
        self.xPos=xPos
        self.yPos=yPos
        self.theta=theta

robot=test(0,0,0)
ball=test(1,1,pi/2)

def gtgVecField(robot,target):
    N=2
    D=8
    r=array([[target.xPos+D*cos(target.theta)],[target.yPos+D*sin(target.theta)]])
    pgAng=arctan2(target.yPos-robot.yPos,target.xPos-robot.xPos)
    prAng=arctan2(r[1]-robot.yPos,r[0]-robot.xPos)
    print(prAng*180/pi)
    grAng=arctan2(r[1]-target.yPos,r[0]-target.xPos)
    phi=arctan2(sin(prAng-pgAng),cos(prAng-pgAng)) #? Trick to mantain phi between [-pi,pi]
    desTheta=pgAng-N*phi
    desTheta=arctan2(sin(desTheta),cos(desTheta)) #? Trick to mantain desTheta between [-pi,pi]
    return desTheta

def dist(class1,class2): #? Euclidean distance in plane
    return sqrt((class2.xPos-class1.xPos)**2+(class2.yPos-class1.yPos)**2)

def phi_h_CW(robot,target):
    d_e=3.48 #? Constant learned from EP
    K_r=4.15 #? Constant learned from EP
    rho=dist(robot,target)
    theta=arctan2(robot.yPos-target.yPos,robot.xPos-target.xPos)
    if rho > d_e:
        phi=theta+pi/2*(2-(d_e+K_r/rho+K_r))
    else:
        phi=theta+pi/2*sqrt(rho/d_e)
    phi=arctan2(sin(phi),cos(phi))
    return phi

def phi_h_CCW(robot,target):
    d_e=3.48 #? Constant learned from EP
    K_r=4.15 #? Constant learned from EP
    rho=dist(robot,target)
    theta=arctan2(robot.yPos-target.yPos,robot.xPos-target.xPos)
    if rho > d_e:
        phi=theta-pi/2*(2-(d_e+K_r/rho+K_r))
    else:
        phi=theta-pi/2*sqrt(rho/d_e)
    phi=arctan2(sin(phi),cos(phi))    
    return phi

def N_h(phi):
    return array([[cos(phi)],[sin(phi)]])

def hipVecField(robot,target):
    d_e=3.48 #? Constant learned from EP
    K_r=4.15 #? Constant learned from EP
    yl=robot.yPos+d_e
    yr=robot.yPos-d_e

robot=test(0,0,0)
ball=test(2,0,0)