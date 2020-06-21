from numpy import array,arctan2,cos,sin,pi,sqrt,matmul,exp
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

def gtgVecField(robot,target):
    n=2
    d=8
    r=array([[target.xPos+d*cos(target.theta)],[target.yPos+d*sin(target.theta)]])
    pgAng=arctan2(target.yPos-robot.yPos,target.xPos-robot.xPos)
    prAng=arctan2(r[1]-robot.yPos,r[0]-robot.xPos)
    print(prAng*180/pi)
    grAng=arctan2(r[1]-target.yPos,r[0]-target.xPos)
    phi=arctan2(sin(prAng-pgAng),cos(prAng-pgAng)) #? Trick to mantain phi between [-pi,pi]
    desTheta=pgAng-n*phi
    desTheta=arctan2(sin(desTheta),cos(desTheta)) #? Trick to mantain desTheta between [-pi,pi]
    return desTheta

def phi_h_CW(x,y,xg,yg):
    d_e=5.37 #? Constant learned from EP
    k_r=4.15 #? Constant learned from EP
    rho=sqrt((xg-x)**2+(yg-y)**2)
    theta=arctan2(y-yg,x-xg)
    if rho > d_e:
        phi=theta+pi/2*(2-(d_e+k_r/rho+k_r))
    else:
        phi=theta+pi/2*sqrt(rho/d_e)
    phi=arctan2(sin(phi),cos(phi))
    return phi

def phi_h_CCW(x,y,xg,yg):
    d_e=5.37 #? Constant learned from EP
    k_r=4.15 #? Constant learned from EP
    rho=sqrt((xg-x)**2+(yg-y)**2)
    theta=arctan2(y-yg,x-xg)
    if rho > d_e:
        phi=theta-pi/2*(2-(d_e+k_r/rho+k_r))
    else:
        phi=theta-pi/2*sqrt(rho/d_e)
    phi=arctan2(sin(phi),cos(phi))    
    return phi

def N_h(phi):
    return array([[cos(phi)],[sin(phi)]])

def hipVecField(robot,target):
    d_e=5.37 #? Constant learned from EP
    yl=robot.yPos+d_e
    yr=robot.yPos-d_e
    nCW=N_h(phi_h_CW(robot.xPos,robot.yPos+d_e,target.xPos,target.yPos))
    nCCW=N_h(phi_h_CCW(robot.xPos,robot.yPos-d_e,target.xPos,target.yPos))
    if (robot.yPos >= -d_e and robot.yPos < d_e):
        phi=(yl*nCCW+yr*nCW)/2*d_e
        phi=arctan2(phi[1],phi[0])
    elif (robot.yPos < -d_e):
        phi=phi_h_CW(robot.xPos,robot.yPos-d_e,target.xPos,target.yPos)
    else:
        phi=phi_h_CCW(robot.xPos,robot.yPos+d_e,target.xPos,target.yPos)
    return phi

def aoFieldVector(robot,obst):
    k_o=0.12                                                    #? Constant learned from EP
    sx=k_o*(obst.v*cos(obst.theta)-robot.v*cos(robot.theta))    #? Components of the shifting vector, where
    sy=k_o*(obst.v*sin(obst.theta)-robot.v*sin(robot.theta))    #?   s=k_o*(v_obst-v_robot)
    s=sqrt(sx**2+sy**2)                                         #? Shifiting vector length
    d=sqrt((obst.xPos-robot.xPos)**2+(obst.yPos-robot.yPos)**2) #? Distance between the robot and the obstacle
    if d >= s:
        px=obst.xPos+sx
        py=obst.yPos+sy
    else:
        px=obst.xPos+(d/s)*sx
        py=obst.yPos+(d/s)*sy
    phi=arctan2(robot.yPos-py,robot.xPos-px)
    return phi

def gaussianFunc(r):
    delta=4.57 #? Constant learned from EP
    return exp(-0.5*(r/delta)**2)

def univecField(robot,target,obst):
    d_min=3.48 #? Constant learned from EP
    d=sqrt((obst.xPos-robot.xPos)**2+(obst.yPos-robot.yPos)**2)
    if (d <= d_min):
        phi=aoFieldVector(robot,obst)
    else:
        phi=gaussianFunc(d-d_min)*aoFieldVector(robot,obst)
        phi+=(1-gaussianFunc(d-d_min))*hipVecField(robot,target)
    return phi