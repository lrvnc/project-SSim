from numpy import cos,sin,arctan2,sqrt,sign,pi,delete,append,array
from behaviours import Univector

#% Function to approximate phi_v
def approx(robot,target,avoidObst=True,obst=None,n=8,d=2):
    navigate=Univector() #? Defines the navigation algorithm
    dl=0.000001          #? Constant to approximate phi_v
    
    x=robot.xPos #? Saving (x,y) coordinates to calculate phi_v
    y=robot.yPos
    robot.xPos=robot.xPos+dl*cos(robot.theta) #? Incrementing robot (x,y) position
    robot.yPos=robot.yPos+dl*sin(robot.theta)
    
    if avoidObst:
        stpTheta=navigate.univecField_N(robot,target,obst,n,d) #? Computing a step Theta to determine phi_v
    else:
        stpTheta=navigate.nVecField(robot,target,n,d) #? Computing a step Theta to determine phi_v
    
    robot.xPos=x #? Returning original (x,y) coordinates
    robot.yPos=y

    return stpTheta

#% Function to control the robot with or without collision avoidance
def univecController(robot,target,avoidObst=True,obst=None,n=8,d=2,stopWhenArrive=False):
    navigate=Univector() #? Defines the navigation algorithm
    dl=0.000001          #? Constant to approximate phi_v
    k_w=1                #? Feedback constant (k_w=1 means no gain)
    k_p=1                #? Feedback constant (k_p=1 means no gain)

    #% Navigation: Go-to-Goal + Avoid Obstacle Vector Field
    if avoidObst:
        desTheta=navigate.univecField_N(robot,target,obst,n,d) #? Desired angle w/ gtg and ao vector field
    #% Navigation: Go-to-Goal Vector Field
    else:
        desTheta=navigate.nVecField(robot,target,n,d) #? Desired angle w/ gtg

    stpTheta=approx(robot,target,avoidObst,obst,n,d)
    phi_v=arctan2(sin(stpTheta-desTheta),cos(stpTheta-desTheta))/dl #? Trick to mantain phi_v between [-pi,pi]
    theta_e=arctan2(sin(desTheta-robot.theta),cos(desTheta-robot.theta)) #? Trick to mantain theta_e between [-pi,pi]
    v1=(2*robot.vMax-robot.L*k_w*sqrt(abs(theta_e)))/(2+robot.L*abs(phi_v))
    v2=(sqrt(k_w**2+4*robot.rMax*abs(phi_v))-k_w*sqrt(abs(theta_e)))/(2*abs(phi_v)+dl)

    if stopWhenArrive:
        v3=k_p*robot.dist(target)
    else:
        v3=robot.vMax

    v=min(v1,v2,v3)
    w=v*phi_v+k_w*sign(theta_e)*sqrt(abs(theta_e))

    #% Some code to store the past position, orientation and velocity
    robot.v=v
    robot.pastPose=delete(robot.pastPose,0,1) #? Deleting the first column
    robot.pastPose=append(robot.pastPose,array([[round(robot.xPos)],[round(robot.yPos)],[round(float(robot.theta))],[round(float(v))]]),1)

    return v,w