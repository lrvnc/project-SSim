from numpy import cos,sin,arctan2,sqrt,sign
from behaviours import Univector

#% Function to approximate phi_v
def approx(robot,target,obst=None,avoid=True):
    navigate=Univector() #? Defines the navigation algorithm
    dl=0.000001          #? Constant to approximate phi_v

    if avoid:
        x=robot.xPos #? Saving (x,y) coordinates for calculate phi_v
        y=robot.yPos
        robot.xPos=robot.xPos+dl*cos(robot.theta)
        robot.yPos=robot.yPos+dl*sin(robot.theta)
        stpTheta=navigate.univecField_N(robot,target,obst) #? Computing a step Theta to determine phi_v
        robot.xPos=x #? Returning original (x,y) coordinates
        robot.yPos=y
        return stpTheta

    else:
        x=robot.xPos #? Saving (x,y) coordinates for calculate phi_v
        y=robot.yPos
        robot.xPos=robot.xPos+dl*cos(robot.theta)
        robot.yPos=robot.yPos+dl*sin(robot.theta)
        stpTheta=navigate.nVecField(robot,target) #? Computing a step Theta to determine phi_v
        robot.xPos=x #? Returning original (x,y) coordinates
        robot.yPos=y
        return stpTheta

#% Function to control the robot with or without collision avoidance
def control(robot,target,obst=None,avoid=True):
    navigate=Univector() #? Defines the navigation algorithm
    dl=0.000001          #? Constant to approximate phi_v
    k_w=1                #? Feedback constant

    #% Navigation: Go-to-Goal + Avoid Obstacle Vector Field
    if avoid:
        desTheta=navigate.univecField_N(robot,target,obst) #? Desired angle w/ gtg and ao vector field
        stpTheta=approx(robot,target,obst,avoid)
        phi_v=arctan2(sin(stpTheta-desTheta),cos(stpTheta-desTheta))/dl #? Trick to mantain phi_v between [-pi,pi]
        theta_e=arctan2(sin(desTheta-robot.theta),cos(desTheta-robot.theta)) #? Trick to mantain theta_e between [-pi,pi]
        v1=(2*robot.vMax-robot.L*k_w*sqrt(abs(theta_e)))/(2+robot.L*abs(phi_v))
        v2=(sqrt(k_w**2+4*robot.rMax*abs(phi_v))-k_w*sqrt(abs(theta_e)))/(2*abs(phi_v)+dl)
        v=min(v1,v2,robot.vMax)
        w=v*phi_v+k_w*sign(theta_e)*sqrt(abs(theta_e))
        return v,w

    
    #% Navigation: Go-to-Goal Vector Field
    else:
        desTheta=navigate.nVecField(robot,target) #? Desired angle w/ gtg
        stpTheta=approx(robot,target,obst,avoid)
        phi_v=arctan2(sin(stpTheta-desTheta),cos(stpTheta-desTheta))/dl #? Trick to mantain phi_v between [-pi,pi]
        theta_e=arctan2(sin(desTheta-robot.theta),cos(desTheta-robot.theta)) #? Trick to mantain theta_e between [-pi,pi]
        v1=(2*robot.vMax-robot.L*k_w*sqrt(abs(theta_e)))/(2+robot.L*abs(phi_v))
        v2=(sqrt(k_w**2+4*robot.rMax*abs(phi_v))-k_w*sqrt(abs(theta_e)))/(2*abs(phi_v)+dl)
        v=min(v1,v2,robot.vMax)
        w=v*phi_v+k_w*sign(theta_e)*sqrt(abs(theta_e))
        return v,w