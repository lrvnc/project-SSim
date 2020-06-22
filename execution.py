from numpy import cos,sin,arctan2,sqrt,sign
from behaviours import Navigation

class Controllers:
    def __init__(self):
        self.navigate=Navigation()
        self.dl=0.000001 #? Constant to approximate phi_v
        self.k_w=5       #? Feedback constant
    
    #% This function compute the output (v,w) of the controllers below
    def output_ctrl(self,phi):
        pass

    #% This controller will use only the behaviour go-to-goal
    def gtg_controller(self,robot,target):
        desTheta=self.navigate.hipVecField(robot,target) #? Desired angle w/ gtg vector field
        x=robot.xPos #? Saving (x,y) coordinates for calculate phi_v
        y=robot.yPos
        robot.xPos=robot.xPos+self.dl*cos(robot.theta)
        robot.yPos=robot.yPos+self.dl*sin(robot.theta)
        stpTheta=self.navigate.hipVecField(robot,target) #? Computing a step Theta to determine phi_v
        robot.xPos=x #? Returning original (x,y) coordinates
        robot.yPos=y
        phi_v=arctan2(sin(stpTheta-desTheta),cos(stpTheta-desTheta))/self.dl #? Trick to mantain phi_v between [-pi,pi]
        theta_e=arctan2(sin(desTheta-robot.theta),cos(desTheta-robot.theta)) #? Trick to mantain theta_e between [-pi,pi]
        v1=(2*robot.vMax-robot.L*self.k_w*sqrt(abs(theta_e)))/(2+robot.L*abs(phi_v))
        v2=(sqrt(self.k_w**2+4*robot.rMax*abs(phi_v))-self.k_w*sqrt(abs(theta_e)))/(2*abs(phi_v))
        v=min(v1,v2,robot.vMax)
        w=v*phi_v+self.k_w*sign(theta_e)*sqrt(abs(theta_e))
        return v,w

    #% This controller will use only the behaviour avoid-obstacle
    def ao_controller(self,robot,obst):
        pass

    #% This controller will use the mixed behaviour go-to-goal and avoid-obstacle
    def mixed_controller(self,robot,target,obst):
        pass