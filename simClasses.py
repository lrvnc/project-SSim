from numpy import arctan2,pi,sqrt

class Ball:
    def __init__(self,pos):
        self.xPos=pos[0]
        self.yPos=pos[1]

    def setPos(self,x_new,y_new):
        self.xPos=x_new
        self.yPos=y_new

    def showInfo(self):
        return print('xPos:{} | yPos:{}'.format(self.xPos,self.yPos))

class Robot:
    def __init__(self,pos,idMarker,teamMarker,leftMotor,rightMotor):
        self.xPos=pos[0]
        self.yPos=pos[1]
        self.theta=arctan2(idMarker[1]-teamMarker[1],idMarker[0]-teamMarker[0])-pi/4
        self.rMotor=rightMotor
        self.lMotor=leftMotor

    def setPose(self,x_new,y_new,idMarker_new,teamMarker_new):
        self.xPos=x_new
        self.yPos=y_new
        self.theta=arctan2(idMarker_new[1]-teamMarker_new[1],idMarker_new[0]-teamMarker_new[0])-pi/4