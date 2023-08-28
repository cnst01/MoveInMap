from mindstorms import MSHub, Motor, MotorPair, ColorSensor, DistanceSensor, App
from mindstorms.control import wait_for_seconds, wait_until, Timer
from mindstorms.operator import greater_than, greater_than_or_equal_to, less_than, less_than_or_equal_to, equal_to, not_equal_to
import math

class myMap:
    def __init__ (self,start_pos):
        self.start_pos = start_pos
        pos = [self.start_pos[0], self.start_pos[1], self.start_pos[2]]
        self.points = [pos]

    def addPoint(self,point):
        pos = [point[0], point[1], point[2]]
        if not pos in self.points:
            self.points += [pos]

class Robot:
    def __init__ (self,motorE,motorD, position = [0,0,0]):
        self.position = position
        self.map = myMap(self.position)
        self.motors = MotorPair(motorE, motorD)
        self.hub = MSHub()
        self.hub.motion_sensor.reset_yaw_angle()

    def pointTo(self, degrees, precision = 1):
        dir = self.position[2]
        if(dir < degrees):
            while not self.hub.motion_sensor.get_yaw_angle() in range(degrees - precision, degrees + precision):
                self.motors.start_tank(20,-20)
            self.motors.stop()
        else:
            while not self.hub.motion_sensor.get_yaw_angle() in range(degrees - precision, degrees + precision):
                self.motors.start_tank(-20,20)
            self.motors.stop()
        print('turning ' + str(degrees) + ' degrees')
        self.position[2] = self.hub.motion_sensor.get_yaw_angle()
        self.map.points[-1][2] = self.position[2]

    def moveX(self,q):
        if q:
            if q > 0:
                self.pointTo(90)
            else:
                self.pointTo(-90)
            self.motors.move(abs(q),'cm', 0, 50)
            print('moving ' + str(q) + ' on X')
            self.position[0] += q
            self.map.addPoint(self.position)
    
    def moveY(self,q):
        if q :
            if q > 0:
                self.pointTo(0)
            else:
                self.pointTo(178)
            self.motors.move(abs(q),'cm', 0, 50)
            print('moving ' + str(q) + ' on Y')
            self.position[1] += q
            self.map.addPoint(self.position)
    

    def goTo(self,x,y):
        self.moveX(x - self.position[0])
        self.moveY(y - self.position[1])




def main():
    robo = Robot('F', 'B')
    robo.goTo(20,30)
    print(robo.map.points)
    robo.goTo(0,-10)
    print(robo.map.points)
    robo.pointTo(90)

main()