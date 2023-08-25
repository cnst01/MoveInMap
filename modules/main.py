import keyboard
import time

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
    def __init__ (self, position = [0,0,90]):
        self.position = position
        self.map = myMap(self.position)    

    def moveX(self,q):
        if q : 
            print('moving ' + str(q) + ' on X')
            self.position[0] += q
            self.map.addPoint(self.position)
    def moveY(self,q):
        if q : 
            print('moving ' + str(q) + ' on Y')
            self.position[1] += q
            self.map.addPoint(self.position)
    def Turn(self, dir):
        if dir : 
            print('turning ' + str(dir) + ' degrees')
            self.position[2] += dir
            self.map.addPoint(self.position)
    
    def goTO(self,x,y,dir = 90):
        self.moveX(x - self.position[0])
        self.moveY(y - self.position[1])
        self.Turn(dir - self.position[2])


def ManualMovement(robot):
    if keyboard.is_pressed('w'):
        robot.moveY(10)
        time.sleep(0.5)
        return 1
    if keyboard.is_pressed('a'):
        robot.moveX(-10)
        time.sleep(0.5)
        return 1
    if keyboard.is_pressed('s'):
        robot.moveY(-10)
        time.sleep(0.5)
        return 1
    if keyboard.is_pressed('d'):
        robot.moveX(10)
        time.sleep(0.5)
        return 1
    return 0 


if __name__ == '__main__':
    robo = Robot()
    while not keyboard.is_pressed('space'):
        if ManualMovement(robo):
            print(robo.map.points)