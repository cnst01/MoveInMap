import MSHub, Motor, MotorPair, ColorSensor, DistanceSensor, App, ForceSensor
import wait_for_seconds, wait_until, Timer
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
    def __init__ (self,motorE,motorD,force_sensor_port = None,position = [0,0,0]):
        self.position = position
        self.map = myMap(self.position)
        self.motors = MotorPair(motorE, motorD)
        self.hub = MSHub()
        self.hub.motion_sensor.reset_yaw_angle()
        if force_sensor_port:
            self.force_sensor = ForceSensor(force_sensor_port)

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
        x = x + (-1*self.position[0])
        y = y + (-1*self.position[1])
        dist = math.sqrt(x**2 + y**2)
        if x != 0 and y > 0:
            self.pointTo(math.degrees(math.atan(x/y)))
        elif x > 0 and y < 0:
            self.pointTo(90 + abs(math.degrees(math.atan(y/x))))
        elif x < 0 and y < 0:
            self.pointTo(-90 - abs(math.degrees(math.atan(y/x))))
        elif x == 0 and y != 0:
            self.moveY(y)
            return 1
        elif x != 0 and y == 0:
            self.moveX(x)
            return 1
        else:
            return 0
        self.motors.move(dist,'cm', 0, 50)
        self.position[0] += x
        self.position[1] += y
        self.map.addPoint(self.position)
        return 1

class HubController:
    def __init__(self,motorE,motorD,force_sensor_port,position = [0,0,0]):
        self.robo = Robot(motorE,motorD,force_sensor_port,position)
        while True:
            self.do(self.beginScreen())
            print(self.robo.map.points)

    def beginScreen(self):
        screen_names = ['GOTO', 'X', 'Y']
        screen_number = 0
        while not self.robo.force_sensor.is_pressed():
            if self.robo.hub.right_button.was_pressed():
                self.robo.hub.right_button.wait_until_released()
                if screen_number < 2:
                    screen_number += 1
                else:
                    screen_number = 0
            self.robo.hub.light_matrix.write(screen_names[screen_number])        
        return screen_names[screen_number]

    def do(self,screen):
        self.robo.force_sensor.wait_until_released()
        if screen == 'GOTO':
            point = [0,0]
            for i in range(2):
                while not self.robo.force_sensor.is_pressed():
                    if self.robo.hub.right_button.was_pressed():
                        self.robo.hub.right_button.wait_until_released()
                        point[i] += 1
                    elif self.robo.hub.left_button.was_pressed():
                        self.robo.hub.left_button.wait_until_released()
                        point[i] -= 1
                    self.robo.hub.light_matrix.write(point[i])
                self.robo.force_sensor.wait_until_released()
            self.robo.goTo(point[0],point[1])
            return 1
        else:
            value = 0
            while not self.robo.force_sensor.is_pressed():
                if self.robo.hub.right_button.was_pressed():
                    self.robo.hub.right_button.wait_until_released()
                    value += 1
                elif self.robo.hub.left_button.was_pressed():
                    self.robo.hub.left_button.wait_until_released()
                    value -= 1
                self.robo.hub.light_matrix.write(value)
            self.robo.force_sensor.wait_until_released()
            if screen == 'X':
                self.robo.moveX(value)
            else:
                self.robo.moveY(value)
            return 1

'''
Orientações de uso:

- Sempre crie o obj Robot passando por parâmetro:
    - idx motor esquerdo
    - idx motor direito
    - posição inicial do robô
        - sendo x,y,direção
            - direção deve ser iniciada como 0,
            paralelo ao eixo Y

- A função GoTo() executa o movimento primeiro no eixo X
para depois fazer o eixo Y

- Os objs do tipo Map permitem adição de vetores através do método
addPoint(), lembrando que possuem direção

'''

def main():
    robo = Robot('E', 'F')
    control = HubController('E','F','A')
