from mindstorms import MSHub, Motor, MotorPair, ColorSensor, DistanceSensor, App, ForceSensor
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
    def moveFree(w,x,y,z):
        MotorPair("F","B").move(w,x,y,z)
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
    def goTo(self,pos):
        x = pos[0]
        y = pos[1]
        self.moveX(x + (-1*self.position[0]))
        self.moveY(y + (-1*self.position[1]))
    def goTo2(self,pos):
        x = pos[0]
        y = pos[1]
        self.moveY(y + (-1*self.position[1]))
        self.moveX(x + (-1*self.position[0]))

class HubController:
    def __init__(self,motorE,motorD, position = [0,0,0]):
        self.robo = Robot(motorE,motorD, position)
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

def alinhar():
    return 1
def abrirgarra():
    return 1
def fechargarra():
    return 1
def subrirRampa():
    return 1
def main():
    animal_enfermo1 = [-41.5,30.3]
    animal_enfermo2 = [-6,15.5]
    animal_enfermo3 = [16.0,45.5]
    destino_enfermo1 = [102,-13]
    destino_enfermo2 = [88,-13]
    destino_enfermo3 = [95,-3.5]
    animal_encalhado1 = [-92.7,-32.0]
    animal_encalhado2 = [-52.0,-27.0]
    animal_encalhado3 = [-7.4,-43.2]
    destino_encalhado1 = [-16,52.5]
    destino_encalhado2 = [10.5, 52.5]
    destino_encalhado3 = [37.2,52.5]
    petroleo = [-94.0,45.9]
    frenterampa = [57,11]
    robo = Robot('F', 'B')
    #control = HubController('F','B')
'''
    #alinhar()
    animais encalhados
    robo.goTo(0,0)
    abrirgarra()
    robo.goTo(animal_encalhado1)
    fechargarra()
    robo.goTo2(destino_encalhado1)
    abrirgarra()
    robo.goTo(0,0)
    robo.goTo(animal_encalhado2)
    fechargarra()
    robo.goTo2(destino_encalhado2)
    abrirgarra()
    robo.goTo(0,0)
    robo.goTo(animal_encalhado3)
    fechargarra()
    robo.goTo2(destino_encalhado3)
    abrirgarra()
    robo.goTo(0,0)
    #animais enfermos
    robo.goTo(animal_enfermo1)
    fechargarra()
    robo.goTo2(destino_enfermo1)
    abrirgarra()
    robo.goTo(0,0)
    robo.goTo(animal_enfermo2)
    fechargarra()
    robo.goTo2(destino_enfermo2)
    abrirgarra()
    robo.goTo(0,0)
    robo.goTo(animal_enfermo3)
    fechargarra()
    robo.goTo2(destino_enfermo3)
    abrirgarra()
    robo.goTo(0,0)
    #trocargarra
    robo.goTo(frenteRampa)
    subirRampa()
    robo.pointTo(135)
'''
main()
