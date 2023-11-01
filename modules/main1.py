from mindstorms import MSHub, Motor, MotorPair, ColorSensor, DistanceSensor, App, ForceSensor
from mindstorms.control import wait_for_seconds, wait_until, Timer
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

class Claw:
    def __init__(self, claw_port):
        self.motor = Motor(claw_port)
        self.motor.set_stop_action('hold')
    def Open(self):
        self.motor.run_for_rotations(0.35,-40)
    def Close(self):
        self.motor.run_for_rotations(0.3,15)

class Robot:
    def __init__ (self,motorE,motorD, claw_port = None, position = [0,0,0] ,force_sensor_port = None, speed = 30):
        self.speed = speed
        self.position = position
        self.map = myMap(self.position)
        self.claw = Claw(claw_port)
        self.motors = MotorPair(motorE, motorD)
        self.motors.set_stop_action('hold')
        self.hub = MSHub()
        self.hub.motion_sensor.reset_yaw_angle()
        if force_sensor_port:
            self.force_sensor = ForceSensor(force_sensor_port)

    def setRoboPosition(self,pos):
        self.position = pos

    def pointTo(self, degrees, precision = 1):
        dir = self.position[2]
        if(dir < degrees and abs(dir - degrees) < 180):
            while not self.hub.motion_sensor.get_yaw_angle() in range(degrees - precision, degrees + precision):
                self.motors.start_tank(10,-10)
            self.motors.stop()
        else:
            while not self.hub.motion_sensor.get_yaw_angle() in range(degrees - precision, degrees + precision):
                self.motors.start_tank(-10,10)
            self.motors.stop()
        print('turning to' + str(degrees) + ' degrees')
        self.position[2] = self.hub.motion_sensor.get_yaw_angle()
        self.map.points[-1][2] = self.position[2]
        if self.hub.motion_sensor.get_yaw_angle() in range(degrees - precision, degrees + precision):
            print("Succeded")
        else:
            self.pointTo(degrees)

    def calibrateDirTo(self, goal, precision = 1):
        dir = self.position[2]
        if dir < goal:
            while not self.hub.motion_sensor.get_yaw_angle() == goal:
                real_value = self.hub.motion_sensor.get_yaw_angle()
                diff = goal - real_value
                print(str(real_value) + " missing " + str(diff) + " to turn ")
        else:
            while not self.hub.motion_sensor.get_yaw_angle() == goal:
                real_value = self.hub.motion_sensor.get_yaw_angle()
                diff = real_value - goal
                print(str(real_value) + " missing " + str(diff) + " to turn ")
        real_value = self.hub.motion_sensor.get_yaw_angle()
        print('dir calibrated to ' + str(real_value) + ' degrees, the goal was ' + str(goal))
        self.position[2] = self.hub.motion_sensor.get_yaw_angle()
        self.map.points[-1][2] = self.position[2]

    def moveX(self,q):
        if q:
            if q > 0:
                self.pointTo(90)
            else:
                self.pointTo(-90)
            self.motors.move(abs(q),'cm', 0, self.speed)
            print('moving ' + str(q) + ' on X')
            self.position[0] += q
            self.map.addPoint(self.position)

    def moveY(self,q):
        if q :
            if q > 0:
                self.pointTo(0)
            else:
                self.pointTo(178)
            self.motors.move(abs(q),'cm', 0, self.speed)
            print('moving ' + str(q) + ' on Y')
            self.position[1] += q
            self.map.addPoint(self.position)

    def goTo(self,x,y):
        x =x + (-1*self.position[0])
        y =y + (-1*self.position[1])
        dist = math.sqrt(x**2 + y**2)
        if x != 0 and y > 0:
            self.pointTo(round(math.degrees(math.atan(x/y))))
        elif x > 0 and y < 0:
            self.pointTo(90 + round(abs(math.degrees(math.atan(y/x)))))
        elif x < 0 and y < 0:
            self.pointTo(-90 - round(abs(math.degrees(math.atan(y/x)))))
        elif x == 0 and y != 0:
            self.moveY(y)
            return 1
        elif x != 0 and y == 0:
            self.moveX(x)
            return 1
        else:
            return 0
        self.motors.move(dist,'cm', 0, self.speed)
        self.position[0] += x
        self.position[1] += y
        self.map.addPoint(self.position)
        return 1

    def doRoute(self, pointlist, goandback = 'go'):
        for point in pointlist:
            self.goTo(point[0],point[1])
        if goandback == 'go_comeback':
            lista = self.map.points.copy()
            lista.reverse()
            self.doRoute(lista)

    def goBack(self, dist):
        self.motors.move(dist,'cm', 0, -self.speed)
        x = abs(dist * math.cos(self.position[2]))
        y = abs(dist * math.sin(self.position[2]))
        self.position[0] -= x
        self.position[1] -= y
        self.map.addPoint(self.position)

robo1 = Robot('F', 'B', 'A', [0,0,0], 'D')

def realign(robo):
    last_x = robo.position[0]
    robo.pointTo(0)
    while not robo.force_sensor.is_pressed():
        robo.motors.start(0, -15)
    robo.motors.stop()
    robo.setRoboPosition([last_x,-60,0])

def align():
    robo1.motors.move(40, 'cm', 0, 40)
    robo1.pointTo(128)
    while not robo1.force_sensor.is_pressed():
        robo1.motors.start(0, -15)
    robo1.motors.stop()
    # robo.setRoboPosition([60,-60,0])

def main():


    animal_enfermo1 = [-36,27.1]
    animal_enfermo2 = [0,14.5]
    animal_enfermo3 = [22,39]
    p_medio_enfermo = [100,-30]
    destino_enfermo1 = [100,-9]
    destino_enfermo2 = [110,-9]
    destino_enfermo3 = [117,-9]
    animal_encalhado1 = [-85,-26.5]
    animal_encalhado2 = [-45.0,-13.5]
    animal_encalhado3 = [-2.3,-37.5]
    destino_encalhado1 = [-16,49]
    destino_encalhado2 = [10.5, 52.5]
    destino_encalhado3 = [37.2,52.5]
    petroleo = [-94.0,45.9]
    frenterampa = [57,11]
    potomedioencalhado = [-3.5,-9]


    estado = 'alinhamento'
    while True:
        if estado == 'alinhamento':
            align()
            robo = Robot('F', 'B', 'A', [0,0,0], 'D')
            robo.setRoboPosition([60,-48,0])
            estado = 'animal_enfermo'
        if estado == 'animal_enfermo':
            # robo.claw.Open()
            robo.goTo(animal_enfermo3[0], animal_enfermo3[1])
            robo.claw.Close()
            robo.goTo(p_medio_enfermo[0], p_medio_enfermo[1])
            # robo.pointTo(5)
            robo.claw.Open()
            robo.goTo(destino_enfermo1[0], destino_enfermo1[1])
            realign(robo)
            

            robo.goTo(animal_enfermo2[0], animal_enfermo2[1])
            robo.goTo(animal_enfermo1[0], animal_enfermo1[1])
            
            robo.claw.Close()
            realign(robo)
            robo.goTo(p_medio_enfermo[0], p_medio_enfermo[1])
            robo.claw.Open()
            robo.goTo(destino_enfermo1[0], destino_enfermo1[1])
            realign(robo)

            estado = 'animal_encalhado'

        if estado == 'animal_encalhado':
            robo.goTo(potomedioencalhado[0],potomedioencalhado[1])
            robo.goTo(animal_encalhado2[0], animal_encalhado2[1])
            robo.goTo(animal_encalhado1[0], animal_encalhado1[1])
            robo.claw.Close()
            robo.goTo(destino_encalhado1[0],destino_encalhado1[1])
            robo.claw.Open()

            robo.goTo(animal_encalhado3[0],animal_encalhado3[1])
            robo.claw.Close()
            robo.goTo(destino_encalhado3[0],destino_encalhado3[1])
            robo.claw.Open()
            realign(robo)

            estado = "garra"
        if estado == "garra":
            robo.goTo(frenterampa[0],freterampa[1])
            while not robo.force_sensor.is_pressed():
                robo.motors.start(0, -15)
            robo.motors.stop()
            estado = 'petroleo'
        if estado == "petroleo":

            estado = "acabou"
        if estado == 'acabou':
            break

    # print(robo.map.points)
main()
