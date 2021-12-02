#!/usr/bin/env python
'''
**********************************************************************
* Filename    : vehicule.py
* Description : A module to control the back AND front wheels of RPi Car along with line follower and distance sensor
* Author      : Vianney Gall
* Brand       : UdS
**********************************************************************
'''

import picar
from .import back_wheels
from .import front_wheels
from .import distance_sensor
from .import line_follower
import math
import time

class Vehicule:
    def __init__(self):
        picar.setup()
        self._fw = front_wheels.Front_Wheels(db='config')
        self._bw = back_wheels.Back_Wheels(db='config')
        self._distance = distance_sensor.Distance_sensor()
        self._lf = line_follower.Line_Follower()
        self.avance = True
        self.longueur = 0.14 # metres
        self.dist_centre_roue = 0.054 # metres
        self.correction_vitesse = [1,0]

    def tourner(self, speed, deg, avancer=True):
        # deg va de -45 a +45
        # Freiner si self.avance != avancer
        fw_angle = deg+90
        self.avance = True
        self._fw.turn(fw_angle)

        distal, proximal = self.calcul_offset_vitesse(speed, deg)
        #print ("DISTAL : {}\tPROXIMAL : {}\n".format(distal, proximal))
        if deg >= 0 :
            self._bw.set_offset_speed(distal+self.correction_vitesse[0], proximal+self.correction_vitesse[1])
        if deg < 0 :
            self._bw.set_offset_speed(proximal+self.correction_vitesse[0], distal+self.correction_vitesse[1])
        if(avancer):
            self._bw.forward()
        else:
            self._bw.backward()
        self._bw.speed = speed

    def tout_droit(self, speed, avancer=True):
        self.tourner(speed, 0, avancer)
        

    def arret(self):
        self._bw.stop()
        self._fw.turn(90)
        self._bw.set_offset_speed(0,0)
    
    def freiner(self):
        pass

    def calcul_offset_vitesse(self, speed, deg):
        rad = math.radians(abs(deg))
        mult = self.dist_centre_roue * math.tan(rad) / self.longueur
        distal = int(mult * speed)
        proximal = -int(mult * speed)
        return distal, proximal

    def get_distance(self):
        dist = self._distance.get_distance()
        print("Distance = {}".format(dist))
        return dist

    def go_until_wall(self, speed, ds):
        distance = ds+1
        try:
            while(distance>ds):
                distance = self.get_distance()
                if distance<ds-3:
                    distance = ds+1
                self.tout_droit(speed)
                time.sleep(0.1)
            self.arret()
        except KeyboardInterrupt:
            self.arret()

    def avancer_ligne(self):
        pass

    def reculer_ligne(self):
        pass
        


