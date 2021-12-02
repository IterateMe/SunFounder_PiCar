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
import math
import time

class Vehicule:
    def __init__(self):
        picar.setup()
        self._fw = front_wheels.Front_Wheels(db='config')
        self._bw = back_wheels.Back_Wheels(db='config')
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
            self._bw.set_offset_speed(distal+correction_vitesse[0], proximal+correction_vitesse[1])
        if deg < 0 :
            self._bw.set_offset_speed(proximal+correction_vitesse[0], distal+correction_vitesse[1])
        if(avancer):
            self._bw.forward()
        else:
            self._bw.backward()
        self._bw.speed = speed

    def tout_droit(self, speed, avancer=True):
        self._fw.turn(90)
        # Freiner si self.avance = False
        self.avance = True
        self._bw.set_offset_speed(self.correction_vitesse[0], self.correction_vitesse[1])
        if(avancer):
            self._bw.forward()
        else:
            self._bw.backward()
        self._bw.speed = speed

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
