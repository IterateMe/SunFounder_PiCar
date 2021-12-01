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

    def tourner_avancer(self, speed, deg):
        # deg va de -45 a +45
        # Freiner si self.avance = False
        fw_angle = deg+90
        self.avance = True
        self._fw.turn(fw_angle)

        distal, proximal = self.calcul_offset_vitesse(speed, deg)
        print ("DISTAL : {}\tPROXIMAL : {}\n".format(distal, proximal))
        if deg >= 0 :
            self._bw.set_offset_speed(distal, proximal)
        if deg < 0 :
            self._bw.set_offset_speed(proximal, distal)
        
        self._bw.forward()
        self._bw.speed = speed


    def tourner_reculer(self, speed, deg):
        self._fw.turn(deg)
        # Freiner si self.avance = True
        self.avance = False

    def avancer(self, speed):
        self._fw.turn(90)
        # Freiner si self.avance = False
        self.avance = True
        self._bw.set_offset_speed(0,0)
        self._bw.forward()
        self._bw.speed = speed

    def reculer(self, speed):
        self._fw.turn(90)
        # Freiner si self.avance = True
        self.avance = False
        self._bw.set_offset_speed(0,0)
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

    # def test_fw(self):
    #     self._fw.turn(90)
    #     time.sleep(1)
    #     self._fw.turn(45)
    #     time.sleep(1)
    #     self._fw.turn(45+90)