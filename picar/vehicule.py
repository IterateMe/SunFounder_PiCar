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
    def __init__(self, lf_ref=[114, 170, 180, 180, 125]):
        picar.setup()
        self._fw = front_wheels.Front_Wheels(db='config')
        self._bw = back_wheels.Back_Wheels(db='config')
        self._distance = distance_sensor.Distance_sensor()

        self._lf = line_follower.Suiveur_ligne()
        self._lf.references = lf_ref
        
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
        

    def arret(self,start=20,stop=20,delay=0):
        diff = start-stop
        for i in range(diff):
            self.tout_droit(start-i)
            time.sleep(delay)
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

    def avancer_ligne(self, speed=70, delay=0.005):
        following_line = True
        previous_lt_status = self._lf.read_digital()
        off_track_left = False
        off_track_right = False
        lt_status_now = None    # Will be set soon with somthing like [0,0,1,0,0]
        step = 0    # Set the angle at which the car will turn
        speed_modifier = 1  # To reduce speed at desired moments (while turning for exemple)
        line_state = 0 # line is : left=-1    center=0    right=1
        a_step = 3
        b_step = 10
        c_step = 30
        d_step = 45

        while following_line:
            lt_status_now = self._lf.read_digital()
            print(lt_status_now)

            if off_track_left or off_track_right:
                speed_modifier = 0.8
                if off_track_left:
                    print("OFF TRACK LEFT")
                    # Turn right
                    self.tourner(int(speed*speed_modifier), 45)
                    # verify if back on track
                    if lt_status_now in ([0,0,0,1,0],[0,0,0,1,1],[0,0,1,0,0]):
                        off_track_left = False
                        off_track_right = False
                        print("BACK ON TRACK")

                elif off_track_right:
                    print("OFF TRACK RIGHT")
                    # Turn left
                    self.tourner(int(speed*speed_modifier), -45)
                    # verify if back on track
                    if lt_status_now in ([0,1,0,0,0],[1,1,0,0,0],[0,0,1,0,0]):
                        off_track_left = False
                        off_track_right = False
                        print("BACK ON TRACK")

            else: # IF ON TRACK
                # Angle calculate
                if	lt_status_now == [0,0,1,0,0]:
                    speed_modifier = 1
                    step = 0	
                elif lt_status_now == [0,1,1,0,0] or lt_status_now == [0,0,1,1,0]:
                    speed_modifier = 1
                    step = a_step
                elif lt_status_now == [0,1,0,0,0] or lt_status_now == [0,0,0,1,0]:
                    speed_modifier = 1
                    step = b_step
                elif lt_status_now == [1,1,0,0,0] or lt_status_now == [0,0,0,1,1]:
                    speed_modifier= 1
                    step = c_step
                elif lt_status_now == [1,0,0,0,0] or lt_status_now == [0,0,0,0,1]:
                    speed_modifier = 0.8
                    step = d_step
                elif lt_status_now == [0,0,0,0,0]:  # OFF TRACK
                    speed_modifier = 0.8
                    step = d_step
                else:
                    speed_modifier = 1

                # Direction calculate
                if lt_status_now == [0,0,1,0,0]:
                    line_state = 0
                    self.tout_droit(int(speed*speed_modifier))
                # stop at the T shaped end
                elif lt_status_now == [1,1,1,1,1]:
                    line_state = 0
                    print("STTOOOOOOOOP")
                    self.arret(start=int(speed*speed_modifier))
                    break
                # turn right
                elif lt_status_now in ([0,1,1,0,0],[0,1,0,0,0],[1,1,0,0,0],[1,0,0,0,0]):
                    line_state = 1
                    self.tourner(int(speed*speed_modifier), -int(step)) 
                # turn left
                elif lt_status_now in ([0,0,1,1,0],[0,0,0,1,0],[0,0,0,1,1],[0,0,0,0,1]):
                    line_state = -1
                    self.tourner(int(speed*speed_modifier), int(step))
                #
                elif lt_status_now == [0,0,0,0,0]:
                    self.tourner(int(speed*speed_modifier), int(step)*(-line_state))
                    print("GOING OFF TRACK")
                    if line_state == 1:
                        off_track_right = True
                    elif line_state == -1:
                        off_track_left = True
                    # else:
                    #     print("ERREUR DE LOGIQUE")
                    #     self.arret(start=int(speed*speed_modifier))
                    #     break
                else:
                    self.tout_droit(int(speed*speed_modifier))

            previous_lt_status = lt_status_now
            time.sleep(delay)


    def reculer_ligne(self):
        pass
        
    def test_lf(self):
        while True:
            digital = self._lf.read_digital()
            analog = self._lf.read_analog()
            diff = analog[2]-analog[1]
            print(digital)
            print(analog,"\t{}".format(diff))
            print('')
            time.sleep(0.2)

    def test_dist_sensor(self):
        pass


