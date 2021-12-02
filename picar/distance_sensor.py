#!/usr/bin/env python
'''
**********************************************************************
* Filename    : distance_sensor.py
* Description : A module to use the distance sensor
* Author      : Vianney Gall
* Brand       : UdS
**********************************************************************
'''

import time
import RPi.GPIO as GPIO


class Distance_sensor(object):
    timeout = 0.05

    def __init__(self, channel=20):
        self.channel = channel
        GPIO.setmode(GPIO.BCM)

    def d_t(self):
        pulse_end = 0
        pulse_start = 0
        GPIO.setup(self.channel,GPIO.OUT)
        GPIO.output(self.channel, False)
        time.sleep(0.01)
        GPIO.output(self.channel, True)
        time.sleep(0.00001)
        GPIO.output(self.channel, False)
        GPIO.setup(self.channel,GPIO.IN)

        timeout_start = time.time()
        while GPIO.input(self.channel)==0:
            pulse_start = time.time()
            # print("START", pulse_start)
            if pulse_start - timeout_start > self.timeout:
                return -1
        while GPIO.input(self.channel)==1:
            pulse_end = time.time()
            # print("END", pulse_end)
            if pulse_start - timeout_start > self.timeout:
                return -1
        return pulse_end-pulse_start

    def d_t_loop(self, nbr):
        count = 0
        avg = 0.0
        while(count<nbr):
            time.sleep(0.1)
            value = self.d_t()
            print("COUNT = {} ------- {} :: {}".format(count, avg, value))
            if value != -1:
                avg = avg + value
                count += 1
            else:
                continue
        print("result = {}".format(avg/nbr))
        return avg/nbr

    def get_distance(self):
        dt = self.d_t()
        while dt == -1:
            print("Error")
            dt = self.d_t()
        distance = dt * 100 * 343.0 /2
        return distance - 2.3


# if __name__ == '__main__':
# 	test = Ultrasonic_Avoidance(20)
# 	data = test.d_t_loop(5)
# 	print(data)
# 	print("LENGTH", len(data))