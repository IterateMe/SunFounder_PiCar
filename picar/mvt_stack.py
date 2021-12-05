#!/usr/bin/env python
'''
**********************************************************************
* Filename    : mvt_FIFO.py
* Description : Data structure made to manage mouvement data along the way while picar is running
* Author      : Vianney Gall
* Brand       : UdS
**********************************************************************
'''

class Mvt_Stack:
    def __init__(self, max_length):
        self.data = []
        self.max_length = max_length

    def insert(self,data):
        self.data.append(data)
        if len(self.data) >max_length:
            element = self.data.pop(0)
            return 0, element
        else:
            return 1, None

    def get(self):
        if len(self.data) < 1:
            return 0
        else:
            return self.data.pop()