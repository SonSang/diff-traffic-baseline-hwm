import itertools as it
import os
import sys
import bpy
import Blender
import math
import random
import array

class car(object):
    __slots__ = ["id", "time", "x", "y", "z", "nx", "ny", "xv", "yv", "accelerations", "lanechanges", "brakepoints"]
    __module__ = "traffic"
    def __init__(self, id):
        self.id = id
        for i in self.__slots__[1:-2]:
            setattr(self, i, array.array('f'))
        self.lanechanges = []
        self.brakepoints = []
    def __getstate__(self):
        return tuple([getattr(self, i) for i in self.__slots__])
    def __setstate__(self, state):
        for i in it.izip(self.__slots__, state):
            setattr(self, i[0], i[1])
    def nrecs(self):
        return len(self.time)
    def interval(self):
        return self.time[-1] - self.time[0]
    def process_lanechanges(self, leadin, leadout):
        current_start = None
        current_dir = 0
        i = 1
        while i < self.nrecs():
            dy = self.yv[i]
            if current_start == None:
                if dy != 0:
                    current_start = i
                    current_dir = dy
            else:
                if dy == 0:
                    self.lanechanges.append([self.time[current_start], self.time[i-1], current_dir])
                    current_dir = 0
                    current_start = None
                elif dy != current_dir:
                    self.lanechanges.append([self.time[current_start], self.time[i-1], current_dir])
                    current_dir = dy
                    current_start = i
            i += 1
        my_eps = 0.001
        if len(self.lanechanges) > 0:
            self.lanechanges[0][0] -= leadin
            if(self.lanechanges[0][0] < 0.0):
                self.lanechanges[0][0] = 0.0

            for lc in xrange(1, len(self.lanechanges)):
                self.lanechanges[lc][0] -= leadin
                if self.lanechanges[lc][0] <= self.lanechanges[lc-1][1]:
                    self.lanechanges[lc][0] = self.lanechanges[lc-1][1] + my_eps
                else:
                    self.lanechanges[lc-1][1] += leadout
                    if self.lanechanges[lc-1][1] >= self.lanechanges[lc][0]:
                        self.lanechanges[lc-1][1] = self.lanechanges[lc][0] - my_eps
    def process_brakepoints(self):
        if self.nrecs() < 2:
            return
        self.accelerations.append(0)
        for i in xrange(1, self.nrecs()):
            self.accelerations.append((self.xv[i]-self.xv[i-1])/self.time[i])
        self.accelerations[0] = self.accelerations[1]
        current_start = None
        i = 0
        while i < self.nrecs():
            if current_start == None:
                if self.accelerations[i] < -1e-4:
                    if len(self.brakepoints) > 1 and self.time[i] - self.brakepoints[-1][1] < 1.0:
                        current_start = self.brakepoints[-1][0] # merge consecutives
                        self.brakepoints.pop()
                    else:
                        current_start = i
            elif self.accelerations[i] >= 0:
                if self.time[i-1] - self.time[current_start] > 0.5: # don't record short decelerations
                    self.brakepoints.append((self.time[current_start], self.time[i-1]))
                current_start = None
            i += 1
        if current_start != None:
            self.brakepoints.append((self.time[current_start], self.time[i-1]))


