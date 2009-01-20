import itertools as it
import os
import sys
import bpy
import Blender
import math
import random
import array

class road(object):
    __slots__ = ["nlanes", "lanewidth", "length", "carwidth", "carlength"]
    def __init__(self, str):
        data = str.split()
        self.nlanes    = int(data[0])
        for i in it.izip(self.__slots__[1:], data[1:]):
            setattr(self, i[0], float(i[1]))
    def __getstate__(self):
        return tuple([getattr(self, i) for i in self.__slots__])
    def __setstate__(self, state):
        for i in it.izip(self.__slots__, state):
            setattr(self, i[0], i[1])
    def __repr__(self):
        return "< %f m long, %d lanes each %f wide. Cars %f x %f >" % (self.length, self.nlanes, self.lanewidth, self.carwidth, self.carlength)
    def make_blender_road(self, scene, road_curve_obj):
        if(road_curve_obj == None):
            road_curve = Blender.Curve.New()
            road_curve.appendNurb([0.0, 0.0, 0.0, 1.0, 0.0])
            cu = road_curve[0]
            cu.append([self.length/3.0  , 0.0, 0.0, 1.0, 0.0])
            cu.append([2*self.length/3.0, 0.0, 0.0, 1.0, 0.0])
            cu.append([self.length      , 0.0, 0.0, 1.0, 0.0])
            cu.setFlagU(2)
            road_curve_obj = scene.objects.new(road_curve, "road_curve")

        corners = [[0,                            0, 0],   [self.length,                  0, 0],
                   [self.length, self.lanewidth*0.5, 0],   [0,           self.lanewidth*0.5, 0]]
        faces = [[0, 1, 2, 3]]
        for l in xrange(1, self.nlanes):
            cidx = len(corners)
            faces += [[cidx-1, cidx-2, cidx+1, cidx  ],
                      [cidx  , cidx+1, cidx+2, cidx+3]]

            corners +=   [[0,           self.lanewidth*l,       0],   [self.length, self.lanewidth*l,       0],
                          [self.length, self.lanewidth*(l+0.5), 0],   [0,           self.lanewidth*(l+0.5), 0]]

        cidx = len(corners)
        faces += [[cidx-1, cidx-2, cidx+1, cidx]]
        corners += [[0, self.lanewidth*self.nlanes,   0],   [self.length,  self.lanewidth*self.nlanes,   0]]

        me = bpy.data.meshes.new('road_mesh')

        me.verts.extend(corners)
        me.faces.extend(faces)

        mat = Blender.Mathutils.Matrix()
        mat.identity()
        mat[3][1] = -self.lanewidth*0.5*self.nlanes

        me.transform(mat)

        for i in it.islice(me.faces, 1, len(me.faces)-1):
            i.mat = 1
        me.faces[0].mat = 0
        me.faces[-1].mat = 0

        me.materials = [Blender.Material.Get(i) for i in ['Boundarylane', 'Middlelane']]

        for i in range(0, len(me.faces), 2):
            me.faces[i].uv = ( Blender.Mathutils.Vector([0, 0]),
                               Blender.Mathutils.Vector([self.length, 0]),
                               Blender.Mathutils.Vector([self.length, 1]),
                               Blender.Mathutils.Vector([0, 1]))

        for i in range(1, len(me.faces), 2):
            me.faces[i].uv = ( Blender.Mathutils.Vector([0, 1]),
                               Blender.Mathutils.Vector([self.length, 1]),
                               Blender.Mathutils.Vector([self.length, 0]),
                               Blender.Mathutils.Vector([0, 0]))

        obj = scene.objects.new(me, 'road')
        obj.layers = [1]

        mod = obj.modifiers.append(Blender.Modifier.Types.CURVE)
        mod[Blender.Modifier.Settings.OBJECT] = road_curve_obj
        return (road_curve_obj, obj)

class car(object):
    __slots__ = ["id", "time", "x", "y", "orientation", "steer_orientation", "velocity", "acceleration", "lanechanges", "brakepoints"]
    __module__ = "traffic"
    def __init__(self, id):
        self.id = id
        for i in self.__slots__[:-1]:
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
        lasty = self.y[0]
        i = 1
        while i < self.nrecs():
            dy = self.y[i] - lasty
            if dy < 0.0:
                dy = -1
            elif dy > 0.0:
                dy = 1
            else:
                dy = 0

            lasty = self.y[i]
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
        current_start = None
        i = 0
        while i < self.nrecs():
            if current_start == None:
                if self.acceleration[i] < 0:
                    current_start = i
            elif self.acceleration[i] >= 0:
                self.brakepoints.append((self.time[current_start], self.time[i-1]))
                current_start = None
            i += 1
        if current_start != None:
            self.brakepoints.append((self.time[current_start], self.time[i-1]))


