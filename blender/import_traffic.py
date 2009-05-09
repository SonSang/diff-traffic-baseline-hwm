import os
import sys
import itertools as it
import bpy
import Blender
import math
import random
import array
import time

import cPickle
sys.path.append(os.getcwd())

import traffic
reload(traffic)
#import nurbs
#reload(nurbs)

fps = 25.0

#<t (s)> <n cars this step>
#<Car id (integer)>  <X-position (meters)> <Y-position (meters)> <Z-position (meters)> <X-component of dir> <Y-component of dir> <X-velocity (m/s)> <Y-velocity (m/s)>

available_cars = { 'tbird' :
                       {'body': 'tbird-body',
                        'wheel': 'tbird-wheel',
                        'wheel-flip': 'tbird-wheel-flip',
                        'wheel-radius' : 0.751*0.5,
                        'wheel_points': [[   0.0,  0.808, -0.507],
                                         [ 2.916,  0.808, -0.507],
                                         [ 2.916, -0.808, -0.507],
                                         [   0.0, -0.808, -0.507]],
                        'z-offs' :  0.507 + 0.751*0.5,
                        'colors' : [[ 0.843137254902, 0.0745098039216, 0.0745098039216 ],
                                    [ 0.450980392157, 0.788235294118, 1.0 ],
                                    [ 0.0, 0.447058823529, 0.0941176470588 ],
                                    [ 1.0, 0.945098039216, 0.0941176470588 ],
                                    [ 0.98431372549, 0.901960784314, 0.450980392157 ],
                                    [ 0.0, 0.239215686275, 0.533333333333 ],
                                    [ 0.811764705882, 0.811764705882, 0.811764705882 ],
                                    [ 0.18431372549, 0.18431372549, 0.18431372549]]
                        },
                   'golf' :
                       {'body': 'golf-body',
                        'wheel': 'golf-wheel',
                        'wheel-flip': 'golf-wheel-flip',
                        'wheel-radius' : 0.578*0.5,
                        'wheel_points': [[   0.0,  0.722, -0.521],
                                         [ 2.490,  0.722, -0.521],
                                         [ 2.490, -0.722, -0.521],
                                         [   0.0, -0.722, -0.521]],
                        'z-offs' :  0.521 + 0.578*0.5,
                        'colors' : [[ 0.992156862745, 0.992156862745, 0.992156862745 ],
                                    [ 0.286274509804, 0.376470588235, 0.76862745098 ],
                                    [ 0.858823529412, 0.792156862745, 0.41568627451 ],
                                    [ 0.474509803922, 0.407843137255, 0.243137254902 ],
                                    [ 0.345098039216, 0.345098039216, 0.345098039216 ],
                                    [ 0.450980392157, 0.0705882352941, 0.607843137255 ],
                                    [ 0.117647058824, 0.388235294118, 0.16862745098 ]]
                        },
                   'pickup' :
                       {'body': 'pickup-body',
                        'wheel': 'pickup-wheel',
                        'wheel-flip': 'pickup-wheel-flip',
                        'wheel-radius' : 0.721*0.5,
                        'wheel_points': [[   0.0,  0.851, -0.513],
                                         [ 3.244,  0.851, -0.513],
                                         [ 3.244, -0.851, -0.513],
                                         [   0.0, -0.851, -0.513]],
                        'z-offs' : 0.513 + 0.721*0.5,
                        'colors' : [[ 0.698039215686, 0.0, 0.0823529411765 ],
                                    [ 0.980392156863, 0.952941176471, 0.921568627451 ],
                                    [ 0.0, 0.376470588235, 0.101960784314 ],
                                    [ 0.121568627451, 0.16862745098, 0.819607843137 ],
                                    [ 0.18431372549, 0.18431372549, 0.18431372549 ],
                                    [ 0.498039215686, 0.435294117647, 0.254901960784 ]]
                        },
                   'chevyvan' :
                       {'body': 'chevyvan-body',
                        'wheel': 'chevyvan-wheel',
                        'wheel-flip': 'chevyvan-wheel-flip',
                        'wheel-radius' : 0.709*0.5,
                        'wheel_points': [[   0.0,  0.922, -0.619],
                                         [ 3.611,  0.922, -0.619],
                                         [ 3.611, -0.922, -0.619],
                                         [   0.0, -0.922, -0.619]],
                        'z-offs' : 0.619 + 0.709*0.5,
                        'colors' : [[ 0.929411764706, 0.929411764706, 0.929411764706 ],
                                    [ 0.827450980392, 0.737254901961, 0.658823529412 ],
                                    [ 0.0745098039216, 0.247058823529, 0.866666666667 ],
                                    [ 0.0, 0.396078431373, 0.145098039216 ],
                                    [ 0.486274509804, 0.21568627451, 0.725490196078 ],
                                    [ 0.364705882353, 0.639215686275, 1.0 ]]
                        },
                   'accord' :
                       {'body': 'accord-body',
                        'wheel': 'accord-wheel',
                        'wheel-flip': 'accord-wheel-flip',
                        'wheel-radius' : 0.698*0.5,
                        'wheel_points': [[   0.0,  0.691, -0.538],
                                         [ 2.787,  0.691, -0.538],
                                         [ 2.787, -0.691, -0.538],
                                         [   0.0, -0.691, -0.538]],
                        'z-offs' : 0.538 + 0.698*0.5,
                        'colors' : [[ 0.843137254902, 0.0745098039216, 0.0745098039216 ],
                                    [ 0.450980392157, 0.788235294118, 1.0 ],
                                    [ 0.0, 0.447058823529, 0.0941176470588 ],
                                    [ 1.0, 0.945098039216, 0.0941176470588 ],
                                    [ 0.98431372549, 0.901960784314, 0.450980392157 ],
                                    [ 0.0, 0.239215686275, 0.533333333333 ],
                                    [ 0.811764705882, 0.811764705882, 0.811764705882 ],
                                    [ 0.18431372549, 0.18431372549, 0.18431372549 ],
                                    [ 0.992156862745, 0.992156862745, 0.992156862745 ],
                                    [ 0.286274509804, 0.376470588235, 0.76862745098 ],
                                    [ 0.858823529412, 0.792156862745, 0.41568627451 ],
                                    [ 0.474509803922, 0.407843137255, 0.243137254902 ],
                                    [ 0.345098039216, 0.345098039216, 0.345098039216 ],
                                    [ 0.450980392157, 0.0705882352941, 0.607843137255 ],
                                    [ 0.117647058824, 0.388235294118, 0.16862745098 ],
                                    [ 0.698039215686, 0.0, 0.0823529411765 ],
                                    [ 0.980392156863, 0.952941176471, 0.921568627451 ],
                                    [ 0.0, 0.376470588235, 0.101960784314 ],
                                    [ 0.121568627451, 0.16862745098, 0.819607843137 ],
                                    [ 0.18431372549, 0.18431372549, 0.18431372549 ],
                                    [ 0.498039215686, 0.435294117647, 0.254901960784 ],
                                    [ 0.929411764706, 0.929411764706, 0.929411764706 ],
                                    [ 0.827450980392, 0.737254901961, 0.658823529412 ],
                                    [ 0.0745098039216, 0.247058823529, 0.866666666667 ],
                                    [ 0.0, 0.396078431373, 0.145098039216 ],
                                    [ 0.486274509804, 0.21568627451, 0.725490196078 ],
                                    [ 0.364705882353, 0.639215686275, 1.0 ]]
                        }}

def print_timing(func):
    def wrapper(*arg):
        t1 = time.time()
        res = func(*arg)
        t2 = time.time()
        print '%s took %0.3f ms' % (func.func_name, (t2-t1)*1000.0)
        return res
    return wrapper

def do_blinker(e_curve, start_t, stop_t, period):
    start_frame = start_t*fps

    e_curve[start_frame] = 0.0

    t = start_t
    onoff = 1.0
    while t < stop_t:
        t += period
        e_curve[t*fps] = onoff
        onoff = 1.0-onoff

    e_curve[t*fps] = 0.0

def find_mat(matlist, name):
    for i in xrange(0, len(matlist)):
        if matlist[i].getName().startswith(name):
            return i
    return None

@print_timing
def apply_basic_motion(id, cartype, carseries, carobj, wheel_rad):
    ipo_r = Blender.Ipo.New("Object", "car_%d_root_ipo" % id)

    ipo_consts = [(Blender.Ipo.OB_LOCX, 'LocX'),
                  (Blender.Ipo.OB_LOCY, 'LocY'),
                  (Blender.Ipo.OB_LOCZ, 'LocZ')]
    ipo_curves = []
    for ipc in ipo_consts:
        if ipo_r[ipc[0]] != None:
            ipo_r[ipc[0]] = None
        curv = ipo_r.addCurve(ipc[1])
        ipo_curves.append(curv)

    zoffs = cartype['z-offs']
    for i in xrange(0, carseries.nrecs()):
        fr = carseries.time[i]*fps

        ipo_curves[0][fr] = carseries.x[i]
        ipo_curves[1][fr] = carseries.y[i]
        ipo_curves[2][fr] = carseries.z[i] + zoffs
        
    zrot_curve = ipo_r.addCurve("RotZ")
    lastangle = math.atan2(carseries.ny[0],carseries.nx[0]) * 18.0/math.pi
    for i in xrange(0, carseries.nrecs()):
        frame = carseries.time[i]*fps
        angle = math.atan2(carseries.ny[i],carseries.nx[i]) * 18.0/math.pi
        if (angle - lastangle) > 18:
            angle = angle-36
        elif (lastangle - angle) > 18:
            angle = angle+36
        zrot_curve[frame] = angle
        lastangle = angle
    carobj[0].setIpo(ipo_r)

    ipo_s = Blender.Ipo.New("Object", "car_%d_steer_ipo" % id)
    zrot_curve = ipo_s.addCurve("dRotZ")
#    for i in xrange(0, carseries.nrecs()):
#        frame = carseries.time[i]*fps
#        zrot_curve[frame] = carseries.steer_orientation[i] * 18.0/math.pi
    carobj[1].setIpo(ipo_s)

    ipo_d = Blender.Ipo.New("Object", "car_%d_drive_ipo" % id)
    yrot_curve = ipo_d.addCurve("RotY")
    frame = carseries.time[0]*fps
    yrot_curve[frame] = 0
    lasty = yrot_curve[frame]
    for i in xrange(1, carseries.nrecs()):
        frame = carseries.time[i]*fps
        dx = carseries.x[i]-carseries.x[i-1]
        dy = carseries.y[i]-carseries.y[i-1]
        dist = math.sqrt(dx*dx+dy*dy)
        yrot_curve[frame] = lasty + dist/(wheel_rad*math.pi) * 18.0
        lasty = yrot_curve[frame]
    carobj[2].setIpo(ipo_d)

    mats = carobj[3].getMaterials()

    tailmat = find_mat(mats, "Taillight")
    assert tailmat != None
    rightmat = find_mat(mats, "rightturn")
    assert rightmat != None
    leftmat = find_mat(mats, "leftturn")
    assert leftmat != None

#    tail_ipo = Blender.Ipo.New("Material", "car_%d_%s_ipo" % (id, mats[tailmat].getName()))
#    mats[tailmat].setIpo(tail_ipo)
#    t_curve = tail_ipo.addCurve("Emit")
#    frame = carseries.time[0]*fps
#    t_curve[frame] = 0.0
#    t_curve.interpolation = Blender.IpoCurve.InterpTypes.CONST
#    for bp in carseries.brakepoints:
#        t_curve[bp[0]*fps] = 1.0
#        t_curve[bp[1]*fps] = 0.0

#    right_ipo = Blender.Ipo.New("Material", "car_%d_%s_ipo" % (id, mats[rightmat].getName()))
#    mats[rightmat].setIpo(right_ipo)
#    right_curve = right_ipo.addCurve("Emit")
#    right_curve.interpolation = Blender.IpoCurve.InterpTypes.CONST
#
#    left_ipo = Blender.Ipo.New("Material", "car_%d_%s_ipo" % (id, mats[leftmat].getName()))
#    mats[leftmat].setIpo(left_ipo)
#    left_curve = left_ipo.addCurve("Emit")
#    right_curve.interpolation = Blender.IpoCurve.InterpTypes.CONST

#    for lc in carseries.lanechanges:
#        if lc[2] == -1:
#            ccurve = right_curve
#        else :
#            ccurve = left_curve
#        do_blinker(ccurve, lc[0], lc[1], 0.1)

    intime  = carseries.time[ 0]*fps
    outtime = carseries.time[-1]*fps
    for i in carobj:
        ipo_v = i.getIpo()
        if ipo_v == None:
            ipo_v = Blender.Ipo.New("Object", "car_%d_%s_ipo" % (id, i.getName()))
            i.setIpo(ipo_v)
        layer_curve = ipo_v.addCurve("Layer")
        layer_curve[0]       = 0
        layer_curve[intime]  = 1
        layer_curve[outtime] = 0

#@print_timing
#def apply_curve_motion(id, cartype, carseries, road_nurb, the_road, carobj, steerobj, wheelblen):
#    ipo = carobj.getIpo()
#    ipo_consts = [(Blender.Ipo.OB_LOCX, 'LocX'),
#                  (Blender.Ipo.OB_LOCY, 'LocY'),
#                  (Blender.Ipo.OB_LOCZ, 'LocZ'),
#                  (Blender.Ipo.OB_ROTX, 'RotX'),
#                  (Blender.Ipo.OB_ROTY, 'RotY'),
#                  (Blender.Ipo.OB_ROTZ, 'RotZ')]
#
#    ipo_curves = []
#    for ipc in ipo_consts:
#        if ipo[ipc[0]] != None:
#            ipo[ipc[0]] = None
#        curv = ipo.addCurve(ipc[1])
#        ipo_curves.append(curv)
#
#    start = road_nurb.knots[ 0]
#    end   = road_nurb.knots[-1]
#
##    steer_ipo = steerobj.getIpo()
##    if steer_ipo[Blender.Ipo.OB_ROTZ] != None:
##        steer_ipo[Blender.Ipo.OB_ROTZ] = None
##    steer_curve = steer_ipo.addCurve('RotZ')
#
#    len = end-start
#
#    ustep = len/the_road.length
#
#    for i in xrange(0, carseries.nrecs()):
#        fr = carseries.time[i]*fps
#
#        u = start + carseries.x[i]*ustep
##        pts = road_nurb.ck(u,2)
#        pts = road_nurb.ck(u,1)
#        blpt = Blender.Mathutils.Vector(pts[0])
#        bltan = Blender.Mathutils.Vector(pts[1])
##        blnorm = Blender.Mathutils.Vector(pts[2])
##
##        curvature = blnorm.length
##
#        bltan = bltan.normalize()
#
#        ylocal = Blender.Mathutils.CrossVecs(Blender.Mathutils.Vector([0, 0, 1]), bltan)
#        ylocal.normalize()
#        zlocal = Blender.Mathutils.CrossVecs(bltan, ylocal)
#
#        yoffs = carseries.y[i] - (the_road.nlanes-1)*the_road.lanewidth*0.5
#        zoffs = cartype['z-offs']
#
#        ipo_curves[0][fr] = blpt[0] + yoffs*ylocal[0] + zoffs*zlocal[0]
#        ipo_curves[1][fr] = blpt[1] + yoffs*ylocal[1] + zoffs*zlocal[1]
#        ipo_curves[2][fr] = blpt[2] + yoffs*ylocal[2] + zoffs*zlocal[2]
#
#        mat = Blender.Mathutils.Matrix(bltan, ylocal, zlocal)
#        e = mat.toEuler()
#
#        ipo_curves[3][fr] = e.x * 0.1
#        ipo_curves[4][fr] = e.y * 0.1
#        ipo_curves[5][fr] = e.z * 0.1
#
##        zrot = math.atan(wheelblen*curvature)
##        steer_curve[fr] = zrot * 18.0 / math.pi

def read_car_record(fp, t, ncars, cardict):
    for i in xrange(0, ncars):
        data = fp.readline().rstrip().split()
        id = int(data[0])
        rec = [float(x) for x in data[1:]]
        if(cardict.has_key(id)):
            the_car = cardict[id]
        else:
            the_car = traffic.car(id)
            cardict[id] = the_car
        for i in it.izip(the_car.__slots__[1:], [t] + rec):
            getattr(the_car, i[0]).append(i[1])

@print_timing
def load_traffic(file):
    (froot, fext) = os.path.splitext(file)
    try:
        pick = open(froot+".pkc", 'rb')
        print "Loading pickled data from %s" % (froot+".pkc",)
        cars = cPickle.load(pick)
        pick.close()
    except:
        print "Loading plaintext data"
        fp = open(file, 'r')
        cars = dict()
        li = fp.readline().rstrip().split()
        while li:
            t    = float(li[0])
            ncars = int(li[1])
            read_car_record(fp, t, ncars, cars)
            li = fp.readline().rstrip().split()
        fp.close()
#        min_int = 1e10
#        min_car = -1
#        max_int = -1
#        max_car = -1
#        print "Processing lanechanges and brakepoints"
#        for i in cars.keys():
#            cars[i].process_lanechanges(2.0, 0.1)
#            cars[i].process_brakepoints()
#            iv = cars[i].interval()
#            if iv < min_int:
#                min_int = iv
#                min_car = i
#            elif iv > max_int:
#                max_int = iv
#                max_car = i
#        print "Slowest car is", max_car, " which takes", max_int, "seconds"
#        print "Fastest car is", min_car, " which takes", min_int, "seconds"
#        print "Pickling data to %s" % (froot+".pkc",)
#        pick = open(froot+".pkc", 'wb')
#        cPickle.dump((the_road, cars), pick)
#        pick.close()

    print "Read %d cars" % len(cars.keys())
    return cars

def build_car(scn, id, color, car, scale):
    print "Grabbing objects..."
    body_obj = Blender.Object.Get(car['body'])
    wheel_obj = Blender.Object.Get(car['wheel'])
    wheel_obj_flip = Blender.Object.Get(car['wheel-flip'])
    wheel_points = car['wheel_points']
    print "Got all neede objects"

    print "Making empties"
    root = scn.objects.new("Empty")
    root.setName("root_%d" % id)
    root.setSize(scale, scale, scale)

    steer = scn.objects.new("Empty")
    steer.setName("steering_%d" % id)

    drive = scn.objects.new("Empty")
    drive.setName("drive_%d" % id)
    print "Got empties"

    print "Copying body"
    scn.objects.selected = [body_obj]
    Blender.Object.Duplicate()
    new_body = scn.objects.selected[0]
    new_body.setName("%s_%d" % (body_obj.getName(), id))

    mats = new_body.getMaterials()

    body = find_mat(mats, "Bodycolor")
    assert body != None
    origname_b = mats[body].getName()
    mats[body] = mats[body].__copy__()
    mats[body].setName("%s_%d" % (origname_b, id))
    mats[body].setRGBCol(color)

    for i in ["leftturn", "rightturn", "Taillight"]:
        num = find_mat(mats, i)
        assert num != None
        origname_t = mats[num].getName()
        mats[num] = mats[num].__copy__()
        mats[num].setName("%s_%d" % (origname_t, id))
    new_body.setMaterials(mats)

    body_cons = new_body.constraints.append(Blender.Constraint.Type.CHILDOF)
    body_cons[Blender.Constraint.Settings.TARGET] = root
    print "Copied body"

# Wheel ordering
######################
#                    #
# + y ^              #
#                    #
# + x ->             #
#                    #
#^out of wheel_obj
#  -0-----1-         #
#  |        |<-front #
#  -3-----2-         #
#                    #
######################

    print "Copying wheels"
    scn.objects.selected = [wheel_obj]
    wheels = [None] * 4
    for i in [0, 1]:
        Blender.Object.Duplicate()
        wheels[i] = scn.objects.selected[0]
        wheels[i].setName("%s_n%d_%d" % (wheel_obj.getName(), i, id))

    scn.objects.selected = [wheel_obj_flip]
    for i in [2, 3]:
        Blender.Object.Duplicate()
        wheels[i] = scn.objects.selected[0]
        wheels[i].setName("%s_n%d_%d" % (wheel_obj_flip.getName(), i, id))

    print "Copied wheels"

    for i in range(0, 4):
        while len(wheels[i].constraints) > 0:
            wheels[i].constraints.remove(wheels[i].constraints[0])

        if i == 1 or i == 2:
            wheel_cons = wheels[i].constraints.append(Blender.Constraint.Type.COPYROT)
            wheel_cons[Blender.Constraint.Settings.COPY] = Blender.Constraint.Settings.COPYZ
            wheel_cons[Blender.Constraint.Settings.TARGET] = steer

        wheel_cons = wheels[i].constraints.append(Blender.Constraint.Type.COPYROT)

        wheel_cons[Blender.Constraint.Settings.COPY] = Blender.Constraint.Settings.COPYY
        wheel_cons[Blender.Constraint.Settings.TARGET] = drive

        wheel_cons = wheels[i].constraints.append(Blender.Constraint.Type.CHILDOF)
        wheel_cons[Blender.Constraint.Settings.TARGET] = root

        wheels[i].setLocation(wheel_points[i][0], wheel_points[i][1], wheel_points[i][2])

    return [root, steer, drive, new_body] + wheels

if __name__ == '__main__':
    print "Loading data"
    cars = load_traffic("/playpen/sewall/traffic/hwm/blender/output.txt")
    print "Data loaded"
    print cars[3].nx, cars[3].ny
    scn = bpy.data.scenes.active
    scn.setLayers(range(1, 21))

    #Blender.Registry.RemoveKey('import_traffic')
    cartypes = Blender.Registry.GetKey('import_traffic')
    if cartypes == None:
        cartypes = {}
        
    print "Doing cars"
    random.seed()
    for id in cars.keys():
        print "Processing car ", id
        try:
            car_root = Blender.Object.Get("root_%d" % id)
            car_steer = Blender.Object.Get("steering_%d" % id)
            print "Found car, don't need to load"
        except:
            car = available_cars.keys()[random.randint(0, len(available_cars.keys())-1)]
            print "Making a copy of ", car, " for car ", id
            ci = build_car(scn, id,
                           available_cars[car]['colors'][random.randint(0, len(available_cars[car]['colors'])-1)],
                           available_cars[car],
                           1.0)
            cartypes[id] = car
            print "Done making car"
            print "Applying basic motion to car"
            wheel_rad = available_cars[car]['wheel-radius']
            apply_basic_motion(id, available_cars[cartypes[id]], cars[id], ci, wheel_rad)
            print "Done applyng basic motion"
        print "Done processing car ", id
    print "Cars done"
    Blender.Registry.SetKey('import_traffic', cartypes)
    print "All done"

    ##Blender.Save("moab-200cars-downsamp.blend")