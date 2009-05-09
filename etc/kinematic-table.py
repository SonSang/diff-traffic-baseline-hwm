import subprocess
import math
import pylab
import numpy

exe = './ss_kinematic'

car_length = 4.5
lane_width = 2.5
max_steering_angle = math.pi/4.0
final_orient = math.pi/2.0
resolution = 50

def ke_lc_lengths():
    exec_seq = [exe, 0, str(lane_width), str(car_length), str(max_steering_angle)]

    speed_int = [5, 100]
    nspeeds = 100

    speeds    = numpy.zeros(nspeeds)
    times     = numpy.zeros(nspeeds)
    distances = numpy.zeros(nspeeds)

    for speed_count in xrange(0, nspeeds):
        speed = ((speed_int[1]-speed_int[0])*speed_count/(nspeeds-1.0) + speed_int[0])/3.6
        exec_seq[1] = str(speed)
        p = subprocess.Popen(exec_seq, stderr=subprocess.PIPE, stdout=subprocess.PIPE)
        p.wait()
        p.stderr.readline()
        [time, distance] = [float(x) for x in p.stderr.readline().strip().split()]
        speeds   [speed_count] = speed
        times    [speed_count] = time
        distances[speed_count] = distance

    return (speeds, times, distances)

def ke_ss_lengths():
    exec_seq = [exe, 0, str(10/3.6), str(final_orient), str(lane_width), str(car_length), str(max_steering_angle)]

    speed_int = [15, 100]
    nspeeds = 100

    speeds    = numpy.zeros(nspeeds)
    times     = numpy.zeros(nspeeds)
    distances = numpy.zeros(nspeeds)

    for speed_count in xrange(0, nspeeds):
        speed = ((speed_int[1]-speed_int[0])*speed_count/(nspeeds-1.0) + speed_int[0])/3.6
        exec_seq[1] = str(speed)
        p = subprocess.Popen(exec_seq, stderr=subprocess.PIPE, stdout=subprocess.PIPE)
        p.wait()
        p.stderr.readline()
        [time, distance] = [float(x) for x in p.stderr.readline().strip().split()]
        speeds   [speed_count] = speed
        times    [speed_count] = time
        distances[speed_count] = distance

    return (speeds, times, distances)

def ke_lc_path(speed):
    exec_seq = [exe, str(speed), str(lane_width), str(car_length), str(max_steering_angle), str(resolution)]

    p = subprocess.Popen(exec_seq, stderr=subprocess.PIPE, stdout=subprocess.PIPE)

    di = dict( ((i, c) for c,i in enumerate(p.stdout.readline().split())))
    data = numpy.zeros((0, len(di)), numpy.float)

    for l in p.stdout:
        data.resize((data.shape[0]+1, data.shape[1]))
        data[data.shape[0]-1,:] = [float(x) for x in l.split()]

    return (di, data)

def ke_ss_path(v0, v1):
    exec_seq = [exe, str(v0), str(v1), str(final_orient), str(car_length), str(max_steering_angle), str(resolution)]

    p = subprocess.Popen(exec_seq, stderr=subprocess.PIPE, stdout=subprocess.PIPE)

    di = dict( ((i, c) for c,i in enumerate(p.stdout.readline().split())))
    data = numpy.zeros((0, len(di)), numpy.float)

    for l in p.stdout:
        data.resize((data.shape[0]+1, data.shape[1]))
        data[data.shape[0]-1,:] = [float(x) for x in l.split()]

    return (di, data)

def cubic_bezier(t, pts):
    return numpy.multiply.outer(numpy.power(numpy.subtract(1, t), 3), pts[0,:]) + \
        numpy.multiply.outer(3*t*numpy.power(numpy.subtract(1,t),2),  pts[1,:]) + \
        numpy.multiply.outer(3*numpy.subtract(1,t)*numpy.power(t,2),  pts[2,:]) + \
        numpy.multiply.outer(numpy.power(t,3),                        pts[3,:])

if __name__ == '__main__':
    (speeds, times, distances) = ke_ss_lengths()

    pylab.clf()

    pylab.show()



