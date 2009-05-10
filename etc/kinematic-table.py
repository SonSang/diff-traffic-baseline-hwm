import subprocess
import math
import pylab
import numpy

exe = './ss_kinematic'

car_length = 4.5
lane_width = 2.5
max_steering_angle = 7*math.pi/16.0
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
    exec_seq = [exe, 0, str(10/3.6), str(final_orient), str(car_length), str(max_steering_angle)]

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

        line = p.stderr.readline()
        line = line.strip().split()

        [time, distance] = [float(x) for x in line]
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

def cpoly(coeffs, var):
    res = str(coeffs[0]) + "*" + var + "+\n"
    for i in coeffs[1:-1]:
        res = "(" + res + str(i) + "*" + var + ") +\n"
    res = res + str(coeffs[-1]) + ";\n"
    return res

def cubic_bezier(t, pts):
    return numpy.multiply.outer(numpy.power(numpy.subtract(1, t), 3), pts[0,:]) + \
        numpy.multiply.outer(3*t*numpy.power(numpy.subtract(1,t),2),  pts[1,:]) + \
        numpy.multiply.outer(3*numpy.subtract(1,t)*numpy.power(t,2),  pts[2,:]) + \
        numpy.multiply.outer(numpy.power(t,3),                        pts[3,:])

if __name__ == '__main__':
    (speeds, times, distances) = ke_ss_lengths()
    m = (distances[-1]-distances[0])/(speeds[-1]-speeds[0])
    b =  distances[0]-m*speeds[0]
    print 'xend', cpoly([m, b],'speed')
    #speed vs x: [0.843112757647 2.45445917647]
#    p = numpy.poly1d([m, b])
#    pylab.clf()
#    pylab.plot(speeds, distances, speeds, p(speeds))
#    pylab.show()

    (speeds, times, distances) = ke_ss_lengths()
    m = (distances[-1]-distances[0])/(speeds[-1]-speeds[0])
    b =  distances[0]-m*speeds[0]
    print 'yend', cpoly([m, b],'speed')
    #speed vs y: [0.244331745882 4.11774005882]
#    p = numpy.poly1d([m, b])
#    pylab.clf()
#    pylab.plot(speeds, distances, speeds, p(speeds))
#    pylab.show()

    (idx, data) = ke_ss_path(16.6667, 10/3.6)

    end   = data[-1, idx['t']]
    ylast = data[-1, idx['y']]
    ycoeff = numpy.polyfit(numpy.divide(data[:, idx['t']],end), numpy.divide(data[:, idx['y']], ylast), 5)

    py = numpy.poly1d(ycoeff)

    xlast = data[-1, idx['x']]
    xcoeff = numpy.polyfit(numpy.divide(data[:, idx['t']],end), numpy.divide(data[:, idx['x']], xlast), 4)

    px = numpy.poly1d(xcoeff)

    pylab.clf()
#    pylab.plot(data[:, idx['t']], data[:, idx['y']], data[:, idx['t']], py(numpy.divide(data[:, idx['t']], end)))
#    pylab.plot(data[:, idx['t']], data[:, idx['x']], data[:, idx['t']], px(numpy.divide(data[:, idx['t']], end)))

    newt = numpy.linspace(0, 1, 100)

    speed = 16.6667

    (idx, data) = ke_ss_path(speed, 10/3.6)


    endx = 0.843112757647*speed + 2.45445917647
    endy = 0.244331745882*speed + 4.11774005882

#    pylab.plot(data[:, idx['x']], data[:, idx['y']], endx*px(newt),
#               endy*py(newt))
#    pylab.show()

    print cpoly(xcoeff, 't')
    print cpoly(ycoeff, 't')



