import subprocess
import math
import pylab
import numpy

exe = './kinematic'

car_length = 4.5
lane_width = 2.5
max_steering_angle = math.pi/4.0
resolution = 50

def ke_lengths():
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

def ke_path(speed):
    exec_seq = [exe, str(speed), str(lane_width), str(car_length), str(max_steering_angle), str(resolution)]

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
#    speeds = []
#    res = []
#    ends = []
#    for speed in numpy.linspace(5, 100, 20):
#        speed /= 3.6
#        (idx, data) = ke_path(speed)
#        coeffs = numpy.polyfit(data[:, idx['t']], data[:, idx['y']], 3)
#        speeds.append(speed)
#        res.append(coeffs.tolist())
#        ends.append(data[-1,idx['t']])

#    print 'data[][] = {' + ',\n'.join(['{' + ', '.join([str(x) for x in m]) + '}' for m in res]) + '}'
#    print 'idx[] = {' + ', '.join([str(x) for x in speeds]) + '}'
#    print 'limits[] = {' + ', '.join([str(x) for x in ends]) + '}'
#    speed = 1
#    (idx, data) = ke_path(speed)

#    coeffs = numpy.polyfit(data[:, idx['t']], data[:, idx['x']], 1)

#    print coeffs

    speed = 10
    speed /= 3.6
    (idx, data) = ke_path(speed)
    #coeff = numpy.polyfit(data[:data.shape[0]/2+1, idx['t']], data[:data.shape[0]/2+1, idx['y']], 3)

    end = data[-1,idx['t']]
    coeff = numpy.polyfit(numpy.divide(data[:data.shape[0], idx['t']],end), numpy.divide(data[:data.shape[0], idx['y']], lane_width), 5)

    print coeff
    poly = numpy.poly1d(coeff)

    print math.sqrt(speed)*end

    def f(x):
        return numpy.divide(math.sqrt(speeds)*end, numpy.power(x, 0.5))

    speed = 10
    speed /= 3.6
    (idx, data) = ke_path(speed)
    pylab.clf()
    pylab.plot(data[:, idx['t']], data[:, idx['y']])
#    pylab.plot(data[:data.shape[0]/2+1, idx['t']], poly(numpy.multiply(end/data[-1,idx['t']], data[:data.shape[0]/2+1, idx['t']])))
    pylab.plot(data[:data.shape[0], idx['t']], poly(numpy.multiply(1.0/data[-1,idx['t']], data[:data.shape[0], idx['t']])))
#    pylab.plot(numpy.subtract(data[-1,idx['t']],data[:data.shape[0]/2+1, idx['t']]), numpy.subtract(lane_width, poly(numpy.multiply(end/data[-1,idx['t']], data[:data.shape[0]/2+1, idx['t']]))))

    a = numpy.array([[0.0, 0.0], [0.55, 0.0], [0.63, 0.3], [1.0, 1.0]])
    scale_t = numpy.divide(data[:, idx['t']], data[-1, idx['t']])

    res = cubic_bezier(scale_t, a)
    res[:,1] *= lane_width*0.5
    res[:,0] *= data[-1, idx['t']]*0.5
#    pylab.plot(res[:,0], res[:,1])
#    pylab.plot(numpy.subtract(data[-1, idx['t']],res[:,0]), numpy.subtract(lane_width, res[:,1]))
    pylab.show()

#    pylab.plot(speeds, ends)
#    pylab.plot(speeds, f(speeds))

#    pylab.subplot(411)
#    pylab.ylabel('theta')
#    pylab.plot( data[:, idx['t']], data[:, idx['theta']])
#
#    pylab.subplot(412)
#    pylab.ylabel('x')
#    pylab.plot( data[:, idx['t']], data[:, idx['x']])
#
#    pylab.subplot(413)
#
#    pylab.ylabel('y')
#    pylab.xlabel('t')
#    pylab.plot( data[:, idx['t']], data[:, idx['y']])
#
#    pylab.subplot(414)
#    pylab.ylabel('y')
#    pylab.xlabel('x')
#    pylab.plot( data[:, idx['x']], data[:, idx['y']])

#    pylab.show()

#    (speeds, times, distances) = ke_lengths()
#
#    coeffs = numpy.polyfit(speeds, distances, 4)
#    print coeffs
#    p     = numpy.poly1d(coeffs)
#
#    pylab.clf()
#    pylab.plot(speeds, distances, speeds, p(speeds))
#    pylab.plot(speeds, distances)
#    pylab.show()





