import math
import numpy
import pylab
import matplotlib

def circle(center, radius, angint, ccw, nsteps):
    angint2 = [ math.fmod(x + 2*math.pi, 2*math.pi) for x in angint ]
    inorder = angint2[0] <= angint2[1]
    adist = abs(angint2[1] - angint2[0])
    if (not ccw and inorder) or (ccw and not inorder):
        adist = 2*math.pi - adist
    if not ccw:
        adist *= -1
    steps = ( math.fmod(angint2[0] + (adist)*x/float(nsteps-1), 2*math.pi) for x in xrange(nsteps) )
    return ( (radius*math.cos( theta ) + center[0], radius*math.sin( theta ) + center[1]) for theta in steps )

def ext_tangents(c0, r0, c1, r1):
    if(r0 > r1):
        r0, r1 = r1, r0
        c0, c1 = c1, c0

    delr = r1 - r0
    delc = c1 - c0
    delc_theta = math.pi/2 - math.atan2(delc[1], delc[0])
    delc_len = math.sqrt(numpy.dot(delc, delc))
    theta = math.atan2(delr, delc_len)
    z0 = math.pi/2 - theta + delc_theta
    l0 = (r0*numpy.array([math.cos(z0), math.sin(z0)]) + c0,
          r1*numpy.array([math.cos(z0), math.sin(z0)]) + c1)
    z1 = -(math.pi/2 - theta) + delc_theta
    l1 = (r0*numpy.array([math.cos(z1), math.sin(z1)]) + c0,
          r1*numpy.array([math.cos(z1), math.sin(z1)]) + c1)
    return (z0, l0), (z1, l1)

def ramp(p0, n0, p1, n1, rad):
    c0 = rad*numpy.array([n0[1], -n0[0]]) + p0
    c1 = rad*numpy.array([n1[1], -n1[0]]) + p1
    print c0, c1

if __name__ == '__main__':
    c0 = numpy.array([70, 10])
    r0 = 10
    c1 = numpy.array([10, 70])
    r1 = 20
    (l0, l1) =  ext_tangents(c0, r0, c1, r1)
    print l0, l1
    pylab.clf()
    ax = pylab.subplot(111)
    circle = matplotlib.patches.Circle(c0, r0, ec='none', facecolor='#00ff00', fill=True)
    ax.add_patch(circle)
    circle = matplotlib.patches.Circle(c1, r1, ec='none', facecolor='#ff0000', fill=True)
    ax.add_patch(circle)

    pl0 = matplotlib.lines.Line2D((l0[0][0], l0[1][0]),
                                  (l0[0][1], l0[1][1]),
                                  color='blue', linewidth=1.0 )
    ax.add_line(pl0)
    pl1 = matplotlib.lines.Line2D((l1[0][0], l1[1][0]),
                                  (l1[0][1], l1[1][1]),
                                  color='blue', linewidth=1.0 )
    ax.add_line(pl1)

    ax.axis('auto')
    ax.axis('equal')

    pylab.show()


    # ramp(numpy.array([70, 10]),
    #      numpy.array([-1, 0]),
    #      numpy.array([10, 70]),
    #      numpy.array([0, 1]),
    #      20)
