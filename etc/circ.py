import math
import numpy
import pylab
import matplotlib
import scipy.linalg

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

def min_arc(a0, a1):
    a0 = math.fmod(a0 + 2*math.pi, 2*math.pi)
    a1 = math.fmod(a1 + 2*math.pi, 2*math.pi)
    if(a0 > a1):
        a0, a1 = a1, a0
    if(2*math.pi-a1 + a0 < a1 - a0):
        a0, a1 = a1, a0
    return a0, a1

def tan_circ(plist, radius=None):
    p    = plist[1]
    back = plist[0] - plist[1]
    fwd  = plist[2] - plist[1]
    blen = scipy.linalg.norm(back)
    back /= blen
    flen = scipy.linalg.norm(fwd)
    fwd  /= flen

    det = back[1]*fwd[0] - back[0]*fwd[1]

    if(det > 0):
        back_t = numpy.array([ back[1], -back[0]])
        fwd_t  = numpy.array([ -fwd[1],  fwd[0]])
    elif det < 0:
        back_t = numpy.array([-back[1], back[0]])
        fwd_t  = numpy.array([  fwd[1], -fwd[0]])
    else:
        return (plist[2], None, None, None, None, plist[2])

    if not radius:
        radius = min(blen, flen) * (1-numpy.dot(fwd, back))/math.sqrt(1-numpy.dot(fwd, back)**2)
        alpha = min(blen, flen)
    else:
        alpha = radius*(fwd_t[0]-back_t[0])/(back[0] - fwd[0])
        # alpha = radius*(1+numpy.dot(fwd, back))/math.sqrt(1-numpy.dot(fwd, back)**2)

    # d0 = 2*radius/math.sqrt(1-numpy.dot(fwd, back))
    #print d0*math.sqrt(1+numpy.dot(fwd, back))/2

    angle0 = math.atan2(-back_t[1], -back_t[0])#*180.0/math.pi
    angle1 = math.atan2( -fwd_t[1],  -fwd_t[0])#*180.0/math.pi

    angle0, angle1 = min_arc(angle0, angle1)

    center0 = alpha * back + radius*back_t + p
    return (alpha * back + p, angle0, center0, radius, angle1, alpha * fwd + p)

def run_tan_circ(pts, debug=False):
    lastpt = pts[0]
    res = [lastpt]
    # pylab.clf()
    # ax = pylab.subplot(111)
    for idx in xrange(1, len(pts)-1):
        rad = 200 - (4  + 3/2)*3.62
        (p0, angle0, center, thisrad, angle1, p1) = tan_circ(pts[idx-1:idx+2], rad)

        if center == None:
            continue

#        res.append(p0)
        res += circle(center, thisrad, (angle1, angle0), False, 6)

        # linedata = zip(lastpt, p0)
        # l0 = matplotlib.lines.Line2D(linedata[0], linedata[1], color='black', linewidth=2.0 )
        # ax.add_line(l0)

        # if debug:
        #     linedata = zip(pts[idx-1], pts[idx])
        #     l1 = matplotlib.lines.Line2D(linedata[0], linedata[1], color='blue', linewidth=1.0 )
        #     ax.add_line(l1)

        # if debug:
        #     cshape = matplotlib.patches.Cshape(center, thisrad, ec='none', facecolor='#00ff00', fill=True)
        #     ax.add_patch(cshape)
        # cshape = matplotlib.patches.Arc(center, 2*thisrad, 2*thisrad, 0.0, angle0*180/math.pi, angle1*180/math.pi, edgecolor='black', lw=2.0, fill=False)
        # ax.add_patch(cshape)

        # lastpt = p1

    # linedata = zip(lastpt, pts[-1])
    # l0 = matplotlib.lines.Line2D(linedata[0], linedata[1], color='black', linewidth=2.0)
    # ax.add_line(l0)
    res.append(pts[-1])

    # if debug:
    #     linedata = zip(pts[-2], pts[-1])
    #     l1 = matplotlib.lines.Line2D(linedata[0], linedata[1], color='blue', linewidth=1.0)
    #     ax.add_line(l1)

    # ax.axis('auto')
    # ax.axis('equal')

    # pylab.show()
    return res

def ramp(p0, n0, p1, n1, rad):
    pass

def ne_circle(center, rad, nsteps):
    ne_circle_center = center
    ne_circle_ang = (math.pi, 3*math.pi/2)
    ne_circle_ccw = False

    res = circle(ne_circle_center,
                 rad,
                 ne_circle_ang,
                 ne_circle_ccw,
                 nsteps)
    for (ct, (x,y)) in enumerate(res):
        print "%+8.5f %+8.5f %+8.5f 0.0" %  (x, y, 1.0-ct/float(nsteps-1))

def se_circle(center, rad, nsteps):
    se_circle_center = (center[0], -center[1])
    se_circle_ang = (math.pi/2, math.pi)
    se_circle_ccw = False

    res = circle(se_circle_center,
                 rad,
                 se_circle_ang,
                 se_circle_ccw,
                 nsteps)
    for (ct, (x,y)) in enumerate(res):
        print "%+8.5f %+8.5f %+8.5f 0.0" %  (x, y, ct/float(nsteps-1))

def nw_circle(center, rad, nsteps):
    nw_circle_center = (-center[0], center[1])
    nw_circle_ang = (3*math.pi/2, 0)
    nw_circle_ccw = False

    res = circle(nw_circle_center,
                 rad,
                 nw_circle_ang,
                 nw_circle_ccw,
                 nsteps)
    for (ct, (x,y)) in enumerate(res):
        print "%+8.5f %+8.5f %+8.5f 0.0" %  (x, y, ct/float(nsteps-1))

def sw_circle(center, rad, nsteps):
    sw_circle_center = (-center[0], -center[1])
    sw_circle_ang = (0, math.pi/2)
    sw_circle_ccw = False

    res = circle(sw_circle_center,
                 rad,
                 sw_circle_ang,
                 sw_circle_ccw,
                 nsteps)
    for (ct, (x,y)) in enumerate(res):
        print "%+8.5f %+8.5f %+8.5f 0.0" %  (x, y, 1.0-ct/float(nsteps-1))

if __name__ == '__main__':
    neramppts = numpy.array([[500.0, 18.1], [325.0, 18.1], [18.1, 325.0], [18.1, 500.0]], dtype=numpy.float32)
    nwramppts = numpy.array([[-18.1, 500], [-18.1, 325.0], [-325.0, 18.1], [-500, 18.1]], dtype=numpy.float32)
    swramppts = numpy.array([[-500, -18.1], [-325.0, -18.1], [-18.1, -325.0], [-18.1, -500]], dtype=numpy.float32)
    seramppts = numpy.array([[18.1, -500], [18.1, -325.0], [325.0, -18.1], [500, -18.1]], dtype=numpy.float32)

    res = run_tan_circ(seramppts)
    for (ct, (x,y)) in enumerate(res):
        print "%+8.5f %+8.5f %+8.5f 0.0" %  (x, y, 1.0-ct/float(nsteps-1))

    # circle_radius = 81.9
    # nsteps = 24
    # center = (100, 100)

    # sw_circle(center, circle_radius, nsteps)

    # pylab.clf()
    # ax = pylab.subplot(111)
    # for i in xrange(1, len(res)):
    #     linedata = zip(res[i-1], res[i])
    #     l0 = matplotlib.lines.Line2D(linedata[0], linedata[1], color='black', linewidth=2.0)
    #     ax.add_line(l0)

    #     print res[i-1], res[i]

    # ax.axis('auto')
    # ax.axis('equal')

    # pylab.show()
