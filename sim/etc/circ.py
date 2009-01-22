import math

def circle2(radius, ang_range, nsteps, center):
    for i in xrange(nsteps):
        ang = ang_range[0] + (ang_range[1]-ang_range[0])*i/float(nsteps-1)
        print radius*math.cos(ang) + center[0], radius*math.sin(ang) + center[1]



