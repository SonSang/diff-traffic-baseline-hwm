import math

def circle2(radius, ang_range, nsteps, center, direction):
    #direction = -1 # cw
    ra2 = [ math.fmod(x + 2*math.pi, 2*math.pi) for x in ang_range ]
    if direction == 1:
        dist = ra2[1] - ra2[0]
    else:
        dist = (2*math.pi-ra2[1]) + (2*math.pi-ra2[0])
    for i in xrange(nsteps):
        ang = direction*(ra2[0] + dist*i/float(nsteps-1))
        print radius*math.cos(ang) + center[0], radius*math.sin(ang) + center[1], 0.0, 0.0
