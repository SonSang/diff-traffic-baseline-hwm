import math

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

