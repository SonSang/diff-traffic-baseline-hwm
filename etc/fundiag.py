#!/usr/bin/python

from matplotlib import rc
import numpy as np
import pylab
import itertools as it
import operator as op
import sys
import re
import math

umax = 1.0
gamma = 0.5

def to_u(rho, y):
    return y/rho + ueq(rho)

def to_y(rho, u):
    return rho*(u - ueq(rho))

def lambda0(rho, y):
    u = to_u(rho, y)
    return u + rho*ueq_prime(rho)

def lambda1(rho, y):
    u = to_u(rho, y)
    return u

def ueq(rho):
    return umax*(1.0-rho**gamma)

def ueq_prime(rho):
    return -umax*gamma*rho**(gamma-1)

def invariant0(rho, y):
    u = to_u(rho, y)
    return u - ueq(rho)

def invariant1(rho, y):
    u = to_u(rho, y)
    return u

def do_i0(wl):
    x = np.arange(0.0, 1.0, 0.01)
    y = np.ones(len(x))
    for i in xrange(0, len(x)):
        y[i] = x[i]*(wl + ueq(x[i]))
    return (x,y)

def critval(wl):
    return ((wl + umax)/(umax*(gamma+1.0)))**(1.0/gamma)

if __name__ == '__main__':
    I0 = -0.5

    (a, b) = do_i0(I0)

    critical = critval(I0)

    print critical
    cmax = critical*(I0 + ueq(critical))
    print cmax

    fsize = 20
    rc('text', usetex=True)
    pylab.plot(a,b, color='orange', label='FD')

    pylab.axvline(critical, color='blue')
    pylab.axhline(cmax, color='blue')


#    pylab.plot(a,b, color='orange', label='0-Invariant')
#    pylab.plot(a0,b0, color='green', label='1-Invariant')
#
#    pylab.plot([q_l[0]], [to_u(*q_l)], 'o', color='red', label='_nolegend_')
#    pylab.text(q_l[0]+0.02, to_u(*q_l)-0.02, 'q_l', ha='left', va='top', size=fsize)
#
#    pylab.plot([q_r[0]], [to_u(*q_r)], 's', color='blue', label='_nolegend_')
#    pylab.text(q_r[0]+0.02, to_u(*q_r)-0.02, 'q_r', ha='left', va='top', size=fsize)
#
#    pylab.plot([0.0], [to_u(*q_l) - ueq(q_l[0]) + umax], 'x', color='blue', label='_nolegend_')
#    pylab.text(0.0+0.02, to_u(*q_l) - ueq(q_l[0]) + umax-0.02, 'is', ha='left', va='top', size=fsize)
#

#    pylab.plot([q_m[0]], [q_m[1]], 'x', color='black', label='_nolegend_')
#    pylab.text(q_m[0]+0.02, q_m[1]-0.02, 'q_m', ha='left', va='top', size=fsize)

    pylab.axis([0, 1.0, 0.0, cmax+0.2])
#    pylab.xlabel(r'\rho', size=fsize)
#    pylab.ylabel(r'u', size=fsize)
#    pylab.legend(loc='best')
#    pylab.title(title, fontsize=fsize)
    if(len(sys.argv) == 2):
        pylab.savefig(sys.argv[1])
    else:
        pylab.show()
    pylab.clf()
