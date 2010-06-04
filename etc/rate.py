import numpy
from matplotlib import pyplot
from math import exp

def bump(x):
    if abs(x) < 1.0:
        return exp(-1.0/(1.0-x*x))
    else:
        return 0.0

def tod_car_rate(t):
    MORNING_RUSH_PEAK_T = 8.5*60*60.0
    EVENING_RUSH_PEAK_T = 17.5*60*60.0
    RUSH_DURATION       = 1.5*60*60.0

    BASE_RATE = 0.05
    PEAK_RATE = 0.2-BASE_RATE

    return bump((t-MORNING_RUSH_PEAK_T)*2/RUSH_DURATION)*PEAK_RATE \
           + bump((t-EVENING_RUSH_PEAK_T)*2/RUSH_DURATION)*PEAK_RATE + BASE_RATE

if __name__ == '__main__':
    s = numpy.linspace(0, 24*60*60, 1000)
    z = numpy.zeros(len(s))
    for (c, i) in enumerate(s):
        z[c] = tod_car_rate(i)
    pyplot.plot(s, z)
    pyplot.show()

