import math
import numpy
import pylab
import matplotlib
import bisect
import random

car_length = 4.5

class pc_data(object):
    def __init__(self, dx, data, inf):
        self.dx = dx
        self.inf = inf
        self.n = len(data)
        self.build(data)
    def __iter__(self):
        def it(start, obj):
            i = start
            while i < obj.n:
                yield obj[i]
                i += 1
        return it(0, self)
    def end(self):
        return self.n*self.dx
    def build(self, data):
        self.integration = numpy.zeros((self.n+1))
        for i in xrange(1, len(self.integration)):
            self.integration[i] = self.integration[i-1] + data[i-1]*self.dx
    def __getitem__(self, idx):
        if(idx >= self.n):
            return self.inf
        return self.integration[idx+1] - self.integration[idx]
    def integrate(self, t):
        if t >= self.n*self.dx:
            local = t - self.n*self.dx
            return self.integration[-1] + local*self.inf
        else:
            scaled = t/self.dx
            cell = math.floor(scaled)
            local = scaled - cell
            return (1.0-local)*self.integration[cell] + local*self.integration[cell+1]
    def inv_integrate(self, v):
        if v < self.integration[0]:
            raise Exception("Value is beyond range of integral!")
        if v > self.integration[-1]:
            last_v = self.integration[-1]
            t = (v-last_v)/self.inf + self.n*self.dx
        else:
            idx = bisect.bisect_right(self.integration, v)
            last_v = self.integration[idx-1]
            t = (v-last_v)/self[idx-1] * self.dx + (idx-1) * self.dx
        return t
    def plot(self, ax, data, integrate, extra):
        assert extra >= self.n*self.dx
        if data:
            pl = numpy.zeros((2, self.n*2+2))
            for i in xrange(0, self.n*2):
                if i % 2 == 0:
                    xval = i/2 * self.dx
                else:
                    xval = (i/2 + 1) * self.dx
                pl[0][i] = xval
                pl[1][i] = self[i/2]

            pl[0][-2] = self.n*self.dx
            pl[1][-2] = self.inf
            pl[0][-1] = extra
            pl[1][-1] = self.inf

            ax.plot(pl[0], pl[1])
        if integrate:
            x = numpy.linspace(0, self.dx*self.n, self.n+1)
            ax.plot(list(x)+[extra], list(self.integration)+[self.integration[-1] + self.inf*(extra-x[-1])])
        return ax

def pc_func(func, dx, n):
    data = numpy.zeros((n))
    x = 0
    for i in xrange(0, n):
        data[i] = 0.5*(func(x) + func(x+dx))
        x += dx
    return pc_data(dx, data, func(x))

def pc_avg(places, dx, n, fac=1):
    data = numpy.zeros((n))
    inv_dx = 1.0/dx
    for p in places:
        data[int(math.floor(p*inv_dx))] += fac
    return pc_data(dx, data, 0)

def plot_inv_web(ax, pc, y):
    x = pc.inv_integrate(y)
    l0 = matplotlib.lines.Line2D((0, x), (y, y), color='black', linewidth=1.0)
    l1 = matplotlib.lines.Line2D((x, x), (0, y), color='black', linewidth=1.0)
    ax.add_line(l0)
    ax.add_line(l1)
    return ax

def plot_web(ax, pc, x):
    y = pc.integrate(x)
    l0 = matplotlib.lines.Line2D((0, x), (y, y), color='black', linewidth=1.0)
    l1 = matplotlib.lines.Line2D((x, x), (0, y), color='black', linewidth=1.0)
    ax.add_line(l0)
    ax.add_line(l1)
    return ax

def exp_rvar(p):
    return -math.log(p)

def ih_poisson(start, limit, pc):
    t = start
    arg = pc.integrate(t)
    while True:
        U = random.random()
        E = exp_rvar(U)
        # t = pc.inv_integrate(E + pc.integrate(t))
        arg += E
        t = pc.inv_integrate(arg)
        if t > limit:
            return
        yield t

def show_poisson_proc(ax, start_end, pc):
    ax = pc.plot(ax, True, False, max(start_end[1], pc.end()))

    gen = ih_poisson(start_end[0], start_end[1], pc)
    res = list(gen)
    maxy = max(max(pc), pc.inf)
    ax = plot_events(ax, res, maxy)

    return ax

def plot_events(ax, res, height):
    for i in res:
        l0 = matplotlib.lines.Line2D((i, i), (0, height), color='black', linewidth=1.0)
        ax.add_line(l0)
    return ax

if __name__ == '__main__':
    i = pc_func(lambda x: 1/car_length*1.1*(math.cos(x)+1), 10.0*car_length, 10)
    # i = pc_func(lambda x: 1/car_length, car_length, 1)
    random.seed(1994)
    # gen = list(ih_poisson(0, i.end(), i))
    pylab.clf()
    ax = pylab.axes()
    ax = i.plot(ax, True, True, i.end()+200)
    # ax = plot_events(ax, gen, max(i))

    # i2 = pc_avg(gen, 10.0*car_length, 10, 1.0/car_length)
    # ax = i2.plot(ax, True, False, i2.end())
    # # ax = show_poisson_proc(pylab.axes(), (0, i.end()), i)
    plot_web(ax, i, 100.0)
    plot_web(ax, i, 400.0)
    plot_web(ax, i, 250.0)
    plot_inv_web(ax, i, 80.0)
    plot_inv_web(ax, i, 110.0)

    # # ax.axis('equal')
    pylab.show()
