#!/usr/bin/python

from matplotlib import rc
import numpy.numarray as na
import pylab
import itertools as it
import operator as op
import sys
import re
import math
import os.path

def scale_line(grids, allthreads):
    ngrids = len(grids)
    gridnames = [os.path.splitext(os.path.basename(x[0]))[0] for x in grids]
    colors = it.cycle(("#E6DA73", "#507EA1", "#CF6795"))
    symbols = it.cycle(("o-", "s-", "D-", "v-"))

    ymax = max(allthreads[-1], max((max(l[2]) for l in grids)))

    xpos = ( g[1] for g in grids)
    ypos = ( g[2] for g in grids)

    z = it.izip(xpos, ypos, symbols, colors)
    plots = []
    for i in z:
        d = dict()# zip(('color'),
                  #    i[3:4]))
        plots.append(pylab.plot(*i[:3], **d))
    pylab.hold(True)
    plots.append(pylab.plot(allthreads, allthreads, '--', color='black'))
    pylab.legend(plots, gridnames + ["linear"], 'best')
    pylab.xticks(allthreads, [str(x) for x in allthreads])
    pylab.yticks(range(0, int(math.ceil(ymax*1.1)), 2))
    for yl in xrange(0, int(math.ceil(ymax*1.1)), 2):
        pylab.axhline(y=yl, color='#444444', alpha=0.5)
    pylab.xlim(allthreads[0], allthreads[-1])
    pylab.ylim(1, ymax*1.1)


plotfuncs = {'-l'     : scale_line,
             '--line' : scale_line}

def threadread(flag):
    datafiles = []
    allthreads = set()
    line = sys.stdin.readline()
    while line:
        header = line

        blank = sys.stdin.readline()

        slots = sys.stdin.readline().rstrip().split()

        blank = sys.stdin.readline()

        current_data = sys.stdin.readline().rstrip()
        blank = sys.stdin.readline()

        datafiles.append( (current_data, [], []) )

        line = sys.stdin.readline()

        while not re.compile("[\@]+").match(line, 1):
            if re.compile("[\*]+").match(line, 1):
                line = sys.stdin.readline()
                if re.compile("[\@]+").match(line, 1):
                    break
                current_grid = line.rstrip()
                datafiles.append( (current_data, [], []) )
                blank = sys.stdin.readline()
            else:
                data = line.rstrip().split()
                data = dict(zip(slots, data))

                thread = int(data['threads'])
                if thread == 1 or (thread % 2) == 0:
                    datafiles[-1][1].append(thread)
                    datafiles[-1][2].append(float(data['thread_scaling']))
            line = sys.stdin.readline()
        line = sys.stdin.readline()

    for g in datafiles:
        allthreads.update(g[1])
    allthreads = list(allthreads)
    allthreads.sort()

    plotfuncs[flag](datafiles, allthreads)

    pylab.xlabel('threads', fontsize=fsize)
    pylab.ylabel('speedup over 1 thread', fontsize=fsize)

readfuncs = {scale_line : threadread}

if __name__ == '__main__':
    if len(sys.argv) < 3:
        pfk = plotfuncs.items()
        pfk.sort(cmp=lambda x,y: cmp(x[1],y[1]) ^ cmp(x[0], y[1]))
        flist = [sorted(list(b), reverse=True) for a, b in it.groupby(pfk, key=lambda x: x[1])]
        flags = "[" + ']|['.join('|'.join([opt[0] for opt in func]) for func in flist) + ']'
        print "%s: %s <titlestr> [<outputfile>]" % (sys.argv[0], flags)
    else:
        fsize = 20
        readfuncs[plotfuncs[sys.argv[1]]](sys.argv[1])

        rc('text', usetex=True)
        pylab.title(sys.argv[2], fontsize=fsize)
        if(len(sys.argv) == 4):
            pylab.savefig(sys.argv[3])
        else:
            pylab.show()
        pylab.clf()
