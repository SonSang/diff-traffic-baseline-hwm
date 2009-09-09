#!/usr/bin/python

import math
import numpy
import bisect
import random

testI = sorted([random.randint(100, 1000) for x in xrange(0, 2500)])

def size(I, m):
    return max(sum(I)/m, max(I))

def eps_makespan(I, m, eps):
    print I
    lower = size(I, m)
    upper = 2*lower
    while abs(upper - lower) > 1e-1:
        d = (upper+lower)/float(2)
        part = dual([item/float(d) for item in I], eps)
        if len(part) > m:
            lower = d
        else:
            upper = d
    part = dual([item/float(upper) for item in I], eps)
    print upper/size(I, m)
    return [ [ t * upper for t in partpart] for partpart in part ]

def partition(seq, split):
    left = []
    right = []
    for i in seq:
        if i <= split:
            left.append(i)
        else:
            right.append(i)
    return (left, right)

def eps_histogram(seq, eps):
    nbins = int(math.ceil(1.0/eps**2.0))
    h = (1.0 - eps)/nbins
    bins = []
    for i in xrange(0, nbins):
        bins.append(0)
    for i in seq:
        bins[int(math.ceil((i - eps)/h)) - 1] += 1
#    for ct in xrange(0, nbins):
#        print "(",eps+ ct*h, ",", eps + (ct+1)*h, "], ",
#    print " "
    return bins

def dual(I, eps):
    return fifth_approx(I)

def feasiblep(config, eps, h):
    s = 0
    for (ct, x) in enumerate(config):
        s += x*(eps + h*ct)
        if s > 1.0:
            return false
    return true

class minbins(object):
    def __init__(self, items, eps, h):
        self.items = items
        self.eps = eps
        self.h = h
        self.bin_table = numpy.empty( self.items, numpy.int32 )
        self.bin_table[:] = -1
        self.bin_table[numpy.zeros(len(self.items), numpy.int32)] = 0
    def all_feasible(config, bin_size = 1):
        pass

def L(I, items):
    result = []
    for i in reversed(items):
        if(len(result) > 0):
            end = result[-1]
        else:
            end = len(I)
        position = bisect.bisect_right(I, i, 0, end) - 1
        if position >= 0:
            result.append(position)
        else:
            return []
    result.reverse()
    return result

def fifth_approx(inI):
    small, I = partition(inI, 0.2)
    bins = []

    # "Stage 1" - >= 0.6
    while I and I[-1] >= 0.6:
        p = I.pop()
        l = L(I, [1.0 - p])
        if l:
            bins.append([p, I[l[0]]])
            del I[l[0]]
        else:
            bins.append([p])
    # print "Stage 1:"
    # print "Bins: ", bins
    # print "I ", I

    # "Stage 2"
    while I and I[-1] >= 0.5 and I[-2] >= 0.5:
        n0 = I.pop()
        n1 = I.pop()
        bins.append([n0, n1])
    if I and I[-1] >= 0.5:
        pick = random.randint(0,len(I))
        if pick == 0:
            n0 = I.pop()
            bins.append([n0])
        if pick == 2:
            l = L(I, [0.3, 0.4])
            if l:
                n0 = I.pop()
                bins.append([n0] + [I[x] for x in l])
                for x in reversed(l):
                    del I[x]
            else:
                pick = 1
        if pick == 1:
            n0 = I.pop()
            n1 = I.pop()
            bins.append([n0, n1])

    # print "Stage 2:"
    # print "Bins: ", bins
    # print "I ", I

    # "Stage 3"
    l = L(I, [0.3, 0.4, 0.5])
    while l and I[l[-1]] > 0.4:
        bins.append([I[x] for x in l])
        for x in reversed(l):
            del I[x]
        l = L(I, [0.3, 0.4, 0.5])

    # print "Stage 3:"
    # print "Bins: ", bins
    # print "I ", I

    # "Stage 4"
    while len(I) > 1 and I[-1] >= 0.4:
        n0 = I.pop()
        n1 = I.pop()
        bins.append([n0, n1])

    # print "Stage 4:"
    # print "Bins: ", bins
    # print "I ", I

    # "Stage 5"
    while I:
        three_bins = (I[0] > 0.25)
        if not three_bins:
            delta = 0.25 - I[0]
            l = L(I, [0.25 + delta/3.0, 0.25 + delta, 0.25 + 3*delta])
            if l and not 0 in l:
                bins.append([I[0]] + [I[x] for x in l])
                for i in reversed(l):
                    del I[i]
                del I[0]
            else:
                three_bins = True

        if three_bins:
            while I:
                if len(I) > 2:
                    bins.append([I[-3], I[-2], I[-1]])
                    del I[-1]
                    del I[-1]
                    del I[-1]
                elif len(I) > 1:
                    bins.append([I[-2], I[-1]])
                    del I[-1]
                    del I[-1]
                else:
                    bins.append([I[-1]])
                    del I[-1]

    # print "Stage 5:"
    # print "Bins: ", bins
    # print "I ", I

    bins.sort(key=sum)
    bins.reverse()
    start = 0
    for ct, bi in enumerate(bins):
        if sum(bi) < 1:
            start = ct
            break

    for sm in small:
        if start == len(bins):
            bins.append([sm])
        else:
            bins[start].append(sm)
        if sum(bins[start]) > 1:
            start += 1
    return bins

if __name__ == '__main__':
    print [(len(x), sum(x)) for x in eps_makespan(testI, 15, 0.0)]
