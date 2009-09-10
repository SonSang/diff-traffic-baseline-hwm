#!/usr/bin/python

import math
import numpy
import bisect
import random
import itertools as it

testI = sorted([random.randint(100, 1000) for x in xrange(0, 1000)])

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
    seq_bins = []
    for i in xrange(0, nbins):
        bins.append(0)
        seq_bins.append([])
    for i in seq:
        bin_no = int(math.ceil((i - eps)/h)) - 1
        bins[bin_no] += 1
        seq_bins[bin_no].append(i)
    return (bins, h, seq_bins)

def dual(I, eps):
##    small, large = partition(I, 0.2)
##    bins = fifth_approx(large)

    small, large = partition(I, eps)

    actual_bins = []
    if large:
        (hist, h, seq_hist) = eps_histogram(large, eps)
        hist_bins = minbins(hist, eps, h)
        for hb in hist_bins:
            actual_bins.append([])
            for pos, ct in enumerate(hb):
                for i in xrange(ct):
                    actual_bins[-1].append(seq_hist[pos].pop())

    return pack_small(actual_bins, small)

def feasiblep(config, eps, h):
    s = 0
    for (ct, x) in enumerate(config):
        s += x*(eps + h*ct)
        if s > 1.0:
            return false
    return true

def minbins(items, eps, h):
    bin_table = dict()
    todo = set(feasible(list(numpy.zeros(len(items), numpy.int32)), items, eps, h))
    for t0 in todo:
        bin_table[t0] = (1, None)
    while todo:
        t0 = todo.pop()
        for t1 in feasible(t0, items, eps, h):
            if not bin_table.has_key(t1) or 1 + bin_table[t0][0] < bin_table[t1][0]:
                bin_table[t1] = (1 + bin_table[t0][0], t0)
                todo.add(t1)
    res = []
    last = tuple(items)
    current = bin_table[last][1]
    while current != None:
        res.append(tuple((i - j for i, j in it.izip(last, current))))
        last = current
        current = bin_table[last][1]
    res.append(last)
    return res

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

    return bins

def pack_small(bins, small):
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

def feasible_iterator(maxconfig):
    if len(maxconfig) == 0:
        yield []
    else:
        for i in xrange(0, maxconfig[0]):
            for sub in feasible_iterator(maxconfig[1:]):
                yield [i] + sub

def feasible(startconfig, maxconfig, base, h):
    left = 1.0
    possible_config = []
    for (i, j) in it.izip(maxconfig, startconfig):
        if i >= j:
            possible_config.append(i - j)
        else:
            return []
    fi = (tuple((i + j for i, j in it.izip(startconfig, x))) for x in feasible_iterator2(left,  possible_config, base, h))
    fi.next()
    return fi

def feasible_iterator2(left, maxconfig, base, h):
    if not maxconfig:
        yield []
    else:
        for i in xrange(maxconfig[0]+1):
            newleft = left - base*i
            if newleft >= -1e-4:
                for sub in feasible_iterator2(newleft, maxconfig[1:], base + h, h):
                    yield [i] + sub

if __name__ == '__main__':
#    print dual([0.25, 0.33, 0.34, 0.41, 0.55, 0.56], 0.2)
    print [(len(x), sum(x)) for x in eps_makespan(testI, 8, 0.2)]
