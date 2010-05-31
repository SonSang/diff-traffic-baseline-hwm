#!/usr/bin/python

import sys
import csv
import re

class run_record_origin(object):
    __slots__ = ["data_file",
                 "simulation_iterations",
                 "threads",
                 "total_time"]
    def __init__(self, lst):
        setattr(self, self.__slots__[0], lst[0])
        for (s,i) in zip(self.__slots__[1:3], lst[1:3]):
            setattr(self, s, int(i))
        for (s,i) in zip(self.__slots__[3:], lst[6:]):
            setattr(self, s, float(i))
    def __repr__(self):
        return "%s\t%d\t%d % 5.3f" % (self.data_file, self.threads, self.simulation_iterations, self.total_time)

class cmp_attr_chain:
    def __init__(self, attrs):
        self.attrs = attrs
    def __call__(self, x, y):
        for thisat in self.attrs:
            r = cmp(getattr(x, thisat), getattr(y, thisat))
            if r != 0:
                return r
        return 0

def best_records(reclist):
    "Should be sorted"
    res = []
    current_best     = reclist[0]
    for r in reclist[1:]:
        if r.threads != current_best.threads:
            res.append(current_best)
            current_best = r
        elif current_best.total_time > r.total_time:
            current_best = r
    res.append(current_best)
    return res

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print >> sys.stderr, "usage: %s <csvfile>" % sys.argv[0]
        sys.exit(1)
    for logfile in sys.argv[1:]:
        reader = csv.reader(open(logfile, "r"))
        grids = {}
        header = reader.next()
        print logfile
        print '-'*len(logfile)

        cac = cmp_attr_chain(["threads"])
        for row in reader:
            rec = run_record_origin(row)
            try:
                grids[rec.data_file].append(rec)
            except KeyError:
                grids[rec.data_file] = [rec]

        print "data_file threads  simulation_iterations total_time thread_scaling"
        print '@'*70
        for i in grids.keys():
            print i
            print '-' * len(i)
            grids[i].sort(cac)
            br = best_records(grids[i])
            base = br[0].total_time
            for line in br:
                print line, "\t% 5.3f" % (base/line.total_time,)
            print '*'*70
        print '@'*70
