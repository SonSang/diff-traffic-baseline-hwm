import sys
import random
import itertools as it

def okaylanes(lanemap, time):
    ok = []
    for num, l_time in lanemap.items():
        if l_time == -1 or time - l_time > 1:
            ok.append(num)
    return ok

def generate_route(file, ncars, time_sep, vel_range=(10,14), lanelist=xrange(6)):
    out = open(file, 'w')
    out.write("<routes>\n")

    lanelast = dict( it.izip(lanelist, it.repeat(-1)) )
    assert len(lanelast) > 0
    N = ncars
    time = 0
    for i in xrange(0, N):
        valid_lanes = okaylanes(lanelast, time)
        while len(valid_lanes) == 0:
            time += 1
            valid_lanes = okaylanes(lanelast, time)
        departlane = valid_lanes[random.randint(0, len(valid_lanes)-1)]
        departspeed = random.uniform(*vel_range)

        lanelast[departlane] = time

        out.write("<vehicle id=\"0_%02d\" depart=\"%d\" departlane=\"%d\" departspeed=\"%d\"/>\n" % (i, time,
                                                                                                    departlane,
                                                                                                    departspeed))
        time += random.randint(*time_sep)

    out.write("</routes>")
    out.close()
    return time

if __name__ == '__main__':
    clovertime = generate_route("/home/sewall/unc/traffic/hwm/data/clover.rou.xml",
                                10000,
                                (0, 0),
                                lanelist=xrange(5, 21))

    print "Clover end time: ", clovertime

# 0  : lane-ne-ramp
# 1  : lane-nw-ramp
# 2  : lane-sw-ramp
# 3  : lane-se-ramp
# 4  : loops
# 9  : lane-ew-0
# 10 : lane-ew-1
# 11 : lane-ew-2
# 12 : lane-ew-3
# 17 : lane-ns-0
# 18 : lane-ns-1
# 19 : lane-ns-2
# 20 : lane-ns-3
# 13 : lane-sn-0
# 14 : lane-sn-1
# 15 : lane-sn-2
# 16 : lane-sn-3
# 5  : lane-we-0
# 6  : lane-we-1
# 7  : lane-we-2
# 8  : lane-we-3

