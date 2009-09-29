import sys
import random

def okaylanes(lanemap, time):
    ok = []
    for (lno, i) in enumerate(lanemap):
        if i == -1 or time - i > 1:
            ok.append(lno)
    return ok

def generate_route(file, ncars, time_sep, vel_range=(10,14), nlanes=6):
    out = open(file, 'w')
    out.write("<routes>\n")

    lanelast = []
    for i in xrange(nlanes):
        lanelast.append(-1)

    N = ncars
    time = 0
    for i in xrange(0, N):
        valid_lanes = okaylanes(lanelast, time)
        while len(valid_lanes) == 0:
            time += 1
            valid_lanes = okaylanes(lanelast, time)
        departlane = valid_lanes[random.randint(0, len(valid_lanes)-1)]
        arrivallane = random.randint(0, nlanes-1)
        departspeed = random.uniform(*vel_range)

        lanelast[departlane] = time

        out.write("<vehicle id=\"0_%02d\" depart=\"%d\" departlane=\"%d\" arrivallane=\"%d\" departspeed=\"%d\">\n" % (i, time,
                                                                                                                       departlane,
                                                                                                                       arrivallane,
                                                                                                                       departspeed))
        time += random.randint(*time_sep)

        out.write("<route edges=\"e0 e1 e2\"/></vehicle>\n")

    out.write("</routes>")
    out.close()
    return time

if __name__ == '__main__':
    sptime = generate_route("/home/sewall/unc/traffic/hwm/etc/sumo-compare/sparse/sparse.rou.xml",
                          1000000,
                          (7, 10))
    print "Sparse: ", sptime

    medtime = generate_route("/home/sewall/unc/traffic/hwm/etc/sumo-compare/medium/medium.rou.xml",
                             1000000,
                             (3, 6))
    print "Medium: ", medtime

    meddensetime = generate_route("/home/sewall/unc/traffic/hwm/etc/sumo-compare/med-dense/med-dense.rou.xml",
                               1000000,
                               (2, 3))
    print "Med-Dense: ", meddensetime

    densetime = generate_route("/home/sewall/unc/traffic/hwm/etc/sumo-compare/dense/dense.rou.xml",
                               1000000,
                               (1, 2))
    print "Dense: ", densetime

    megadensetime = generate_route("/home/sewall/unc/traffic/hwm/etc/sumo-compare/megadense/megadense.rou.xml",
                               1000000,
                               (0, 1))
    print "Mega dense: ", megadensetime
