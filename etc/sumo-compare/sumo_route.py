import sys
import random

def okaylanes(lanemap, time):
    ok = []
    for (lno, i) in enumerate(lanemap):
        if i == -1 or i - time > 1:
            ok.append(lno)
    if len(ok) == 0:
        return range(len(lanemap))
    else:
        return ok

if __name__ == '__main__':
    out = open("/home/sewall/unc/traffic/hwm/etc/sumo-compare/sumo-data/fout2.rou.xml", 'w')
    out.write("<routes>\n")

    nlanes = 6
    lanelast = []
    for i in xrange(nlanes):
        lanelast.append(-1)

    N = 1000000
    time = 0
    for i in xrange(0, N):
        valid_lanes = okaylanes(lanelast, time)
        departlane = valid_lanes[random.randint(0, len(valid_lanes)-1)]
        arrivallane = random.randint(0, nlanes-1)
        departspeed = random.uniform(10, 14)

        lanelast[departlane] = time

        out.write("<vehicle id=\"0_%02d\" depart=\"%d\" departlane=\"%d\" arrivallane=\"%d\" departspeed=\"%d\">\n" % (i, time,
                                                                                                                       departlane,
                                                                                                                       arrivallane,
                                                                                                                       departspeed))

        time += random.randint(1, 2)

        out.write("<route edges=\"e0 e1 e2\"/></vehicle>\n")

    out.write("</routes>")
    out.close()
    print time
