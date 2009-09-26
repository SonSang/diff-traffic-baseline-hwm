from xml.dom.minidom import Document
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
    doc = Document()
    routes = doc.createElement('routes')
    doc.appendChild(routes)

    nlanes = 6
    lanelast = []
    for i in xrange(nlanes):
        lanelast.append(-1)

    N = 10000
    time = 0
    for i in xrange(0, N):
        v = doc.createElement('vehicle')
        routes.appendChild(v)
        v.setAttribute("id", "0_%02d" % i)
        v.setAttribute("depart", str(time))

        valid_lanes = okaylanes(lanelast, time)
        departlane = valid_lanes[random.randint(0, len(valid_lanes)-1)]
        lanelast[departlane] = time

        v.setAttribute("departlane",  str(departlane))
        v.setAttribute("arrivallane", str(random.randint(0, nlanes-1)))
        v.setAttribute("departspeed", str(random.uniform(10, 14)))

        time += random.randint(1, 2)

        r = doc.createElement('route')
        r.setAttribute("edges", "e0 e1 e2")
        v.appendChild(r)

    # print doc.toprettyxml(indent="   ")

    fi = open("/home/sewall/unc/traffic/hwm/etc/sumo-compare/sumo-data/fout2.rou.xml", 'w')
    fi.write(doc.toprettyxml(indent="   "))
    fi.close()
    print time
