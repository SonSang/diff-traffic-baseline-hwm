from xml.dom.minidom import Document
import random

if __name__ == '__main__':
    doc = Document()
    routes = doc.createElement('routes')
    doc.appendChild(routes)

    N = 10000
    time = 0
    for i in xrange(0, N):
        v = doc.createElement('vehicle')
        routes.appendChild(v)
        v.setAttribute("id", "0_%02d" % i)
        v.setAttribute("depart", str(time))
        v.setAttribute("departlane",  str(random.randint(0, 5)))
        v.setAttribute("arrivallane", str(random.randint(0, 5)))
        v.setAttribute("departspeed", str(random.uniform(10, 14)))

        time += random.randint(1, 5)

        r = doc.createElement('route')
        r.setAttribute("edges", "e0 e1 e2")
        v.appendChild(r)

    fi = open("/home/sewall/src/sumo-0.11.0/src/fout.rou.xml", 'w')
    fi.write(doc.toprettyxml(indent="   "))
    fi.close()

    print time
