#!/usr/bin/python
import subprocess
import os.path
import os
import re
import time
import socket
import sys
import math
import operator
import string

import datetime
import smtplib

import mimetypes
from email import encoders
from email.message        import Message
from email.mime.message   import MIMEMessage
from email.mime.audio     import MIMEAudio
from email.mime.image     import MIMEImage
from email.mime.base      import MIMEBase
from email.mime.multipart import MIMEMultipart
from email.mime.text      import MIMEText

mimetypes.add_type('text/plain','.log')

def mail(serverURL, sender, to, subject, text, files=[]):
    wrapper = MIMEMultipart()

    wrapper['Subject'] = subject
    wrapper['From'] = sender
    wrapper['To'] = to
    wrapper.preamble = 'You will not see this in a MIME-aware mail reader.\n'

    wrapper.attach(MIMEText(text))

    for f in files:
        ctype, encoding = mimetypes.guess_type(f)
        if ctype is None or encoding is not None:
            ctype = 'application/octet-stream'
        maintype, subtype = ctype.split('/', 1)
        if maintype == 'text':
            fp = open(f)
            msg = MIMEText(fp.read(), _subtype=subtype)
        elif maintype == 'image':
            fp = open(f, 'rb')
            msg = MIMEImage(fp.read(), _subtype=subtype)
            fp.close()
        elif maintype == 'audio':
            fp = open(f,'rb')
            msg = MIMEAudio(fp.read(), _subtype=subtype)
            fp.close()
        else:
            fp = open(f, 'rb')
            msg = MIMEBase(maintype, subtype)
            msg.set_payload(fp.read())
            fp.close()
            encoders.encode_base64(msg)
        msg.add_header('Content-Disposition', 'attachment', filename=f)
        wrapper.attach(msg)
    s = smtplib.SMTP(serverURL)
    s.sendmail(sender, [to], wrapper.as_string())
    s.quit()

def run_sumo_test(exe, params, niters=1):
    ## params: 0 -> begin; 1 -> end; 2 -> route; 3 -> net;
    best = {}
    for itno in xrange(0, niters):
        res = {}
        print "Doing run %d" % itno
        run = subprocess.Popen([exe, "-b", str(params[0]), "-e", str(params[1]), "-r", params[2], "-n", params[3], "-v", "-W", "--no-step-log"], stdout=subprocess.PIPE)
        for line in run.stdout:
            print "  [sumo output]", line,
            v = [string.strip(x) for x in line.split(":")]
            if v[0] == "Duration":
                res[v[0]] = float(v[1][:-2])/1000.0
            elif v[0] == "Simulation ended at time":
                res[v[0][-4:]] = int(v[1])
            elif v[0] == "Emitted" or v[0] == "Running" or v[0] == "Waiting":
                res[v[0]] = int(v[1])
        if(not best.has_key("Duration") or best["Duration"] > res["Duration"]):
            best = res
        print "Done with run %d" % itno
        print "-" * 70
    return best

def run_hwm_test(exe, params, niters=1):
    ## params: 0 -> begin; 1 -> end; 2 -> route; 3 -> net;
    best = {}
    for itno in xrange(0, niters):
        res = {}
        print "Doing run %d" % itno
        run = subprocess.Popen([exe, params[3], params[2], str(params[0]), str(params[1])], stdout=subprocess.PIPE)
        for line in run.stdout:
            print "  [hwm output]", line,
            v = [string.strip(x) for x in line.split(":")]
            if v[0] == "duration":
                res["Duration"] = float(v[1][:-1])
            elif v[0] == "end time":
                res["time"] = int(float(v[1]))
        if(not best.has_key("Duration") or best["Duration"] > res["Duration"]):
            best = res
        print "Done with run %d" % itno
        print "-" * 70
    return best

def loghead():
    return "sim," \
        + "run,"  \
        + "endtime," \
        + "duration"

def logentry(simname, datafile, res):
    fmt = "%s," \
        + "%s," \
        + "%f," \
        + "%f"
    tup = (simname,
           datafile,
           res['time'],
           res['Duration'])
    return fmt % tup

root = '/home/sewall/unc/traffic/hwm/etc/sumo-compare'

input_files = ( ('sparse', 0, 100, 'sparse.rou.xml', 'sparse.net.xml', 'sparse-hwm.xml'),
                ('medium', 0, 100, 'medium.rou.xml', 'medium.net.xml', 'medium-hwm.xml'),
                ('dense',  0, 100, 'dense.rou.xml',  'dense.net.xml',  'dense-hwm.xml'))

exes = ['/home/sewall/src/sumo-0.11.0/src/sumo', '/home/sewall/unc/traffic/hwm/test-progs/run-sim']

if __name__ == '__main__':
    start = time.time()

    sim_iters = int(sys.argv[1])
    if(sim_iters < 1):
        print "Need positive sim_iters"
        sys.exit(1)

    logfile_name = "%s_%s_%s.log" % (socket.gethostname(),
                                     time.strftime("%m_%d_%y_%H%M_%S"),
                                     "sumohwmcompare")
    logfile = open(logfile_name, 'w')
    print >> logfile, loghead()
    print "Log is %s" % logfile_name
    for ifc in input_files:
        jobname = ifc[0]
        time_interval = ifc[1:3]
        (route_file, sumo_net, hwm_net) = ( os.path.join(root, jobname, x) for x in ifc[3:] )
        print "Doing job file %s" % (jobname,)

        print "Doing sumo run"
        res = run_sumo_test(exes[0],
                            time_interval + (route_file, sumo_net),
                            sim_iters)
        print >> logfile, logentry("sumo",jobname, res)
        logfile.flush()
        print "Done with sumo run"
        print "*" * 70

        print "Doing hwm run"
        res = run_hwm_test(exes[1],
                            time_interval + (route_file, hwm_net),
                            sim_iters)
        print >> logfile, logentry("hwm",jobname, res)
        logfile.flush()
        print "Done with hwm run"
        print "*" * 70

        print "Done with data file %s" % (jobname,)
        print "@" * 70
    logfile.close()
    print "Output written to %s" % logfile_name
    end = time.time()

    mail('smtp.unc.edu', 'jsewall@email.unc.edu', 'jasonsewall@gmail.com',
         "Run %s finished at %s" %(logfile_name, time.asctime()),
         "Started at %s\nFinished at %s\nElapsed %s\n" % (time.ctime(start), time.ctime(end), str(datetime.timedelta(0, end-start))),
         [logfile_name])


