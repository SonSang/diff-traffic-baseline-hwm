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

def run_test(exe, input_file, niters, nthreads, ntests=1):
    best = {}
    for iter in xrange(0, ntests):
        res = {}
        print "Doing run %d" % iter
        run = subprocess.Popen([exe, input_file, niters, nthreads], stdout=subprocess.PIPE)
        regex = re.compile("([^=]*)= (-?[0-9\.]+)")
        for line in run.stdout:
            print "  [output]", line,
            result = regex.search(line)
            if result != None:
                res[result.group(1).rstrip()] = float(result.group(2))
        if(not best.has_key("tot. time") or best["tot. time"] > res["tot. time"]):
            best = res
        print "Done with run %d" % iter
        print "-" * 70
    return best

def loghead():
    return "data file,"  \
        + "simulation iterations,"   \
        + "nthreads," \
        + "riemann," \
        + "max," \
        + "update," \
        + "total time,"

def logentry(datafile, sim_iters, nthreads, res):
    fmt = "%s,"  \
        + "%d,"  \
        + "%d,"  \
        + "%f,"  \
        + "%f,"  \
        + "%f,"  \
        + "%f,"
    tup = ((datafile,) + (sim_iters,) + (nthreads,) + \
           (res['riemann'],
            res['max'],
            res['update'],
            res['tot. time']))
    return fmt % tup

if __name__ == '__main__':
    if len(sys.argv) < 4:
        print "usage: %s: <exe> <sim_iters> <threadmax>" % sys.argv[0]
        sys.exit(1)
    else:
        start = time.time()
        threads = range(1, int(sys.argv[3])+1)
        sim_iters = int(sys.argv[2])
        if(sim_iters < 1):
            print "Need positive sim_iters"
            sys.exit(1)
        datafiles = ["/home/jsewall/ref/traffic/hwm-perf/biscoe-test.xml.gz"]

        exe = sys.argv[1]

        logfile_name = "%s_%s_%s.log" % (socket.gethostname(),
                                         time.strftime("%m_%d_%y_%H%M_%S"),
                                         os.path.basename(exe).rstrip('.exe'))
        logfile = open(logfile_name, 'w')
        print >> logfile, loghead()
        print "Log is %s" % logfile_name
        for df in datafiles:
            print "Doing data file %s" % df
            for t in threads:
                print "Doing %d threads" % t
                res= run_test(os.path.join(os.getcwd(), exe),
                              df,
                              str(sim_iters),
                              str(t),
                              2)
                print >> logfile, logentry(df, sim_iters, t, res)
                logfile.flush()
                print "Done with %d threads" % t
                print "#" * 70
            print "Done with data file %s" % df
            print "@" * 70
        logfile.close()
        print "Output written to %s" % logfile_name
	end = time.time()

	print "Run %s finished at %s" % (logfile_name, time.asctime())
        print "Started at %s\nFinished at %s\nElapsed %s\n" % (time.ctime(start), time.ctime(end), str(datetime.timedelta(0, end-start)))

	# mail('smtp.unc.edu', 'jsewall@email.unc.edu', 'jasonsewall@gmail.com',
	#      "Run %s finished at %s" %(logfile_name, time.asctime()),
	#      "Started at %s\nFinished at %s\nElapsed %s\n" % (time.ctime(start), time.ctime(end), str(datetime.timedelta(0, end-start))),
	#      [logfile_name])
