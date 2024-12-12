#!/usr/bin/env python

import threading, sys, time
import data_server, image_server

d = threading.Thread(target=data_server.run)
i = threading.Thread(target=image_server.run)

d.daemon = True
i.daemon = True

print "Starting servers"

d.start()
i.start()

try:
  while threading.activeCount():
    time.sleep(0.1)
except:
  print "Exiting"
