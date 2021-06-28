#!/usr/bin/env python3

from __future__ import print_function
import pdb
import sys
from os import path, getenv

# if PAPARAZZI_SRC or PAPARAZZI_HOME not set, then assume the tree containing this
# file is a reasonable substitute
PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
PPRZ_SRC = getenv("PAPARAZZI_SRC", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
sys.path.append(PPRZ_SRC + "/sw/lib/python")
sys.path.append(PPRZ_HOME + "/var/lib/python") # pprzlink

# from pprzlink.ivy import IvyMessagesInterface
# from pprzlink.message import PprzMessage
# from settings_xml_parse import PaparazziACSettings

# from math import radians
# from time import sleep
# import numpy as np

import pprz_connect
import settings
import time

# define a callack
def new_ac(conf):
    print(conf)



connect = pprz_connect.PprzConnect(notify=new_ac, verbose=False)

time.sleep(1)
# aset = connect.conf_by_id(7)
# pdb.set_trace()
# do some things here or wait for event
# aset.settings
try:
    # sm = settings.PprzSettingsManager(connect.conf_by_id('4').settings, 4, connect.ivy)
    i=0
    while True:
        time.sleep(2)
        connect.get_aircraft
        ac_id_list = connect.conf_by_id()
        # # pdb.set_trace()
        print('Here is the list !', ac_id_list.keys())
        # sm["nav_heading"] = 1000*i
        # i +=1
except KeyboardInterrupt:
    pass

# close before leaving
connect.shutdown()