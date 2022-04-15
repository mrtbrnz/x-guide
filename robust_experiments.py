#!/usr/bin/env python3
from __future__ import print_function
from mission_control import *
import sys
from os import path, getenv

from math import radians, atan2
import time
import numpy as np

# # if PAPARAZZI_SRC or PAPARAZZI_HOME not set, then assume the tree containing this
# # file is a reasonable substitute
PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
PPRZ_SRC = getenv("PAPARAZZI_SRC", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
sys.path.append(PPRZ_SRC + "/sw/lib/python")
sys.path.append(PPRZ_HOME + "/var/lib/python") # pprzlink
import settings
import pprz_connect
from settings_xml_parse import PaparazziACSettings

from pprzlink.ivy import IvyMessagesInterface

from pprzlink.message import PprzMessage

from vector_fields import TrajectoryEllipse, ParametricTrajectory, spheric_geo_fence, repel, Controller

def main():
    import argparse
    parser = argparse.ArgumentParser(description="Mission Control")
    parser.add_argument("-on", "--running-on", help="Where is the code running-on", dest='running_on', default='ground')
    parser.add_argument("-f", "--file", help="path to messages.xml file", default='pprzlink/messages.xml')
    parser.add_argument("-c", "--class", help="message class", dest='msg_class', default='telemetry')
    parser.add_argument("-d", "--device", help="device name", dest='dev', default='/dev/ttyUSB0') #ttyTHS1
    parser.add_argument("-b", "--baudrate", help="baudrate", dest='baud', default=230400, type=int)
    parser.add_argument("-id", "--ac_id", help="aircraft id (receiver)", dest='ac_id', default=23, type=int)
    parser.add_argument("--interface_id", help="interface id (sender)", dest='id', default=0, type=int)
    # parser.add_argument("-ti", "--target_id", dest='target_id', default=2, type=int, help="Target aircraft ID")
    # parser.add_argument("-ri", "--repel_id", dest='repel_id', default=2, type=int, help="Repellant aircraft ID")
    # parser.add_argument("-bi", "--base_id", dest='base_id', default=10, type=int, help="Base aircraft ID")
    args = parser.parse_args()

    if args.running_on == 'ground' :
        interface  = IvyMessagesInterface("PprzConnect")

    if args.running_on == "serial" :
        from pprzlink.serial import SerialMessagesInterface
        interface = SerialMessagesInterface(None, device=args.dev,
                                               baudrate=args.baud, msg_class=args.msg_class, interface_id=args.id, verbose=False)


    mission_plan_dict={ 'morph' :{'start':None, 'duration':3, 'finalized':False},
                        'takeoff' :{'start':None, 'duration':15, 'finalized':False},
                        # 'circle'  :{'start':None, 'duration':15, 'finalized':False},
                        # 'parametric_circle'  :{'start':None, 'duration':15, 'finalized':False},
                        # 'nav2land':{'start':None, 'duration':5,  'finalized':False},
                        # 'land'    :{'start':None, 'duration':10, 'finalized':False} } #,
                        # 'kill'    :{'start':None, 'duration':10, 'finalized':False} }
                        }
    # mission_plan_dict={ 'parametric_circle'  :{'start':None, 'duration':15, 'finalized':False} }

    mission_plan_dict={#'takeoff' :{'start':None, 'duration':15, 'finalized':False},
                        'morph' :{'start':None, 'duration':3, 'finalized':False},
                        'M1_fault' :{'start':None, 'duration':10, 'finalized':False},
                        'Resurrect1' :{'start':None, 'duration':10, 'finalized':False},
                        'M2_fault' :{'start':None, 'duration':10, 'finalized':False},
                        'Resurrect2' :{'start':None, 'duration':10, 'finalized':False},
                        'M3_fault' :{'start':None, 'duration':10, 'finalized':False},
                        'Resurrect3' :{'start':None, 'duration':10, 'finalized':False},
                        'M4_fault' :{'start':None, 'duration':10, 'finalized':False},
                        'Resurrect4' :{'start':None, 'duration':10, 'finalized':False},
                        'M5_fault' :{'start':None, 'duration':10, 'finalized':False},
                        'Resurrect5' :{'start':None, 'duration':10, 'finalized':False},
                        'M6_fault' :{'start':None, 'duration':10, 'finalized':False},
                        'Resurrect6' :{'start':None, 'duration':10, 'finalized':False},
                        }

    # Automatic Exploration of own Robustness
    mission_plan_dict={#'takeoff' :{'start':None, 'duration':15, 'finalized':False},
                        'morph' :{'start':None, 'duration':3, 'finalized':False},
                        'M1_fault' :{'start':None, 'duration':5, 'finalized':False},
                        'Explore_robustness' :{'start':None, 'duration':600, 'finalized':False},
                        'Resurrect1' :{'start':None, 'duration':10, 'finalized':False},
                        # 'M1_fault' :{'start':None, 'duration':10, 'finalized':False},
                        }

    mission_plan_dict={ 'circle'  :{'start':None, 'duration':15, 'finalized':False} }

    # mission_plan_dict={ 'parametric_circle'  :{'start':None, 'duration':15, 'finalized':False} }

    # mission_plan_dict={ 'debug_mode'  :{'start':None, 'duration':15, 'finalized':False} }

    mission_plan_dict={ 'Explore_robustness_hover_step'  :{'start':None, 'duration':600, 'finalized':False} }


    mission_plan_dict={ 'Explore_robustness_circle_step'  :{'start':None, 'duration':600, 'finalized':False} }


    vehicle_parameter_dict={}

    if args.running_on == 'ground' :
        try:
            mc = MissionControl(interface=interface)
            mc.assign(mission_plan_dict)
            mc.assign_vehicle_properties()
            time.sleep(1.5)

            while True:
                mc.run_every_vehicle()
                time.sleep(0.09)

        except (KeyboardInterrupt, SystemExit):
            mission_end_plan_dict={'Resurrect7' :{'start':None, 'duration':10, 'finalized':False}, 'safe2land'  :{'start':None, 'duration':15, 'finalized':False} }
            mc.assign(mission_end_plan_dict)  # mc.assign_vehicle_properties()
            time.sleep(0.5)
            for i in range(10):
                mc.run_every_vehicle()
                time.sleep(0.5)
            print('Shutting down...')
            # mc.set_nav_mode()
            mc.shutdown()
            time.sleep(0.6)
            exit()


if __name__ == '__main__':
    main()