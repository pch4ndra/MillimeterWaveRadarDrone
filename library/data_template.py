#!/usr/bin/env python
# -*- coding: utf-8 -*-


#-- Library imports
import time
import math
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil
import argparse
from curses import baudrate
import logging
import threading
import time
#from __future__ import print_function


class Template(threading.Thread):
    """
    Templte(lock, vehicle)
    """
    def __init__(self, **kwargs):
        threading.Thread.__init__(self)
        self._kwargs = kwargs
        self._lock = kwargs.get("lock", None)
        self.vehicle = kwargs.get("vehicle", None)

        # Check if vehicle is given
        # if self.vehicle == None:
        #     raise Exception("no_vehicle_excetion: No vehicle provided")

        # you may add any other parameters to this constructor in the following format
        # self.[CLASS VARIABLE] = kwargs.get([PARAMETER NAME], [DEFAULT VALUE IF NO PARAM PASSED])

    
    def run(self):
        while (self.droneclass.mission_in_progress()):
            # add any code you want to run while the drone is performing it's mission
            ...

        

