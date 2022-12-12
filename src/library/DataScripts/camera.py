#!/usr/bin/env python
# -*- coding: utf-8 -*-

#-- Library imports
import time
from datetime import datetime
import logging
import threading
from picamera import PiCamera


class CameraCapture(threading.Thread):
    """
    CameraCapture(lock, interval, delay)
    """
    def __init__(self, **kwargs):
        threading.Thread.__init__(self)
        self._kwargs = kwargs
        self._lock = kwargs.get("lock", None)
        self._mission_in_progress = True

        self.camera = PiCamera()
        self.camera.start_preview()


    def run(self):
        time.sleep(self._kwargs.get("startafter", 1.0))
        while (self._mission_in_progress):
            self.camera.capture('data\\camera\\{0}.jpeg'.format(datetime.now().strftime("%x-%X")))
            time.sleep(self._kwargs.get("interval", 1.0))
        self.camera.stop_preview()
        print(" Image Capture Complete")       
        
    def mission_complete(self):
        self._mission_in_progress = False


