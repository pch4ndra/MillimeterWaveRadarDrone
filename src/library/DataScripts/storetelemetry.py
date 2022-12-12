#!/usr/bin/env python
# -*- coding: utf-8 -*-

#-- Library imports
from curses import baudrate
from dronekit import connect, VehicleMode
import time
from datetime import datetime
import scipy.io as sio
import pandas as pd
import logging
import threading
import time


class StoreTelemetry(threading.Thread):
    """
    StoreTelemtry(lock, vehicle, interval)
    """
    def __init__(self, **kwargs):
        threading.Thread.__init__(self)
        self._kwargs = kwargs
        self._lock = kwargs.get("lock", None)
        self.vehicle = kwargs.get("vehicle", None)
        self._mission_in_progress = True

        if self.vehicle == None:
            raise Exception("no_vehicle_excetion: No vehicle provided")

        self._filename = datetime.now().strftime("%x-%X") + ".mat"
        # self._filename = datetime.now().strftime("%x-%X") + ".csv"

        self._data = pd.DataFrame(columns = ['Timestamp', 'Autopilot Firmware version', 
                                            'Autopilot capabilities (supports ftp)', 'Global Location', 
                                            'Global Location (relative altitude)', 'Local Location', 
                                            'Attitude', 'Velocity', 'GPS', 'Groundspeed', 'Airspeed', 
                                            'Gimbal status', 'Battery', 'EKF OK?', 'Last Heartbeat', 
                                            'Rangefinder', 'Rangefinder distance', 'Rangefinder voltage', 
                                            'Heading', 'Is Armable?', 'System status', 'Mode', 'Armed'])


    def run(self):
        while (self._mission_in_progress):
            self._data.loc[len(self._data)] = [datetime.now().strftime("%x-%X"), self.vehicle.version, 
                                                self.vehicle.capabilities.ftp, 
                                                self.vehicle.location.global_frame, 
                                                self.vehicle.location.global_relative_frame, 
                                                self.vehicle.location.local_frame, 
                                                self.vehicle.attitude, self.vehicle.velocity, 
                                                self.vehicle.gps_0, self.vehicle.groundspeed, 
                                                self.vehicle.airspeed, self.vehicle.gimbal, 
                                                self.vehicle.battery, self.vehicle.ekf_ok, 
                                                self.vehicle.last_heartbeat, self.vehicle.rangefinder, 
                                                self.vehicle.rangefinder.distance, 
                                                self.vehicle.rangefinder.voltage, self.vehicle.heading, 
                                                self.vehicle.is_armable, 
                                                self.vehicle.system_status.state, 
                                                self.vehicle.mode.name, self.vehicle.armed]
            time.sleep(self._kwargs.get("interval", 1.0))
        
        print(" Saving data to data\\%s"%self._filename)
        sio.savemat('data\\telemetry\\'+self._filename, {name: col.values for name, col in self._data.items()})
        
        # if pandas dataframe
        # self._data.to_csv(index=False)

        #---- PARAMETERS
        # print("Maximum Throttle: %d"%self.vehicle.parameters['THR_MIN']) 

        #-- You can read and write the parameters
        # self.vehicle.parameters['THR_MIN'] = 50
        # time.sleep(1)
        # print("Maximum Throttle: %d"%self.vehicle.parameters['THR_MIN'])
        # Set delay between thread ouputs
    
    def mission_complete(self):
        self._mission_in_progress = False


# lock = None
# drone = StoreTelemetry(lock=lock, vehicle = vehicle, interval = 2)
# drone.start()
# drone.join()


