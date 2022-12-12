from curses import baudrate
from dronekit import connect, VehicleMode
import time

from library.Rad24GHz import Rad24GHz
from scipy.io import savemat

import logging
import threading
import time
logging.basicConfig(level=logging.INFO,
                    format="%(asctime)s (T:%(thread)d):- %(message)s")


class TelemetryTester(threading.Thread):
    def __init__(self, **kwargs):
        threading.Thread.__init__(self)
        self._kwargs = kwargs
        self._lock = kwargs.get("lock", None)
        self._connection_string = kwargs.get("connectstring", "/dev/ttyAMA0") # defaults to serial
        self._baud_rate = kwargs.get("baudrate", 57600) # defaults to dronekit default of 57600

        print(">>>> Connecting with the UAV <<<")
        #- wait_ready flag hold the program untill all the parameters are been read (=, not .)
        self.vehicle = connect(self._connection_string, baud=self._baud_rate, wait_ready=True)
    def run(self):
        for i in range(5):
            print("=======================================================")
            print("==================  Cycle ", str(i+1))
            print("=======================================================")
            self.readTele()
            # Set delay between thread ouputs
            time.sleep(self._kwargs.get("delay", 1.0))
        
        #--- Now we close the simulation
        self.vehicle.close()
    
    # print the telemetry data
    def readTele(self):
        #-- Read information from the autopilot:
        #- Version and attributes
        self.vehicle.wait_ready('autopilot_version')
        print('Autopilot version: %s'%self.vehicle.version)
        #- Does the firmware support the companion pc to set the attitude?
        print('Supports set attitude from companion: %s'%self.vehicle.capabilities.set_attitude_target_local_ned)
        #- Read the actual position
        print('Position: %s'% self.vehicle.location.global_relative_frame)
        #- Read the actual attitude roll, pitch, yaw
        print('Attitude: %s'% self.vehicle.attitude)
        #- Read the actual velocity (m/s)
        print('Velocity: %s'%self.vehicle.velocity) #- North, east, down
        #- When did we receive the last heartbeat
        print('Last Heartbeat: %s'%self.vehicle.last_heartbeat)
        #- Is the vehicle good to Arm?
        print('Is the vehicle armable: %s'%self.vehicle.is_armable)
        #- Which is the total ground speed?   Note: this is settable
        print('Groundspeed: %s'% self.vehicle.groundspeed) #(%)
        #- What is the actual flight mode?    Note: this is settable
        print('Mode: %s'% self.vehicle.mode.name)
        #- Is the vehicle armed               Note: this is settable
        print('Armed: %s'%self.vehicle.armed)
        #- Is thestate estimation filter ok?
        print('EKF Ok: %s'%self.vehicle.ekf_ok)


        print("Adding an altitude listener")
        self.vehicle.add_attribute_listener('attitude', TelemetryTester.altitude_callback) #-- message type, callback function
        # time.sleep(1) #-- Print data for 1 seconds

        #--- Now we print the attitude from the callback for 5 seconds, then we remove the callback
        self.vehicle.remove_attribute_listener('attitude', TelemetryTester.altitude_callback) #(.remove)

        #-- Do same listener process for location
        print("Adding a GPS location listener")
        self.vehicle.add_attribute_listener('GLOBAL_POSITION_INT', TelemetryTester.location_callback)
        time.sleep(2)
        self.vehicle.remove_attribute_listener('GLOBAL_POSITION_INT', TelemetryTester.location_callback)
        print("done with gps")

        print("Add GPS try 2")
        print("Global Location: %s" % self.vehicle.location.global_frame)
        print("done try 2")

        #---- PARAMETERS
        # print("Maximum Throttle: %d"%self.vehicle.parameters['THR_MIN']) 

        #-- You can read and write the parameters
        # self.vehicle.parameters['THR_MIN'] = 50
        # time.sleep(1)
        # print("Maximum Throttle: %d"%self.vehicle.parameters['THR_MIN'])
    
    #----- Listeners
    def altitude_callback(self, attr_name, value):
        print(self.vehicle.attitude)

    def location_callback(self, attr_name, msg):
        print("Location (Global): ", msg)

# -- Declare Connection System
connection_string = "/dev/ttyACM0"
baud_rate = 115200

lock = None

drone = TelemetryTester(lock=lock, connectstring = connection_string, baudrate = baud_rate)
drone.start()
drone.join()

# Collect data for 10 seconds
# DataAll, Timestamps = Rad24GHz.connect(action='measure', duration=10)
# save = {"Data": DataAll, "dtime": Timestamps}
# savemat("RPiData.mat", save)
