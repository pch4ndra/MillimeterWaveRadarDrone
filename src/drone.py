#-- Library imports
import time
import math
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil
import argparse
from curses import baudrate
from datetime import datetime
from library.Rad24GHz import Rad24GHz
import scipy.io as sio
import pandas as pd
import logging
import threading

from src.library.DataScripts.storetelemetry import StoreTelemetry
from src.library.DataScripts.camera import CameraCapture
from src.library.FlightScripts.squareflight import SquareFlight
from src.library.FlightScripts.tagflight import TagFlight

from tqdm import tqdm


#--------------------------------------------------
#-------------- PARSE ARGUMENTS  
#--------------------------------------------------    
# -- Establish Connection System
# parser = argparse.ArgumentParser(description='commands')
# parser.add_argument('--connect', help='connect to vehicle')
# parser.add_argument("--verbosity", help="increase output verbosity")
# args = parser.parse_args()


#--------------------------------------------------
#-------------- INITIALIZE  
#--------------------------------------------------      
logging.basicConfig(level=logging.INFO,
                    format="%(asctime)s (T:%(thread)d):- %(message)s")

lock = threading.Lock()

#--------------------------------------------------
#-------------- CONNECTION  
#--------------------------------------------------    
# -- Declare Connection System
connection_string = "/dev/ttyACM0" # USB Connection
baud_rate = 115200
# connection_string = "/dev/ttyAMA0" # Serial Connection

# -- Connect to the vehicle
print(">> Connecting with the UAV...")
vehicle = connect(connection_string, baud=baud_rate, wait_ready=True) #- wait_ready flag hold the program untill all the parameters are been read (=, not .)
print(">> Connection with UAV established on %s"%connection_string)

#--------------------------------------------------
#-------------- MAIN FUNCTION  
#--------------------------------------------------    
# TODO: radar script
drone = TagFlight(lock=lock, vehicle=vehicle, alt=1.5, tag=1, testtime=30)
telemetry = StoreTelemetry(lock=lock, vehicle=vehicle, interval = 0.5)
camera = CameraCapture(lock=lock, interval=0.5, startafter=3)

# run objects
drone.start()
telemetry.start()
camera.start()

# join threads to background
drone.join()
telemetry.join()
camera.join()

# wait until mission complete
while drone.mission_in_progress():
    continue

# stop telemetry reading and save file
telemetry.mission_complete()
camera.mission_complete()


#-- Wait 5 seconds for threads termination
print (">> Closing vehicle connection in 5 seconds...")
for sec in tqdm(range(5)):
    continue


#-- Close connection
vehicle.close()
print (">> Vehicle connection closed")

