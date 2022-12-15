#-- Library imports
import time
import math
import sys
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
import dronekit_sitl
from pymavlink import mavutil
import argparse
from curses import baudrate
from datetime import datetime
import scipy.io as sio
import pandas as pd
import logging
import threading
from tqdm import tqdm

# TODO: Update Radar Package
# from library.Rad24GHz import Rad24GHz
from library.DataScripts.storetelemetry import StoreTelemetry
from library.FlightScripts.squareflight import SquareFlight
from library.FlightScripts.tagflight import TagFlight

#--------------------------------------------------
#-------------- ESTABLISH ARGUMENTS  
#--------------------------------------------------    
# -- Declare Program Values
parser = argparse.ArgumentParser(
                    prog = 'MMWave Radar Drone',
                    description = 'Millimeter Wave Radar Drone Script Handler',
                    epilog = 'Developed by Pranav Chandra')
# parser.add_argument('--connect', help='connect to vehicle', action='store_true')
parser.add_argument("--sitl", help="run a simulation drone", action='store_true')
args = parser.parse_args()

#--------------------------------------------------
#-------------- IMPORT REMAINING LIBRARIES  
#--------------------------------------------------    
if not args.sitl:
    from library.DataScripts.camera import CameraCapture

#--------------------------------------------------
#-------------- PARSE ARGUMENTS  
#--------------------------------------------------    
# if not args.connect:
#     sys.exit("connect command not found")

#--------------------------------------------------
#-------------- INITIALIZE  
#--------------------------------------------------      
logging.basicConfig(level=logging.NOTSET,
                    format="%(asctime)s (T:%(thread)d):- %(message)s")

lock = threading.Lock()

#--------------------------------------------------
#-------------- CONNECTION  
#--------------------------------------------------    
# -- Declare Connection System
if args.sitl:
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()
else: # if real drone used
    connection_string = "/dev/ttyACM0" # USB Connection
    # connection_string = "/dev/ttyAMA0" # Serial Connection
    baud_rate = 115200

# -- Connect to the vehicle
print(">> Connecting with the UAV...")
if args.sitl:
    vehicle = connect(connection_string, wait_ready=True)
else: # if real drone used
    vehicle = connect(connection_string, baud=baud_rate, wait_ready=True) #- wait_ready flag hold the program untill all the parameters are been read (=, not .)
print(">> Connection with UAV established on %s"%connection_string)

#--------------------------------------------------
#-------------- PRINT SOME VEHICLE ATTRIBUTES  
#--------------------------------------------------  
# Get some vehicle attributes (state)
print("Get some vehicle attribute values:")
print(" GPS: %s" % vehicle.gps_0)
print(" Battery: %s" % vehicle.battery)
print(" Last Heartbeat: %s" % vehicle.last_heartbeat)
print(" Is Armable?: %s" % vehicle.is_armable)
print(" System status: %s" % vehicle.system_status.state)
print(" Mode: %s" % vehicle.mode.name)    # settable

#--------------------------------------------------
#-------------- MAIN FUNCTION  
#--------------------------------------------------    
# TODO: radar script
drone = TagFlight(lock=lock, vehicle=vehicle, alt=1.5, tag=1, testtime=30)
telemetry = StoreTelemetry(lock=lock, vehicle=vehicle, interval = 0.001, droneclass=drone)
camera = CameraCapture(lock=lock, interval=0.5, startafter=3, droneclass=drone) if not args.sitl else ""

# run objects
drone.start()
telemetry.start()
if not args.sitl:
    camera.start()

# join threads to background
drone.join()
telemetry.join()
if not args.sitl:
    camera.join()

#-- Wait until mission threads complete

#-- Wait 5 seconds for threads termination
print (">> Closing vehicle connection in 5 seconds...")
for sec in range(5):
    time.sleep(1)
filename = telemetry.get_filename()

#-- Close connection
vehicle.close()
print (">> Vehicle connection closed")

#-- Stop simulation if needed
if args.sitl:
    # Shut down simulator
    sitl.stop()
    print("Simulation Completed")

