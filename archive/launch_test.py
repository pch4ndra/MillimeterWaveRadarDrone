#!/usr/bin/env python
# -*- coding: utf-8 -*-

#-- Library imports
import time
import math
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil
import argparse
from tqdm import tqdm
import sys
#from __future__ import print_function

TAKEOFF_HEIGHT = 10 if len(sys.argv) == 1 else int(sys.argv[1])
# print(TAKEOFF_HEIGHT)

#-- Connect to the vehicle
print(">> Connecting to Vehicle...")
# parser = argparse.ArgumentParser(description='commands')
# parser.add_argument('--connect')
# args = parser.parse_args()

# -- Declare Connection System
connection_string = "/dev/ttyACM0" # USB Connection
baud_rate = 115200
# connection_string = "/dev/ttyAMA0" # Serial Connection

# -- Establish Vehicle Connection
print("Connection to the vehicle on %s"%connection_string)
vehicle = connect(connection_string, baud=baud_rate, wait_ready=True)


def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

        
    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:      
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command 
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)      
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: #Trigger just below target alt.
            print("Reached target altitude")
            break
        time.sleep(1)

# Clear any existing commands on the drone
cmds = vehicle.commands
print("> SAFETY CHECK: Clear any existing commands")
cmds.clear()        

# From Copter 3.3 you will be able to take off using a mission item. Plane must take off using a mission item (currently).
print(">>> Taking off")
arm_and_takeoff(TAKEOFF_HEIGHT)

# Flight mode
print(">>> Flight status")
for timer in tqdm(range(0,30)):
    time.sleep(1)

print('>>> Return to launch')
vehicle.mode = VehicleMode("RTL")

#Close vehicle object before exiting script
print(">>> Close vehicle object")
vehicle.close()

print(">> Flight Complete")
