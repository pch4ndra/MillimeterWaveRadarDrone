#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
import time
import math
from pymavlink import mavutil


#Set up option parsing to get connection string

connection_string = "/dev/serial0"

FLIGHT_ALT = 1.5
DISTANCE_FROM_TAG = 1
TAG_TEST_TIME = 30

# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True, baud=57600)


def get_location_metres(original_location, dNorth, dEast):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the 
    specified `original_location`. The returned Location has the same `alt` value
    as `original_location`.
    The function is useful when you want to move the vehicle around specifying locations relative to 
    the current vehicle position.
    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius=6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    return LocationGlobal(newlat, newlon,original_location.alt)


def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.
    This method is an approximation, and will not be accurate over large distances and close to the 
    earth's poles. It comes from the ArduPilot test code: 
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5



def distance_to_current_waypoint():
    """
    Gets distance in metres to the current waypoint. 
    It returns None for the first waypoint (Home location).
    """
    nextwaypoint = vehicle.commands.next
    if nextwaypoint==0:
        return None
    missionitem=vehicle.commands[nextwaypoint-1] #commands are zero indexed
    lat = missionitem.x
    lon = missionitem.y
    alt = missionitem.z
    targetWaypointLocation = LocationGlobalRelative(lat,lon,alt)
    distancetopoint = get_distance_metres(vehicle.location.global_frame, targetWaypointLocation)
    return distancetopoint


def download_mission():
    """
    Download the current mission from the vehicle.
    """
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready() # wait until download is complete.



# TODO: change mission style
def adds_mission(aLocation, aSize):
    """
    Adds a takeoff command and waypoint commands to the current mission. 

    The mission should take off FLIGHT_ALT meters in the air and travel DISTANCE_FROM_TAG to the east

    The function assumes vehicle.commands matches the vehicle mission state 
    (you must have called download at least once in the session and after clearing the mission)
    """	

    cmds = vehicle.commands

    print(" Clear any existing commands")
    cmds.clear() 
    
    print(" Define/add new commands.")
    # Add new commands. The meaning/order of the parameters is documented in the Command class. 
     
    #Add MAV_CMD_NAV_TAKEOFF command. This is ignored if the vehicle is already in the air.
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, FLIGHT_ALT))

    #Define the MAV_CMD_NAV_WAYPOINT locations and add the commands
    point1 = get_location_metres(aLocation, 0, aSize)
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point1.lat, point1.lon, FLIGHT_ALT))
    #add dummy waypoint "2" (lets us know when have reached destination)
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point1.lat, point1.lon, FLIGHT_ALT))

    print(" Upload new commands to vehicle")
    cmds.upload()


def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
        
    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:      
        print(" Waiting for arming...")
        time.sleep(1)
        # Try arming again
        # vehicle.armed = True

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

        
print('Create a new mission (for current location)')
adds_mission(vehicle.location.global_frame, DISTANCE_FROM_TAG) # TODO : change up mission


# Takeoff to FLIGHT_ALT meters
arm_and_takeoff(FLIGHT_ALT)

print("Starting mission")
# Reset mission set to first (0) waypoint
vehicle.commands.next=0

# Set mode to AUTO to start mission
vehicle.mode = VehicleMode("AUTO")


# Monitor mission. 
# Demonstrates getting and setting the command number 
# Uses distance_to_current_waypoint(), a convenience function for finding the 
#   distance to the next waypoint.
while True:
    nextwaypoint=vehicle.commands.next
    print('Distance to waypoint (%s): %s' % (nextwaypoint, distance_to_current_waypoint()))
  
    # if nextwaypoint==2: #Skip to next waypoint
    #     print('AT Waypoint 2. Moving to Final Waypoint 3')
    #     vehicle.commands.next = 3
    if nextwaypoint==2: #Dummy waypoint - as soon as we reach waypoint 2 this is true and we exit.
        print("Exit 'standard' mission when start heading to final waypoint (3)")
        print("Waiting %i seconds for Tag Test"%TAG_TEST_TIME)
        for t in range(TAG_TEST_TIME):
            print("Conducting Tag Test for %i seconds"%(TAG_TEST_TIME-t))
            time.sleep(1)
        print("Finished Conducting Tag Test for %i seconds ==> Returning Home"%(TAG_TEST_TIME))
        break
    time.sleep(1)

print('Return to launch')
vehicle.mode = VehicleMode("RTL")
vehicle.mode = VehicleMode("Land")

# Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()
