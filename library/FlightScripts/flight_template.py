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
        self.FLIGHT_ALT = kwargs.get("alt", 1.5)
        self._mission_in_progress = True

        # Check if vehicle is given
        # if self.vehicle == None:
        #     raise Exception("no_vehicle_excetion: No vehicle provided")

        # TODO: you may add any other parameters to this constructor in the following format
        # self.[CLASS VARIABLE] = kwargs.get([PARAMETER NAME], [DEFAULT VALUE IF NO PARAM PASSED])

    
    def run(self):
        print('Create a new mission (for current location)')
        self.adds_mission(self.vehicle.location.global_frame)


        # Takeoff to FLIGHT_ALT meters
        self.arm_and_takeoff(self.FLIGHT_ALT)

        print("Starting mission")
        # Reset mission set to first (0) waypoint
        self.vehicle.commands.next=0

        # Set mode to AUTO to start mission
        self.vehicle.mode = VehicleMode("AUTO")


        # Monitor mission. 
        # Demonstrates getting and setting the command number 
        # Uses distance_to_current_waypoint(), a convenience function for finding the 
        #   distance to the next waypoint.
        while True:
            nextwaypoint=self.vehicle.commands.next
            print('Distance to waypoint (%s): %s' % (nextwaypoint, self.distance_to_current_waypoint()))

            # TODO: Change Value to number of waypoints added
            if nextwaypoint==...:
                break

            time.sleep(1)

        print('Return to launch')
        self.vehicle.mode = VehicleMode("RTL")

        # Finish flight
        print("Flight Complete")
        self._mission_in_progress = False


    def adds_mission(self, aLocation, aSize):
        """
        Adds a takeoff command and waypoint commands to the current mission. 

        The function assumes vehicle.commands matches the vehicle mission state 
        (you must have called download at least once in the session and after clearing the mission)
        """	

        cmds = self.vehicle.commands

        print(" Clear any existing commands")
        cmds.clear() 
        
        print(" Define/add new commands.")
        # Add new commands. The meaning/order of the parameters is documented in the Command class. 
        
        #Add MAV_CMD_NAV_TAKEOFF command. This is ignored if the vehicle is already in the air.
        cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, self.FLIGHT_ALT))

        # Define the MAV_CMD_NAV_WAYPOINT locations and add the commands
        # TODO: add waypoints as shown below
        # NOTE: you MUST add the final waypoint twice

        # point1 = self.get_location_metres(aLocation, 0, aSize)
        # cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point1.lat, point1.lon, self.FLIGHT_ALT))

        print(" Upload new commands to vehicle")
        cmds.upload()

    def abort_mission(self):
        """
        abort mission still in development.\n
        Clear the current mission.
        """
        cmds = self.vehicle.commands
        self.vehicle.commands.clear()
        self.vehicle.flush()

        cmds = self.vehicle.commands
        cmds.download()
        cmds.wait_ready()
        cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 0, 0, 0))


    def mission_in_progress(self):
        return self._mission_in_progress


    def get_location_metres(self, original_location, dNorth, dEast) -> LocationGlobal: 
        """
        Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the 
        specified `original_location`. The returned Location has the same `alt` value
        as `original_location`.

        The function is useful when you want to move the vehicle around specifying locations relative to 
        the current vehicle position.
        The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
        """
        earth_radius=6378137.0 #Radius of "spherical" earth
        #Coordinate offsets in radians
        dLat = dNorth/earth_radius
        dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

        #New position in decimal degrees
        newlat = original_location.lat + (dLat * 180/math.pi)
        newlon = original_location.lon + (dLon * 180/math.pi)
        return LocationGlobal(newlat, newlon,original_location.alt)


    def get_distance_metres(self, aLocation1, aLocation2) -> float:
        """
        Returns the ground distance in metres between two LocationGlobal objects.

        This method is an approximation, and will not be accurate over large distances and close to the 
        earth's poles.
        """
        dlat = aLocation2.lat - aLocation1.lat
        dlong = aLocation2.lon - aLocation1.lon
        return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5


    def distance_to_current_waypoint(self):
        """
        Gets distance in metres to the current waypoint. 
        It returns None for the first waypoint (Home location).
        """
        nextwaypoint = self.vehicle.commands.next
        if nextwaypoint==0:
            return None
        missionitem=self.vehicle.commands[nextwaypoint-1] #commands are zero indexed
        lat = missionitem.x
        lon = missionitem.y
        alt = missionitem.z
        targetWaypointLocation = LocationGlobalRelative(lat,lon,alt)
        distancetopoint = self.get_distance_metres(self.vehicle.location.global_frame, targetWaypointLocation)
        return distancetopoint


    def download_mission(self):
        """
        Download the current mission from the vehicle.
        """
        cmds = self.vehicle.commands
        cmds.download()
        cmds.wait_ready() # wait until download is complete.


    def arm_and_takeoff(self, aTargetAltitude):
        """
        Arms vehicle and fly to aTargetAltitude.
        """

        print("Basic pre-arm checks")
        # Don't let the user try to arm until autopilot is ready
        while not self.vehicle.is_armable:
            print(" Waiting for vehicle to initialise...")
            time.sleep(1)

            
        print("Arming motors")
        # Copter should arm in GUIDED mode
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True

        while not self.vehicle.armed:      
            print(" Waiting for arming...")
            time.sleep(1)

        print("Taking off!")
        self.vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

        # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command 
        #  after Vehicle.simple_takeoff will execute immediately).
        while True:
            print(" Altitude: ", self.vehicle.location.global_relative_frame.alt)      
            if self.vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: #Trigger just below target alt.
                print("Reached target altitude")
                break
            time.sleep(1)

