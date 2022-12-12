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


class SquareFlight(threading.Thread):
    """
    SquareFlight(lock, vehicle, aSize)
    """
    def __init__(self, **kwargs):
        threading.Thread.__init__(self)
        self._kwargs = kwargs
        self._lock = kwargs.get("lock", None)
        self.vehicle = kwargs.get("vehicle", None)
        self._aSize = kwargs.get("size", 10)
        self._mission_in_progress = True

    
    def run(self):
        print('Create a new mission (for current location)')
        SquareFlight.adds_square_mission(self.vehicle.location.global_frame,10)


        # From Copter 3.3 you will be able to take off using a mission item. Plane must take off using a mission item (currently).
        SquareFlight.arm_and_takeoff(10)

        print(">>> Starting mission")
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
            print('Distance to waypoint (%s): %s' % (nextwaypoint, SquareFlight.distance_to_current_waypoint()))
        
            if nextwaypoint==3: #Skip to next waypoint
                print('Skipping to Waypoint 5 when reach waypoint 3')
                self.vehicle.commands.next = 5
            if nextwaypoint==5: #Dummy waypoint - as soon as we reach waypoint 4 this is true and we exit.
                print("Exit 'standard' mission when start heading to final waypoint (5)")
                break;
            time.sleep(1)

        print('Return to launch')
        self.vehicle.mode = VehicleMode("RTL")

        # Finish flight
        print("Flight Complete")
        self._mission_in_progress = False

    
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
        cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 0, 0, 10))


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
        distancetopoint = SquareFlight.get_distance_metres(self.vehicle.location.global_frame, targetWaypointLocation)
        return distancetopoint


    def download_mission(self):
        """
        Download the current mission from the vehicle.
        """
        cmds = self.vehicle.commands
        cmds.download()
        cmds.wait_ready() # wait until download is complete.


    def adds_square_mission(self, aLocation, aSize):
        """
        Adds a takeoff command and four waypoint commands to the current mission. 
        The waypoints are positioned to form a square of side length 2*aSize around the specified LocationGlobal (aLocation).

        The function assumes vehicle.commands matches the vehicle mission state 
        """	

        cmds = self.vehicle.commands

        print(" Clear any existing commands")
        cmds.clear() 
        
        print(" Define/add new commands.")
        # Add new commands. The meaning/order of the parameters is documented in the Command class. 
        
        #Add MAV_CMD_NAV_TAKEOFF command. This is ignored if the vehicle is already in the air.
        cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, 10))

        #Define the four MAV_CMD_NAV_WAYPOINT locations and add the commands
        point1 = SquareFlight.get_location_metres(aLocation, aSize, -aSize)
        point2 = SquareFlight.get_location_metres(aLocation, aSize, aSize)
        point3 = SquareFlight.get_location_metres(aLocation, -aSize, aSize)
        point4 = SquareFlight.get_location_metres(aLocation, -aSize, -aSize)
        cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point1.lat, point1.lon, 11))
        cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point2.lat, point2.lon, 12))
        cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point3.lat, point3.lon, 13))
        cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point4.lat, point4.lon, 14))
        #add dummy waypoint "5" at point 4 (lets us know when have reached destination)
        cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point4.lat, point4.lon, 14))    

        print(" Upload new commands to vehicle")
        cmds.upload()


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

        

