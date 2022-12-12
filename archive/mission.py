import time
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil

# -- Declare Connection System
connection_string = "/dev/ttyACM0" # USB Connection
baud_rate = 115200
# connection_string = "/dev/ttyAMA0" # Serial Connection

#--------------------------------------------------
#-------------- FUNCTIONS  
#--------------------------------------------------
#-- Define arm and takeoff
def arm_and_takeoff(altitude):

   while not vehicle.is_armable:
      print("waiting to be armable")
      time.sleep(1)

   print("Arming motors")
   vehicle.mode = VehicleMode("GUIDED")
   vehicle.armed = True

   while not vehicle.armed: time.sleep(1)

   print("Taking Off")
   vehicle.simple_takeoff(altitude)

   while True:
      v_alt = vehicle.location.global_relative_frame.alt
      print(">> Altitude = %.1f m"%v_alt)
      if v_alt >= altitude - 1.0:
          print("Target altitude reached")
          break
      time.sleep(1)

def clear_mission(vehicle):
    """
    Clear the current mission.
    """
    cmds = vehicle.commands
    vehicle.commands.clear()
    vehicle.flush()

    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()

def download_mission(vehicle):
    """
    Download the current mission from the vehicle.
    """
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready() # wait until download is complete.
    

def get_current_mission(vehicle):
    """
    Downloads the mission and returns the wp list and number of WP 
    
    Input: 
        vehicle
        
    Return:
        n_wp, wpList
    """

    print ("Downloading mission")
    download_mission(vehicle)
    missionList = []
    n_WP        = 0
    for wp in vehicle.commands:
        missionList.append(wp)
        n_WP += 1 
        
    return n_WP, missionList
    

def add_last_waypoint_to_mission(                                       #--- Adds a last waypoint on the current mission file
        vehicle,            #--- vehicle object
        wp_Last_Latitude,   #--- [deg]  Target Latitude
        wp_Last_Longitude,  #--- [deg]  Target Longitude
        wp_Last_Altitude):  #--- [m]    Target Altitude
    """
    Upload the mission with the last WP as given and outputs the ID to be set
    """
    # Get the set of commands from the vehicle
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()

    # Save the vehicle commands to a list
    missionlist=[]
    for cmd in cmds:
        missionlist.append(cmd)

    # Modify the mission as needed. For example, here we change the
    wpLastObject = Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 
                           wp_Last_Latitude, wp_Last_Longitude, wp_Last_Altitude)
    missionlist.append(wpLastObject)

    # Clear the current mission (command is sent when we call upload())
    cmds.clear()

    #Write the modified mission and flush to the vehicle
    for cmd in missionlist:
        cmds.add(cmd)
    cmds.upload()
    
    return (cmds.count)    

def ChangeMode(vehicle, mode):
    while vehicle.mode != VehicleMode(mode):
            vehicle.mode = VehicleMode(mode)
            time.sleep(0.5)
    return True
#--------------------------------------------------
#-------------- INITIALIZE  
#--------------------------------------------------      
#-- Setup the commanded flying speed
gnd_speed = 10 # [m/s]
mode      = 'GROUND'

#--------------------------------------------------
#-------------- CONNECTION  
#--------------------------------------------------    
# -- Establish Vehicle Connection
print("Connection to the vehicle on %s"%connection_string)
vehicle = connect(connection_string, baud=baud_rate, wait_ready=True)


#--------------------------------------------------
#-------------- MAIN FUNCTION  
#--------------------------------------------------    
while True:
    
    if mode == 'GROUND':
        #--- Wait until a valid mission has been uploaded
        n_WP, missionList = get_current_mission(vehicle)
        time.sleep(2)
        if n_WP > 0:
            print ("A valid mission has been uploaded: takeoff!")
            mode = 'TAKEOFF'
            
    elif mode == 'TAKEOFF':
       
        #-- Add a fake waypoint at the end of the mission
        add_last_waypoint_to_mission(vehicle, vehicle.location.global_relative_frame.lat, 
                                       vehicle.location.global_relative_frame.lon, 
                                       vehicle.location.global_relative_frame.alt)
        print("Home waypoint added to the mission")
        time.sleep(1)
        #-- Takeoff
        arm_and_takeoff(10)
        
        #-- Change the UAV mode to AUTO
        print("Changing to AUTO")
        ChangeMode(vehicle,"AUTO")
        
        #-- Change mode, set the ground speed
        vehicle.groundspeed = gnd_speed
        mode = 'MISSION'
        print ("Switch mode to MISSION")
        
    elif mode == 'MISSION':
        #-- Here we just monitor the mission status. Once the mission is completed we go back
        #-- vehicle.commands.cout is the total number of waypoints
        #-- vehicle.commands.next is the waypoint the vehicle is going to
        #-- once next == cout, we just go home
        
        print ("Current WP: %d of %d "%(vehicle.commands.next, vehicle.commands.count))
        if vehicle.commands.next == vehicle.commands.count:
            print ("Final waypoint reached: go back home")
            #-- First we clear the flight mission
            clear_mission(vehicle)
            print ("Mission deleted")
            
            #-- We go back home
            ChangeMode(vehicle,"RTL")
            mode = "BACK"
            
    elif mode == "BACK":
        if vehicle.location.global_relative_frame.alt < 1:
            print ("Switch to GROUND mode, waiting for new missions")
            mode = 'GROUND'
    
    
    
    
    time.sleep(0.5)


