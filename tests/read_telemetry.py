from dronekit import connect, VehicleMode
import time

# -- Declare Connection System
connection_string = "/dev/ttyACM0" # USB Connection
baud_rate = 115200
# connection_string = "/dev/ttyAMA0" # Serial Connection

print(">>>> Connecting with the UAV <<<")
vehicle = connect(connection_string, baud=baud_rate, wait_ready=True)     #- wait_ready flag hold the program untill all the parameters are been read (=, not .)

#-- Read information from the autopilot:
#- Version and attributes
vehicle.wait_ready('autopilot_version')
print('Autopilot version: %s'%vehicle.version)

#- Does the firmware support the companion pc to set the attitude?
print('Supports set attitude from companion: %s'%vehicle.capabilities.set_attitude_target_local_ned)

#- Read the actual position
print('Position: %s'% vehicle.location.global_relative_frame)

#- Read the actual attitude roll, pitch, yaw
print('Attitude: %s'% vehicle.attitude)

#- Read the actual velocity (m/s)
print('Velocity: %s'%vehicle.velocity) #- North, east, down

#- When did we receive the last heartbeat
print('Last Heartbeat: %s'%vehicle.last_heartbeat)

#- Is the vehicle good to Arm?
print('Is the vehicle armable: %s'%vehicle.is_armable)

#- Which is the total ground speed?   Note: this is settable
print('Groundspeed: %s'% vehicle.groundspeed) #(%)

#- What is the actual flight mode?    Note: this is settable
print('Mode: %s'% vehicle.mode.name)

#- Is the vehicle armed               Note: this is settable
print('Armed: %s'%vehicle.armed)

#- Is thestate estimation filter ok?
print('EKF Ok: %s'%vehicle.ekf_ok)



#----- Adding a listener
def altitude_callback(self, attr_name, value):
    print(vehicle.attitude)

def location_callback(self, attr_name, msg):
    print("Location (Global): ", msg)

print("")
print("Adding an altitude listener")
vehicle.add_attribute_listener('attitude', altitude_callback) #-- message type, callback function
# time.sleep(1) #-- Print data for 1 seconds

#--- Now we print the attitude from the callback for 5 seconds, then we remove the callback
vehicle.remove_attribute_listener('attitude', altitude_callback) #(.remove)

#-- Do same listener process for location
print("Adding a GPS location listener")
vehicle.add_attribute_listener('GLOBAL_POSITION_INT', location_callback)
time.sleep(2)
vehicle.remove_attribute_listener('GLOBAL_POSITION_INT', location_callback)
print("done with gps")

print("Add GPS try 2")
print("Global Location: %s" % vehicle.location.global_frame)
print("done try 2")

#--- You can create a callback even with decorators, check the documentation out for more details



#---- PARAMETERS
#print("Maximum Throttle: %d"%vehicle.parameters['THR_MIN']) 

#-- You can read and write the parameters
#vehicle.parameters['THR_MIN'] = 50
#time.sleep(1)
#print("Maximum Throttle: %d"%vehicle.parameters['THR_MIN'])



#--- Now we close the simulation
vehicle.close()

print("done")


