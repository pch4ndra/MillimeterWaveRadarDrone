from dronekit import connect, VehicleMode
import time

# -- Declare Connection System
connection_string = "/dev/ttyACM0" # USB Connection
baud_rate = 115200
# connection_string = "/dev/ttyAMA0" # Serial Connection

print(">>>> Connecting with the UAV <<<<")
vehicle = connect(connection_string, baud=baud_rate)

for i in range(120):
    print("Vehicle's Latitude =>  ", vehicle.location.global_relative_frame.lat)
    print("Vehicle's Longitude => ", vehicle.location.global_relative_frame.lon)

    print("GPS Info => ", vehicle.GPSInfo())

    time.sleep(1)

print("Done")
