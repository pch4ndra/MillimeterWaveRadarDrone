# MillimeterWaveRadarDrone

Radar-based Drone Tracking: The key requirement of drone flight controls is a precise six degree of freedom (6-DoF) tracking system that provides a drone with its real-time location and orientation in 3D space. This project explores a sensor fusion approach including camera, radars, and IMU to provide millimeter-level tracking accuracy.

## File Organization

### ./archive/

Here we have old scripts (including old flight scripts and other data storing files) that I used as research notes/code when starting the project. They exist only for storing old code and serve 0 purpose when running the overall program for the radar.

### ./data/

This folder will contain all data that we store from the program. For example, under the camera subdirectory, we store all images taken by the PiCamera. Under the telemetry subdirectory, we will store CSV and MAT files that store all telemetry info of the drone.

### ./library/

This folder contains the threading class for the PiCamera, Telemetry, and Drone Flight Path Scripts. Since each different thread needs to built in its own class, I placed each one in a different python module and placed the data collection threads under ./library/DataScripts and the flight path scripts under ./library/FlightScripts.
#### Templates
There are two files under this folder with "template". These are template frameworks that can be used to develop future functionalities (ie Radar Software) that need to be performed in threads. The flight_template.py is used to add more predetermined flight paths and data_template.py should be used for any data that needs to be collected during the duration of the flight.

### drone.py

This is the central module for the project. I will explain installation instructions later that will clarify this script.

## Code Installation / Execution

If connecting to the RaspberryPi through an SSH connection, run:

```ssh
ssh pi@[IP ADDRESS OF RPI]
```

Next, the only script we need to run here is drone.py.
To run the script in a simulation, run

```py
python drone.py --sitl
```

To run the script on an actual drone, simply run

```py
python drone.py
```

## Files

There are no input files that need to be placed in the project except for the code for whatever radar that needs to be used. For output files, there are some data files that are created throughout the script that will be placed under ./data/.

## Hardcoded Addresses and External Connections

There is one place that an address has been hardcoded. In drone.py (lines 65-71), I have hardcoded the connection string. This will change depending on how we connect the PX4 to the RaspberryPi. If a USB is used, we leave it as is. If a serial connection is used, comment the line 69 and uncomment line 70.

## External Connections

For external connections, we simply need to ensure that the RaspberryPi is connected to the PX4 as specified under "Hardcoded Addresses and External Connections." It is also necessary that a PiCamera is attached to the RaspberryPi through a standard connection. Other than this, the drone must be built through the standard process.
