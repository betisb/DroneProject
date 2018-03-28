#!/usr/bin/env python
# -*- coding: utf-8 -*-


from __future__ import print_function
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative , Command , LocationGlobal
from pymavlink import mavutil
import math
import subprocess
# Set up option parsing to get connection string
import argparse
parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
parser.add_argument('--connect',
                    help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect
sitl = None


# Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()


# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)
print("Basic pre-arm checks")
# Don't try to arm until autopilot is ready
while not vehicle.is_armable:
    print(" Waiting for vehicle to initialise...")
    time.sleep(2)

print("Arming motors")
# Copter should arm in GUIDED mode
vehicle.mode = VehicleMode("GUIDED")
vehicle.armed = True
# Confirm vehicle armed before attempting to take off
while not vehicle.armed:
    print(" Waiting for arming...")
    time.sleep(2)
print("Taking off!")
altitude = float(0.5)
if math.isnan(altitude) or math.isinf(altitude):
    raise ValueError("Altitude was NaN or Infinity. Please provide a real number")
vehicle._master.mav.command_long_send(0, 0, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
0, 0, 0, 0, 0, 0, 0, altitude)
while True:
    print(" Altitude: ", vehicle.location.global_relative_frame.alt)
    # Break and return from function just below target altitude.
    if vehicle.location.global_relative_frame.alt >= altitude * 0.95:
        print("Reached target altitude")
        break
    time.sleep(8)
# msg = vehicle.message_factory.command_long_encode(
#         0, 0,    # target system, target component
#         mavutil.mavlink.MAV_CMD_DO_SET_HOME, #command
#         0, #confirmation
#         1, #param 1: 1 to use current position, 2 to use the entered values.
#             0, 0, 0, #params 2-4
#         vehicle.location.lat,
#         vehicle.location.lon,
#         vehicle.location.alt
#         )
lat=-35.361879
lon=149.166033
frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
alt = 30
vehicle._master.mav.mission_item_send(0, 0, 0, frame,
                                           mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 2, 0, 0,
                                           0, 0, 0, lat, lon,
                                           alt)
speed_type = 0 # air speed
msg = vehicle.message_factory.command_long_encode(
    0, 0,    # target system, target component
    mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, #command
    0, #confirmation
    speed_type, #param 1
    5, # speed in metres/second
    -1, 0, 0, 0, 0 #param 3 - 7
    )
vehicle.send_mavlink(msg)
time.sleep(2)
# cmds = vehicle.commands
# cmds.download()
# cmds.wait_ready()
# cmds = vehicle.commands
# cmds.clear()
# cmd = Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_LOITER_TIME,
#     25, 0, 0, 0, 0, 0,
#     0, 0, 0)
# cmds.add(cmd)
# cmds.upload()
msg = vehicle.message_factory.command_long_encode(
    0, 0,    # target system, target component
    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
    mavutil.mavlink.MAV_CMD_NAV_LOITER_TIME,
    10, #param 1
    0,
    0, 0, 0, 0, 0
    )
vehicle.send_mavlink(msg)
time.sleep(2)
time.sleep(65)

vehicle._master.mav.command_long_send(0, 0,
                    mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0, 0, 0, 0, 0, 1)
print("Setting LAND mode...")
vehicle.mode = VehicleMode("LAND")
time.sleep(50)
print("Close vehicle object")
vehicle.close()

# Shut down simulator if it was started.
if sitl:
    sitl.stop()