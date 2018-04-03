# Kendric Hood, Betis Baheri, Emil Shirima, Govinda Baweja
# Drone Programming CS 49995-013/ CS 59995-013
# Project 1.2
#
# Write a program to maneuver your drone vehicle as outlined below:
#
# I:    Launch the drone vehicle from the Home Location A to an ascending target Location B with gradually increasing velocity
# II:   Control the drone to stop at Location B for 10 seconds
# III:  Control the drone to continue at a higher latitude/longitude to a Location C
# IV:   Control the drone to stop at Location C for 5 seconds
# V:    Control the drone to continue at the same longitude, but increased latitude to Location D
# VI:   Control the drone to wait for 5 seconds at Location D
# VII:  Control the drone to gradually descend from Location D to a new Location E, where E has the same longitude as B, and same latitude as D
# VIII: Control the drone to gradually descend from Location E to its Home Location A


from __future__ import print_function
from MAVLinkCommands import *
import time

mav_steup()
set_groundspeed(10)

# Start part I and II
# Location B

global vehicle
b_lat = vehicle.location.global_relative_frame.lat + 0.005
b_lon = vehicle.location.global_relative_frame.lat + 0.005
b_alt = 10

arm_and_takeoff(b_alt)
local_goto(b_lat, b_lon, b_alt)
time.sleep(10)

# Start part III and IV
# Location C
c_lat = b_lat + 0.005
c_lon = b_lon + 0.005
c_alt = 10

local_goto(c_lat, c_lon, c_alt)
time.sleep(5)

# Start part V and VI
# Location D
d_lat = c_lat + 0.005
d_lon = c_lon
d_alt = 10

local_goto(d_lat, d_lon, d_alt)
time.sleep(5)

# Start part VII
# Location E
e_lat = d_lat
e_lon = b_lon
e_alt = 10

local_goto(e_lat, e_lon, e_alt)
