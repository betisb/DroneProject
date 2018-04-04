#Kendric Hood, Betis Baheri, Emil Shirima, Govinda Baweja, Afrah Arishi
#Drone Programming CS 49995-013/ CS 59995-013
#Project 1.1

#Launch the drone vehicle from the Home Location A to an ascending target Location B with gradually increasing velocity
# Control the drone to stop at B for 10 seconds
# Control the drone to descend from Location B at a gradually decreased velocity to its Home Location A


from __future__ import print_function
from MAVLinkCommands import *
import time


lat = get_lat() + 0.005
lon = get_lon() + 0.005
alt = 10
speed = 10

mav_steup()
arm_and_takeoff(alt)
set_airspeed(speed)
local_goto(lat, lon, alt)
time.sleep(10)
return_to_launch()
mav_cleanup()
