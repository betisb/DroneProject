#!/usr/bin/env python
# -*- coding: utf-8 -*-
#Kendric Hood, Betis Baheri, Emil Shirima, Govinda Baweja

#Launch the drone vehicle from the Home Location A to an ascending target Location B with gradually increasing velocity
# Control the drone to stop at B for 10 seconds
# Control the drone to descend from Location B at a gradually decreased velocity to its Home Location A


from __future__ import print_function
from MAVLinkCommands import *
import time

lat=-35.361879
lon=149.166033
alt=10
speed=10

mav_steup()
arm_and_takeoff(alt)
set_airspeed(speed)
local_goto(lat,lon,alt)
time.sleep(10)
return_to_launch()
mav_cleanup()