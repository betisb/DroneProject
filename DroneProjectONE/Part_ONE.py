#!/usr/bin/env python
# -*- coding: utf-8 -*-
#Kendric Hood, Betis Baheri, Emil Shirima, Govinda Baweja

from __future__ import print_function
from MAVLinkCommands import *

lat=-35.361879
lon=149.166033
alt=10
speed=10

mav_steup()
arm_and_takeoff(alt)
set_airspeed(speed)
local_goto(lat,lon,alt)
land_here()
mav_cleanup()