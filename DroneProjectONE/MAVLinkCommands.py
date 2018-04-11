#Kendric Hood, Betis Baheri, Emil Shirima, Govinda Baweja, Afrah Arishi

#This is a set of functions that run the basic MAVLink commands
#Anyone can add to this or edit the functions as they see fit.

import time
from dronekit import connect, VehicleMode
from pymavlink import mavutil

#
#Setup the mavlink parse params and start SITL as well as the initlize the vehicle object
#
def mav_steup():
    # Set up option parsing to get connection string
    import argparse
    parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
    parser.add_argument('--connect',
                        help="Vehicle connection target string. If not specified, SITL automatically started and used.")
    args = parser.parse_args()

    connection_string = args.connect
    global sitl
    sitl= None

    # Start SITL if no connection string specified
    if not connection_string:
        import dronekit_sitl
        sitl = dronekit_sitl.start_default()
        connection_string = sitl.connection_string()

    # Connect to the Vehicle
    print('Connecting to vehicle on: %s' % connection_string)
    global vehicle
    vehicle = connect(connection_string, wait_ready=True)

#
#Close vehicle object and SITL
#
def mav_cleanup():
    # Close vehicle object before exiting script
    print("Close vehicle object")
    global vehicle
    vehicle.close()

    # Shut down simulator if it was started.
    global sitl
    if sitl:
        sitl.stop()

#
#Returns the Drone to its launch location
#
def return_to_launch():
    print("Landing at launch location")
    msg = vehicle.message_factory.command_long_encode(
        0,  # Target system
        0,  # Target comtroler
        mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,  # command
        0,  # frame
        0,  # param 1
        0,  # param 2
        0,  # param 3
        0,  # param 4
        0,  # param 5
        0,  # param 6
        0   # param 7
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()

    while(vehicle.location.global_relative_frame.alt > 0):
        if vehicle.location.global_relative_frame.alt == 0:
            print("Landed")
            break
        else:
            print("Lat:", vehicle.location.global_relative_frame.lat, ", Lon:",vehicle.location.global_relative_frame.lon, ", Alt:", vehicle.location.global_relative_frame.alt)
            time.sleep(1)
    print(" Altitude: ", vehicle.location.global_relative_frame.alt)
    time.sleep(5)

#
#Starts the Drone and starts the take off
#
def arm_and_takeoff(aTargetAltitude):
    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    time.sleep(2)
    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    msg = vehicle.message_factory.command_long_encode(
        0,#Target system
        0,#Target comtroler
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,#command
        0,#frame
        0,#param 1
        0,#param 2
        0,#param 3
        0,#param 4
        0,#param 5
        0,#param 6
        aTargetAltitude#param 7
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()
    while(vehicle.location.global_relative_frame.alt < aTargetAltitude):
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude:
            print("Reached target altitude")
            break
        else:
            print(" Altitude: ", vehicle.location.global_relative_frame.alt)
            time.sleep(1)
    print(" Altitude: ", vehicle.location.global_relative_frame.alt)
    time.sleep(5)

#
#Lands the drone at it's current location
#
def land_here():
    print("Setting LAND mode")
    vehicle.mode = VehicleMode("LAND")

    while (vehicle.location.global_relative_frame.alt > 0):
        if vehicle.location.global_relative_frame.alt == 0:
            print("Landed")
            break
        else:
            print(
            "Lat:", vehicle.location.global_relative_frame.lat, ", Lon:", vehicle.location.global_relative_frame.lon,
            ", Alt:", vehicle.location.global_relative_frame.alt)
            time.sleep(1)
    print(" Altitude: ", vehicle.location.global_relative_frame.alt)
    time.sleep(5)

#goto reltive to its local lat lon and alt
def local_goto(lat,lon,alt,holdTime):
    print("going to ",lat,", ",lon)
    vehicle._master.mav.mission_item_send(
        0,# Target system
        0,# Target comtroler
        0,# Waypoint number
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,  # frame
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,# command
        2,# current waypoint number
        0,# auto countue to next waypoint
        holdTime,# param 1
        0,# param 2
        0,# param 3
        0,# param 4
        lat,# param 5
        lon,# param 6
        alt# param 7
    )
    while((vehicle.location.global_relative_frame.lat > lat + 0.0000009 or vehicle.location.global_relative_frame.lat < lat - 0.0000009) and (vehicle.location.global_relative_frame.lon > lon + 0.0000009 or vehicle.location.global_relative_frame.lon < lon- 0.0000009)):
        print("Lat:",vehicle.location.global_relative_frame.lat,", Lon:",vehicle.location.global_relative_frame.lon, ", Alt:",vehicle.location.global_relative_frame.alt)
        time.sleep(1)

    print("Lat:", vehicle.location.global_relative_frame.lat, ", Lon:", vehicle.location.global_relative_frame.lon,", Alt:", vehicle.location.global_relative_frame.alt)
    time.sleep(5)

#
#goto reltive to sea level
#
def global_goto(lat,lon,alt,holdTime):
    print("going to ", lat, ", ", lon)
    vehicle._master.mav.mission_item_send(
        0,  # Target system
        0,  # Target comtroler
        0,  # Waypoint number
        mavutil.mavlink.MAV_FRAME_GLOBAL,  # frame
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,  # command
        2,  # current waypoint number
        0,  # auto countue to next waypoint
        holdTime,  # param 1
        0,  # param 2
        0,  # param 3
        0,  # param 4
        lat,  # param 5
        lon,  # param 6
        alt  # param 7
    )
    while ((vehicle.location.global_relative_frame.lat > lat + 0.0000009 or vehicle.location.global_relative_frame.lat < lat) and (vehicle.location.global_relative_frame.lon > lon + 0.0000009 or vehicle.location.global_relative_frame.lon < lon)):
        print("Lat:", vehicle.location.global_relative_frame.lat, ", Lon:", vehicle.location.global_relative_frame.lon,", Alt:", vehicle.location.global_relative_frame.alt)
        time.sleep(1)

    print("Lat:", vehicle.location.global_relative_frame.lat, ", Lon:", vehicle.location.global_relative_frame.lon,
          ", Alt:", vehicle.location.global_relative_frame.alt)
    time.sleep(5)

#
#sets the ground speed
#
def set_groundspeed(speed):
    msg = vehicle.message_factory.command_long_encode(
        0,# target system
        0,# target component
        mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, # command
        0, # confirmation
        1, # param 1 1=groundspeed
        speed, # param 2 speed in metres/second
        0,# param 3
        0,# param 4
        0,# param 5
        0,# param 6
        0 # param 7
        )

    # send command to vehicle
    vehicle.send_mavlink(msg)
    vehicle.flush()

#
#sets the air speed
#
def set_airspeed(speed):
    msg = vehicle.message_factory.command_long_encode(
        0,# target system
        0,# target component
        mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, # command
        0, # confirmation
        0, # param 1 0=airspeed
        speed, # param 2 speed in metres/second
        0,# param 3
        0,# param 4
        0,# param 5
        0,# param 6
        0 # param 7
        )

    # send command to vehicle
    vehicle.send_mavlink(msg)
    vehicle.flush()

# TESTING
# This method is incomplete
def set_home(lat,lon,alt):
    msg = vehicle.message_factory.command_long_encode(
        0,  # target system
        0,  # target component
        mavutil.mavlink.MAV_CMD_DO_SET_HOME,  # command
        0,  # confirmation
        2,  # param 1
        0,  # param 2
        0,  # param 3
        0,  # param 4
        lat,  # param 5
        lon,  # param 6
        alt  # param 7
    )

    # send command to vehicle
    vehicle.send_mavlink(msg)
    vehicle.flush()


#
# getters
#
def get_lat():
    return vehicle.location.global_relative_frame.lat

def get_lon():
    return vehicle.location.global_relative_frame.lon

def get_alt():
    return vehicle.location.global_relative_frame.alt
