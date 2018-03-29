# Author: Emil Shirima
# Date: Monday 26th March 2018
# Group Project Assignment II section I

from cop_main import *
from dronekit import LocationLocal

def altitude_listener(self, _, value):
    print 'Altitude: {}'.format(value.alt)

def heart_beat_listener(self, _, value):
    print 'Heartbeat: {}'.format(value)

copter = Copter()

cop_object, error = copter.create_session()

if error is not None:
    raise SystemExit(error)

status = copter.init_launch_sequence()

cop_object.location.add_attribute_listener('global_relative_frame', altitude_listener)
cop_object.add_message_listener('HEARTBEAT', heart_beat_listener)
copter.add_call_back('global_relative_frame', altitude_listener)
time.sleep(2)

if not status.success:
    raise SystemExit(error)

status = copter.rise_to_height()

if not status.success:
    raise SystemExit(error)

loc_e = LocationLocal(0, 0, 0)

copter.move_to_location_B()
copter.circle_around()

# this value will be used to calculate the final target position
loc_e.north = 0 - cop_object.location.local_frame.north

copter.move_to_location_C()
copter.circle_by_turns()

time.sleep(10)

copter.move_to_location_D()
copter.circle_by_turns(turns=4, radius=5)

# calculate GPS point (lat, lon) of final point
loc_e.down = cop_object.location.local_frame.down
final_loc = copter.get_location_from_ned(cop_object.location.global_relative_frame, loc_e)

time.sleep(10)

copter.move_to_location_E(final_loc)
copter.circle_around(radius=5, duration=20, change_velocity=False, factor=0)

copter.return_to_launch_pad()

time.sleep(180)

copter.shutdown()