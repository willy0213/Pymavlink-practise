import time
import pymavlink.mavutil as utility
import pymavlink.dialects.v20.all as dialect
import geopy.distance

# Start a connection listening to a UDP port
vehicle = utility.mavlink_connection(device="udpin:127.0.0.1:14560")

# Wait for the first heartbeat
# This sets the system and component ID of the remote system for the link
vehicle.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (vehicle.target_system, vehicle.target_component))

vehicle.mav.send(utility.mavlink.MAVLink_set_position_target_local_ned_message(10,vehicle.target_system,
                        vehicle.target_component,utility.mavlink.MAV_FRAME_LOCAL_NED,int(0b110111111000) ,10,0,-10,0,0,0,0,0,0,-1.57,0.5))