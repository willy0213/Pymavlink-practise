import pymavlink.mavutil as utility
import pymavlink.dialects.v20.all as dialect
import geopy.distance

# connect to vehicle
vehicle = utility.mavlink_connection(device="udpin:127.0.0.1:14550")

# wait for a heartbeat
vehicle.wait_heartbeat()

# inform user
print("Connected to system:", vehicle.target_system, ", component:", vehicle.target_component)

# build home position request command
message = dialect.MAVLink_command_long_message(target_system=vehicle.target_system,
                                               target_component=vehicle.target_component,
                                               command=dialect.MAV_CMD_REQUEST_MESSAGE,
                                               confirmation=0,
                                               param1=dialect.MAVLINK_MSG_ID_HOME_POSITION,
                                               param2=0,
                                               param3=0,
                                               param4=0,
                                               param5=0,
                                               param6=0,
                                               param7=0)

# send the command to the vehicle
vehicle.mav.send(message)

# get home position
message = vehicle.recv_match(type=dialect.MAVLink_home_position_message.msgname, blocking=True)

# convert the message to dictionary
message = message.to_dict()

# get home location
home_location = (message["latitude"]*1e-7, message["longitude"]*1e-7)

# debug the home location
print("Home location:", home_location)

# infinite loop the catch vehicle global position
while True:

    # get current vehicle position
    message = vehicle.recv_match(type=dialect.MAVLink_global_position_int_message.msgname, blocking=True)

    # convert the message to dictionary
    message = message.to_dict()

    # get vehicle location
    vehicle_location = (message["lat"]*1e-7, message["lon"]*1e-7)

    # calculate home distance
    home_distance = geopy.distance.GeodesicDistance(home_location, vehicle_location).meters

    # debug the vehicle location
    print("Vehicle location:", vehicle_location, "Home distance:", home_distance)
