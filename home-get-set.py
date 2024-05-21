import time
import pymavlink.mavutil as utility
import pymavlink.dialects.v20.all as dialect

# new home location
new_home_location = (-35.36210556, 149.16373661, 583.9)

# connect to vehicle
vehicle = utility.mavlink_connection(device="udpin:127.0.0.1:14550")

# wait for a heartbeat
vehicle.wait_heartbeat()

"""
# create home position request message
command = dialect.MAVLink_command_long_message(target_system=vehicle.target_system,
                                               target_component=vehicle.target_component,
                                               command=dialect.MAV_CMD_SET_MESSAGE_INTERVAL,
                                               confirmation=0,
                                               param1=dialect.MAVLINK_MSG_ID_HOME_POSITION,
                                               param2=1e6,
                                               param3=0,
                                               param4=0,
                                               param5=0,
                                               param6=0,
                                               param7=0)

# send command to the vehicle
vehicle.mav.send(command)

"""

# create set home position command
command = dialect.MAVLink_command_long_message(target_system=vehicle.target_system,
                                               target_component=vehicle.target_component,
                                               command=dialect.MAV_CMD_DO_SET_HOME,
                                               confirmation=0,
                                               param1=0,
                                               param2=0,
                                               param3=0,
                                               param4=0,
                                               param5=new_home_location[0],
                                               param6=new_home_location[1],
                                               param7=new_home_location[2])

# send command to the vehicle
vehicle.mav.send(command)

# inform user
print("Connected to system:", vehicle.target_system, ", component:", vehicle.target_component)

# infinite loop
while True:
    
    # get HOME_POSITION message
    message = vehicle.recv_match(type=dialect.MAVLink_home_position_message.msgname,
                                 blocking=True)
    
    # convert message to dictionary
    message = message.to_dict()

    # debug the message
    print("Home Position >",
          "Latitude:", message["latitude"] * 1e-7,
          "Longitude:", message["longitude"] * 1e-7,
          "Altitude:", message["altitude"] * 1e-3)
    
    """
    # disable HOME_POSITION message stream
    command = dialect.MAVLink_command_long_message(target_system=vehicle.target_system,
                                                target_component=vehicle.target_component,
                                                command=dialect.MAV_CMD_SET_MESSAGE_INTERVAL,
                                                confirmation=0,
                                                param1=dialect.MAVLINK_MSG_ID_HOME_POSITION,
                                                param2=-1,
                                                param3=0,
                                                param4=0,
                                                param5=0,
                                                param6=0,
                                                param7=0)
    
    # send command to the vehicle
    vehicle.mav.send(command)

    """