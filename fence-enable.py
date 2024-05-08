import sys
import time
import pymavlink.mavutil as utility
import pymavlink.dialects.v20.all as dialect

# fence enable definition
fence_enable_definition = {"DISABLE": 0,
                           "ENABLE": 1,
                           "DISABLE_FLOOR_ONLY": 2}

# get the first argument
fence_enable = sys.argv[1].upper()

if fence_enable in fence_enable_definition.keys():
    print("Sending FENCE {0} to the vehicle".format(fence_enable))
else:
    print("Not supported operation")
    sys.exit()

# connect to vehicle
vehicle = utility.mavlink_connection(device="udpin:127.0.0.1:14550")

# wait for a heartbeat
vehicle.wait_heartbeat()

# inform user
print("Connected to system:", vehicle.target_system, ", component:", vehicle.target_component)

# create the message
message = dialect.MAVLink_command_long_message(target_system=vehicle.target_system,
                                               target_component=vehicle.target_component,
                                               command=dialect.MAV_CMD_DO_FENCE_ENABLE,
                                               confirmation=0,
                                               param1=fence_enable_definition[fence_enable],
                                               param2=0,
                                               param3=0,
                                               param4=0,
                                               param5=0,
                                               param6=0,
                                               param7=0)

# send the message to the vehicle
vehicle.mav.send(message)