import pymavlink.mavutil as utility
import pymavlink.dialects.v20.all as dialect

# helper variables
PAUSE = 0
RESUME = 1

# connect to vehicle
vehicle = utility.mavlink_connection(device="udpin:127.0.0.1:14550")

# wait for a heartbeat
vehicle.wait_heartbeat()

# inform user
print("Connected to system:", vehicle.target_system, ", component:", vehicle.target_component)

# create the command
command = dialect.MAVLink_command_long_message(target_system=vehicle.target_system,
                                               target_component=vehicle.target_component,
                                               command=dialect.MAV_CMD_DO_PAUSE_CONTINUE,
                                               confirmation=0,
                                               param1=PAUSE,
                                               param2=0,
                                               param3=0,
                                               param4=0,
                                               param5=0,
                                               param6=0,
                                               param7=0)

# send this command to the vehicle
vehicle.mav.send(command)