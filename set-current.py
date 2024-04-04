import sys
import time
import pymavlink.mavutil as utility
import pymavlink.dialects.v20.all as dialect

# get the desired sequence number
seq_desired = int (sys.argv[1])      # 在python set-current 指令後面加上一個數字 例如：2 會強迫飛機中止原本的點 直接飛到你指定的數字的點（順序）上

# debug the desired sequence number
print("Desired mission item:", seq_desired)

# connect to vehicle
vehicle = utility.mavlink_connection(device='udpin:127.0.0.1:14550')

# wait for heartbeat
vehicle.wait_heartbeat()

# inform user
print("Connected to system:", vehicle.target_system, ", component:", vehicle.target_component) 

# wait until receive MISSION_CURRENT message
message = vehicle.recv_match(type=dialect.MAVLink_mission_current_message.msgname,blocking=True)

# convert message to dictionary
message = message.to_dict()

# get current sequence number
seq_current = message["seq"]

# debug the message
print("Current mission item:", seq_current)

# create MAV_CMD_DO_SET_MISSION_CURRENT command
message = dialect.MAVLink_command_long_message(target_system=vehicle.target_system,
                                               target_component=vehicle.target_component,
                                               command=dialect.MAV_CMD_DO_SET_MISSION_CURRENT,
                                               confirmation=0,
                                               param1=seq_desired,
                                               param2=0,
                                               param3=0,
                                               param4=0,
                                               param5=0,
                                               param6=0,
                                               param7=0)

# send this message to the vehicle
vehicle.mav.send(message)

# wait until receive MISSION_CURRENT message
message = vehicle.recv_match(type=dialect.MAVLink_mission_current_message.msgname,blocking=True)

# convert message to dictionary
message = message.to_dict()

# get current sequence number
seq_current = message["seq"]

# debug the message
print("Current mission item:", seq_current)

if seq_desired == seq_current:
    print("Set current mission item is successful")
else:
    print("Failed to set current mission item")

