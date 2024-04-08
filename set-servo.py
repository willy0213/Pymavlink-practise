import time
import pymavlink.mavutil as utility
import pymavlink.dialects.v20.all as dialect

# connect to vehicle
vehicle = utility.mavlink_connection(device="udpin:127.0.0.1:14550")

# wait for a heartbeat
vehicle.wait_heartbeat()

# inform user
print("Connected to system:", vehicle.target_system, ", component:", vehicle.target_component)

"""
#create set servo command
command = dialect.MAVLink_command_long_message(target_system=vehicle.target_system,
                                               target_component=vehicle.target_component,
                                               command=dialect.MAV_CMD_DO_SET_SERVO,
                                               confirmation=0,
                                                param1=6,                  # servo編號
                                                param2=1100,               # 設定的pwm值
                                                param3=0,
                                                param4=0,
                                                param5=0,
                                                param6=0,
                                                param7=0)

# send the command to the vehicle
vehicle.mav.send(command)

"""
#create set servo command
command = dialect.MAVLink_command_long_message(target_system=vehicle.target_system,
                                               target_component=vehicle.target_component,
                                               command=dialect.MAV_CMD_DO_REPEAT_SERVO,
                                               confirmation=0,
                                                param1=6,                  # servo編號
                                                param2=1900,               # 設定的pwm值
                                                param3=5,                  # 循環次數
                                                param4=4,                  # 週期
                                                param5=0,
                                                param6=0,
                                                param7=0)

# send the command to the vehicle
vehicle.mav.send(command)
