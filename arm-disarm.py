import pymavlink.mavutil as utility
import pymavlink.dialects.v20.all as dialect
import time

# arm disarm definitions
VEHICLE_ARM = 1
VEHICLE_DISARM = 0

# connect to vehicle
vehicle = utility.mavlink_connection(device="udpin:127.0.0.1:14550")

# vehicle arm message
vehicle_arm_message = dialect.MAVLink_command_long_message(
    target_system = vehicle.target_system,
    target_component = vehicle.target_component,
    command=dialect.MAV_CMD_COMPONENT_ARM_DISARM,
    confirmation=0,
    param1=VEHICLE_ARM,
    param2=0,
    param3=0,
    param4=0,
    param5=0,
    param6=0,
    param7=0
)

# vehicle disarm message
vehicle_disarm_message = dialect.MAVLink_command_long_message(
    target_system = vehicle.target_system,
    target_component = vehicle.target_component,
    command=dialect.MAV_CMD_COMPONENT_ARM_DISARM,
    confirmation=0,
    param1=VEHICLE_DISARM,
    param2=0,
    param3=0,
    param4=0,
    param5=0,
    param6=0,
    param7=0
)

# wait for a heartbeat
vehicle.wait_heartbeat()

# inform user
print("Connected to system:", vehicle.target_system, ", component:", vehicle.target_component)

# check the pre-arm 
while True:

    # observe the SYS_STATUS message
    message = vehicle.recv_match(type = dialect.MAVLink_sys_status_message.msgname, blocking = True)

    # convert to dictionary
    message = message.to_dict()

    # get sensor health
    onboard_control_sensors_health = message['onboard_control_sensors_health']

    # get pre-arm healthy bit
    prearm_status = onboard_control_sensors_health & dialect.MAV_SYS_STATUS_PREARM_CHECK == dialect.MAV_SYS_STATUS_PREARM_CHECK

    if  prearm_status:

        # vehicle can be armable
        print("vehicle is armable")

        # break the prearm check loop
        break

# do below always
while True:

    # arm the vehicle
    print("vehicle is arming....")  

    # send arm message
    vehicle.mav.send(vehicle_arm_message)

    # wait COMMAND_ACK MESSAGE
    message = vehicle.recv_match(type = dialect.MAVLink_command_ack_message.msgname, blocking=True)

    # convert the message to dictionary
    message = message.to_dict()

    # check if the vehicle is armed
    if message['result'] == dialect.MAV_RESULT_ACCEPTED:

        # print that vehicle is armed
        print("Vehicle is armed!")

    else:

        # print that vehicle is not armed
        print("Vehicle is not armed!")

    # wait some time
    time.sleep(10)

    # disarm the vehicle
    print("vehicle is disarming....")

    # send disarm message
    vehicle.mav.send(vehicle_disarm_message)

    # check if the vehicle is armed
    if message['result'] == dialect.MAV_RESULT_ACCEPTED:

        # print that vehicle is disarmed
        print("Vehicle is disarmed!")

    else:

        # print that vehicle is not disarmed
        print("Vehicle is not disarmed!")

    # wait some time
    time.sleep(10)
