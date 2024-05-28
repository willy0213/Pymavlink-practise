import time
import pymavlink.mavutil as utility
import pymavlink.dialects.v20.all as dialect

def task_1():
    for i in range(5):
        print("Running task 1...")
        time.sleep(0.1)

def task_2():
    for i in range(3):
        print("Running task 2...")
        time.sleep(0.1)

# connect to vehicle
vehicle = utility.mavlink_connection(device="udpin:127.0.0.1:14550")

# wait for a heartbeat
vehicle.wait_heartbeat()

# inform user
print("Connected to system:", vehicle.target_system, ", component:", vehicle.target_component)

"""

# infinite loop (read RC channel values)
while True:

    # capture RC_CHANNELS message
    message = vehicle.recv_match(type=dialect.MAVLink_rc_channels_message.msgname,
                                 blocking=True)
    
    # convert the message to dictionary
    message = message.to_dict()

    # check 16th channel's PWM value
    if 1200 < message["chan16_raw"] < 1500:

        # call task 1
        task_1()
    
    elif 1500 < message["chan16_raw"] < 1700:

        # call task 2
        task_2()

    else:

        # do nothing
        print("Invalid channel 16 value:", message["chan16_raw"])

"""


# infinite loop (read servo values)
while True:

    # capture RC_CHANNELS message
    message = vehicle.recv_match(type=dialect.MAVLink_servo_output_raw_message.msgname,
                                 blocking=True)
    
    # convert the message to dictionary
    message = message.to_dict()

    # print the message
    print("Motor 1:", message["servo1_raw"], 
          "Motor 2:", message["servo2_raw"], 
          "Motor 3:", message["servo3_raw"], 
          "Motor 4:", message["servo4_raw"])