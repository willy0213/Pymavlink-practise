import pymavlink.mavutil as utility
import pymavlink.dialects.v20.all as dialect
import getkey
import time

def override_channels(vehicle, channels):
    temp_channels = [65535] * 18
    for i in range(len(temp_channels)):
        if i + 1 in channels.keys():
            temp_channels[i] = channels[i + 1]
    channels = temp_channels
    return dialect.MAVLink_rc_channels_override_message(vehicle.target_system, 
                                                        vehicle.target_component,
                                                        *channels)

def safe_pwm(value, offset):
    value = value + offset
    if value < 1000:
        value = 1000
    if value > 2000:
        value = 2000
    return value

# connect to vehicle
vehicle = utility.mavlink_connection(device="udpin:127.0.0.1:14550")

# wait for a heartbeat
vehicle.wait_heartbeat()

# inform user
print("Connected to system:", vehicle.target_system, ", component:", vehicle.target_component)

# initialize channels
channels = {1: 1500, 2:1500, 3: 1500, 4:1500}

while True:

    # capture key presses
    pressed_key = getkey.getkey()

# check pressed key
    if pressed_key == "f":
        channels[1] = safe_pwm(channels[1], -100)
        print("Roll left")
    elif pressed_key == "h":
        channels[1] = safe_pwm(channels[1], 100)
        print("Roll right")
    elif pressed_key == "w":
        channels[2] = safe_pwm(channels[2], -100)
        print("Move forward")
    elif pressed_key == "s":
        channels[2] = safe_pwm(channels[2], 100)
        print("Move backward")
    elif pressed_key == "t":
        channels[3] = safe_pwm(channels[3], 100)
        print("Increase throttle")
    elif pressed_key == "g":
        channels[3] = safe_pwm(channels[3], -100)
        print("Decrease throttle")
    elif pressed_key == "a":
        channels[4] = safe_pwm(channels[4], -100)
        print("Yaw left")
    elif pressed_key == "d":
        channels[4] = safe_pwm(channels[4], 100)
        print("Yaw right")

    # debug the channels
    print(channels)

    # create rc channels override message
    message = override_channels(vehicle=vehicle, channels=channels)

    # send the message to the vehicle
    vehicle.mav.send(message)

