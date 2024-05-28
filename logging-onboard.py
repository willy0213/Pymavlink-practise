import time
import random
import pymavlink.mavutil as utility
import pymavlink.dialects.v20.all as dialect

# connect to vehicle
vehicle = utility.mavlink_connection(device="udpin:127.0.0.1:14560",
                                     source_system=1,
                                     source_component=1)

# wait for a heartbeat
vehicle.wait_heartbeat()

# inform user
print("Connected to system:", vehicle.target_system, ", component:", vehicle.target_component)

# create an infinite loop
while True:

    # create STATUSTEXT message
    text = f"CUSTOM:{random.randint(1,6)},{random.randint(0,1)},{random.random()}"
    message = dialect.MAVLink_statustext_message(severity=dialect.MAV_SEVERITY_DEBUG,
                                                 text=text.encode("utf-8"))
    print(text)

    # send the message to the vehicle
    vehicle.mav.send(message)

    # wait for 1 second
    time.sleep(1)

"""
# open the log file
with open("logdata.txt", "r") as file:

    # read all lines
    lines = file.readlines()

    # iterate over lines
    for line in lines:

        # check if the log is for our CUSTOM data
        if "CUSTOM" in line:

            # filter the custom data
            line = line.split("CUSTOM:")[1].split("}")[0].split(",")
            dice, coin, my_float = int(line[0]), int(line[1]), float(line[2])

            # debug the result
            print("Dice:", dice, "Coin:", coin, "Float:", my_float)
"""