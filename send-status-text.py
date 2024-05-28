import time
import random
import pymavlink.mavutil as utility
import pymavlink.dialects.v20.all as dialect

# create a source component
my_process = utility.mavlink_connection(device="udpout:127.0.0.1:14550",
                                        source_system=1,
                                        source_component=1)

# inform user
print("Serving as a system:", my_process.mav.srcSystem, ", component:", my_process.mav.srcComponent)

# create an infinite loop
while True:

    # create the text
    text = f"Roll a dice: {random.randint(1, 6)} flip a coin: {random.randint(0, 1)}"

    # create STATUSTEXT message
    message = dialect.MAVLink_statustext_message(severity=dialect.MAV_SEVERITY_INFO,
                                                 text="Hello, My name is Willy.".encode('utf-8'))
    
    # send message to the GCS
    my_process.mav.send(message)

    # sleep a bit
    time.sleep(5)