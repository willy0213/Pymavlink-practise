import pymavlink.mavutil as utility
import pymavlink.dialects.v20.all as dialect
import time
 
#connect to vehicle
vehicle=utility.mavlink_connection(device="udpin:127.0.0.1:14560") 

#wait for a heartbeat
vehicle.wait_heartbeat()

#inform user
print("Connected to system: ",vehicle.target_system,", component:",vehicle.target_component)

#infinite loop
while True:
    try:
        message=vehicle.recv_match(type=dialect.MAVLink_system_time_message.msgname,blocking=True)
        message=message.to_dict()
        for fieldname in dialect.MAVLink_system_time_message.fieldnames:
            if fieldname == "time_boot_ms":
                print(fieldname,message[fieldname])
    except:
        print("no message received from vehicle")

    time.sleep(0.1)