import time
import pymavlink.mavutil as utility
import pymavlink.dialects.v20.all as dialect

# introduce FENCE_TOTAL as bybe array and do not use parameter index
FENCE_TOTAL = "FENCE_TOTAL".encode('utf-8')
PARAM_INDEX = -1

# connect to vehicle
vehicle = utility.mavlink_connection(device="udpin:127.0.0.1:14550")

# wait for a heartbeat
vehicle.wait_heartbeat()

# inform user
print("Connected to system:", vehicle.target_system, ", component:", vehicle.target_component)

# create PARAM_REQUEST_READ message
param_request_read_message = dialect.MAVLink_param_request_read_message(target_system=vehicle.target_system, 
                                                                        target_component=vehicle.target_component, 
                                                                        param_id=FENCE_TOTAL, 
                                                                        param_index=PARAM_INDEX)

# send PARAM_REQUEST_READ message TO vehicle
vehicle.mav.send(param_request_read_message)

# receive PARAM_VALUE message until get FENCE_TOTAL value
while True:

    message = vehicle.recv_match(type=dialect.MAVLink_param_value_message.msgname,
                                 blocking=True)
    
    # convert the messqage to dictionary
    message = message.to_dict()

    # check this parameter value message is for FENCE_TOTAL
    if message["param_id"] == "FENCE_TOTAL":

        # get total fence count number
        fence_count = int(message["param_value"])

        # break the loop
        break

# debug fence count
print("Total fence item count:", fence_count)

# crearte fence item list
fence_list = []

# create for loop to get the fence items
for idx in range(fence_count):
    
    # create FENCE_POINT_MESSAGE
    message = dialect.MAVLink_fence_fetch_point_message(target_system=vehicle.target_system,
                                                        target_component=vehicle.target_component,
                                                        idx=idx)
    
    # send this message to vehicle
    vehicle.mav.send(message)

    # wait until receive FENCE_POINT message
    message = vehicle.recv_match(type=dialect.MAVLink_fence_point_message.msgname,
                                blocking=True)
    
    # convert the message to dictionary
    message = message.to_dict()

    # append the fence point to the list
    fence_list.append((message["lat"], message["lng"]))

# for each fence item
for fence_item in fence_list:

    # debug the fence item
    print("Latitute:", fence_item[0], ", Longitude:", fence_item[1])