import time
import pymavlink.mavutil as utility
import pymavlink.dialects.v20.all as dialect

# create mission item list
target_locations = ((-35.361297, 149.161120, 50.0),
                    (-35.360780, 149.167151, 50.0),
                    (-35.365115, 149.167647, 50.0),
                    (-35.364419, 149.161575, 50.0))

# connect to vehicle
vehicle = utility.mavlink_connection(device="udpin:127.0.0.1:14550")

# wait for a heartbeat
vehicle.wait_heartbeat()

# inform user
print("Connected to system:", vehicle.target_system, ", component:", vehicle.target_component)

# create mission count message
message = dialect.MAVLink_mission_count_message(target_system=vehicle.target_system,
                                                target_component=vehicle.target_component,
                                                 count=len(target_locations) + 2,
                                                 mission_type=dialect.MAV_MISSION_TYPE_MISSION)

# send mission count message to the vehicle
vehicle.mav.send(message)

# this loop will run until receive a valid MISSION_ACK message
while True:
    # catch a message
    message = vehicle.recv_match(blocking=True)

    # convert this message to dictionary
    message = message.to_dict()

    # check this message is MISSION_REQUEST
    if message["mavpackettype"] == dialect.MAVLink_mission_request_message.msgname:
        print(message)

        # check this request is for mission item
        if message["mission_type"] == dialect.MAV_MISSION_TYPE_MISSION:

            # get the sequence number of requested mission item
            seq = message["seq"]

            # create mission item int message
            if seq == 0:
                # create mission item int message that contains the home location (0th mission item)
                message = dialect.MAVLink_mission_item_int_message(target_system=vehicle.target_system,
                                                                   target_component=vehicle.target_component,
                                                                   seq=seq,
                                                                   frame=dialect.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                                                   command=dialect.MAV_CMD_NAV_WAYPOINT,
                                                                   current=0,
                                                                   autocontinue=0,
                                                                   param1=0,
                                                                   param2=0,
                                                                   param3=0,
                                                                   param4=0,
                                                                   x=0,
                                                                   y=0,
                                                                   z=0,
                                                                   mission_type=dialect.MAV_MISSION_TYPE_MISSION,
                                                                   )
            # send takeoff mission item
            elif seq == 1:
                # create mission item int message that contains the takeoff command
                message = dialect.MAVLink_mission_item_int_message(target_system=vehicle.target_system,
                                                                   target_component=vehicle.target_component,
                                                                   seq=seq,
                                                                   frame=dialect.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                                                   command=dialect.MAV_CMD_NAV_TAKEOFF,
                                                                   current=0,
                                                                   autocontinue=0,
                                                                   param1=0,
                                                                   param2=0,
                                                                   param3=0,
                                                                   param4=0,
                                                                   x=0,
                                                                   y=0,
                                                                   z=target_locations[0][2],
                                                                   mission_type=dialect.MAV_MISSION_TYPE_MISSION,
                                                                   )
            # send target location to the vehicle
            else:

                # create mission item int message that contains a target location
                message = dialect.MAVLink_mission_item_int_message(target_system=vehicle.target_system,
                                                                   target_component=vehicle.target_component,
                                                                   seq=seq,
                                                                   frame=dialect.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                                                   command=dialect.MAV_CMD_NAV_WAYPOINT,
                                                                   current=0,
                                                                   autocontinue=0,
                                                                   param1=0,
                                                                   param2=0,
                                                                   param3=0,
                                                                   param4=0,
                                                                   x=int(target_locations[seq-2][0] * 1e7),
                                                                   y=int(target_locations[seq-2][1] * 1e7),
                                                                   z=target_locations[seq-2][2],
                                                                   mission_type=dialect.MAV_MISSION_TYPE_MISSION,
                                                                   )


            # send mission item int message to the vehicle
            vehicle.mav.send(message)

    # check this message is MISSION_ACK
    elif message["mavpackettype"] == dialect.MAVLink_mission_ack_message.msgname:
        
        # check this acknowledgement is for mission and it is accepted
        if message["type"] == dialect.MAV_MISSION_ACCEPTED and message["mission_type"] == dialect.MAV_MISSION_TYPE_MISSION:

            # break the loop since the upioad is successful
            print("Mission upload is successful")
            break