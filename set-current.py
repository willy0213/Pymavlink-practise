import sys
import time
import pymavlink.mavutil as utility
import pymavlink.dialects.v20.all as dialect

vehicle=utility.mavlink_connection(device="udpin:127.0.0.1:14560")

vehicle.wait_heartbeat()

print("Connection to system:",vehicle.target_system, ", component:",vehicle.target_component)