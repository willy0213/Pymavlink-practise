import pymavlink.mavutil

vehicle=pymavlink.mavutil.mavlink_connection(device="udpin:127.0.0.1:14560")

vehicle.wait_heartbeat()

print("Connection to the vehicle")
print("Target system:",vehicle.target_system,"Target component:",vehicle.target_component)