import time
from pymavlink import mavutil

# Establish connection to the drone
master = mavutil.mavlink_connection("udpout:192.168.144.10:14552")

# Send a ping to verify connection
master.mav.ping_send(
    int(time.time() * 1e6),  # Unix time in microseconds
    0,  # Ping number
    0,  # Request ping of all systems
    0   # Request ping of all components
)

# Wait for the first heartbeat to confirm connection
master.wait_heartbeat()
print("Heartbeat received! Drone is online.")

# Function to change mode
def set_mode(mode):
    mode_id = master.mode_mapping()[mode]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )
    print(f"Mode set to {mode}")

# Function to arm/disarm the drone
def arm_disarm(arm):
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1 if arm else 0,  # 1 to arm, 0 to disarm
        0, 0, 0, 0, 0, 0
    )
    state = "Armed" if arm else "Disarmed"
    print(f"Drone {state}")
    time.sleep(2)  # Give some time for the action to take effect

# Set mode to GUIDED (needed for arming)
set_mode("GUIDED")
time.sleep(2)

# Arm the drone
arm_disarm(True)

# Wait for 5 seconds
time.sleep(5)

# Disarm the drone
arm_disarm(False)

print("Mission complete.")
