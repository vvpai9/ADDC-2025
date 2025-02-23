import logging
import sys
import time
from pymavlink import mavutil
from geopy.distance import geodesic

# Configure logging
log_filename = "mission_debug.log"
logging.basicConfig(
    level=logging.DEBUG,  # Capture all levels of logs (DEBUG, INFO, ERROR)
    format="%(asctime)s - %(levelname)s - %(message)s",
    handlers=[
        logging.FileHandler(log_filename, mode='a'),  # Write logs to file
        logging.StreamHandler(sys.stdout)  # Still print to console
    ]
)

# Redirect stdout and stderr to logger
class LoggerWriter:
    def __init__(self, level):
        self.level = level
        self.buffer = ''

    def write(self, message):
        if message != '\n':
            self.buffer += message
        if '\n' in message:
            self.flush()

    def flush(self):
        if self.buffer:
            self.level(self.buffer)
            self.buffer = ''

sys.stdout = LoggerWriter(logging.info)
sys.stderr = LoggerWriter(logging.error)

logging.info("-----------------------------------------------") # Separator for new runs

# Establish connection to the drone
logging.info("Connecting to drone...")
time.sleep(2)
master = mavutil.mavlink_connection("udpout:192.168.144.10:14552")

# Modes for ArduPilot
STABILIZE = 0
ACRO = 1
ALT_HOLD = 2
AUTO = 3
GUIDED = 4
LOITER = 5
RTL = 6
CIRCLE = 7
LAND = 9
DRIFT = 11
SPORT = 13
FLIP = 14
AUTOTUNE = 15
POSHOLD = 16
BRAKE = 17
THROW = 18
AVOID_ADSB = 19
GUIDED_NO_GPS = 20
SMART_RTL = 21
FLOWHOLD = 22
FOLLOW = 23
ZIGZAG = 24
SYSTEMID = 25
AUTOROTATE = 26
AUTO_RTL = 27

# Send a ping to verify connection
master.mav.ping_send(
    int(time.time() * 1e6),  # Unix time in microseconds
    0,  # Ping number
    0,  # Request ping of all systems
    0)   # Request ping of all components

# Wait for the first heartbeat to confirm connection
master.wait_heartbeat()
logging.info("Heartbeat received! Drone is online.")

# Function to change mode and confirm it
def set_mode(mode_id):
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )
    logging.info(f"Mode change command sent for mode ID {mode_id}")
    # Confirm mode change
    while True:
        msg = master.recv_match(type='HEARTBEAT', blocking=True)
        if msg and msg.custom_mode == mode_id:
            logging.info(f"Mode successfully changed to ID {mode_id}")
            break

# Function to arm the drone and takeoff to altitude
def arm_and_takeoff(altitude):
    logging.info("GUIDED Mode")
    set_mode(GUIDED)
    time.sleep(2)
    
    logging.info("Arming motors...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1,  # 1 to arm, 0 to disarm
        0, 0, 0, 0, 0, 0
    )

    time.sleep(2)
    logging.info(f"Initiating takeoff to {altitude} meters...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0, 0, 0, altitude
    )
    logging.info("Takeoff command sent.")
    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
        if msg:
            current_altitude = msg.relative_alt / 1000.0  # Convert from mm to meters
            logging.info(f"Altitude: {current_altitude:.2f} meters")
            if current_altitude >= altitude * 0.95:
                logging.info("Reached target altitude.")
                break
        time.sleep(0.5)

# Function to send velocity command in NED frame
def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration=1):
    for _ in range(duration):
        master.mav.set_position_target_local_ned_send(
            0,  # time_boot_ms (not used)
            master.target_system, # target system
            master.target_component, # target component
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, # frame
            0b10111000111,  # type_mask (only speeds enabled)
            0, 0, 0, # x, y, z positions (not used)
            velocity_x, velocity_y, velocity_z,  # x, y, z velocity in m/s
            0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0) # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
        time.sleep(1)

# Function to drop payload using servo
def drop_payload(PWM):
    logging.info("Dropping payload...")
    master.mav.command_long_send(
        master.target_system, # target_system
        master.target_component, # target_component
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO, # command
        0, # confirmation
        9,  # Servo number
        PWM, # servo position between 1000 and 2000
        0, 0, 0, 0, 0) # param 3 ~ 7 not used
    logging.info("Payload Dropped.")

def go_to_location(latitude, longitude, altitude):
    """
    Sends a command to go to the specified GPS location.
    
    Parameters:
    - connection: MAVLink connection object
    - latitude: Target latitude in degrees
    - longitude: Target longitude in degrees
    - altitude: Target altitude in meters (relative to home position)
    """
    # Convert latitude and longitude to degrees * 1E7 (MAVLink format)
    lat = int(latitude * 1e7)
    lon = int(longitude * 1e7)

    # Sending command using MAV_CMD_NAV_WAYPOINT
    master.mav.command_long_send(
        master.target_system,  # Target system ID
        master.target_component,  # Target component ID
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,  # Command ID
        0,  # Confirmation
        0,  # Hold time in seconds
        0,  # Acceptance radius in meters
        0,  # Pass through (0 = stop at waypoint, 1 = continue)
        float('nan'),  # Yaw (NaN to use default)
        lat,  # Latitude
        lon,  # Longitude
        altitude  # Altitude in meters
    )
    logging.info(f"Command sent to go to Latitude: {latitude}, Longitude: {longitude}, Altitude: {altitude}m")

    # Monitor distance to target
    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        if msg:
            current_lat = msg.lat / 1e7
            current_lon = msg.lon / 1e7
            current_alt = msg.relative_alt / 1000.0
            distance = geodesic((latitude, longitude), (current_lat, current_lon)).meters
            logging.info(f"Distance to target: {distance:.2f} meters | Current Altitude: {current_alt:.2f} m")
            if distance <= 1.0 and abs(current_alt - altitude) <= 0.5:
                logging.info("Reached target location.")
                break
        time.sleep(0.5)

def mission():
    try:
        # Mission sequence
        start_time = time.time()
        logging.info("Mission Begins")

        # Arm and takeoff to 3 meters
        # arm_and_takeoff(3)

        # Example navigation command
        go_to_location(15.369547845989852365, 75.12451452369874226, 3)

        # Wait for 5 seconds
        time.sleep(3)

        # Land the drone
        # logging.info("LAND Mode")
        # set_mode(LAND)
        # time.sleep(2)

        logging.info("RTL Mode")
        # set_mode(RTL)
        time.sleep(2)

        logging.info("Mission complete.")
    except KeyboardInterrupt:
        logging.error("Mission interrupted by user.")
    except Exception as e:
        logging.error(e)
    finally:
        end_time = time.time()
        mission_time = end_time - start_time
        logging.info(f"Mission Time: {mission_time:.2f} seconds")

if __name__ == "__main__":
    mission()
