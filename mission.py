import time
import cv2
import numpy as np
from pyzbar.pyzbar import decode
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
from geopy.distance import geodesic

# Define geofence
geofence = [
    (13.2865093, 77.5961633),  # Bottom-Left
    (13.2864995, 77.5963980),  # Bottom-Right
    (13.2868813, 77.5964610),  # Top-Right
    (13.2869165, 77.5962344)   # Top-Left
] # Change values as required

x_divisions = 15
y_divisions = 10
altitude = 15
vehicle = None
initial_qr_data = None

def generate_grid(geofence, x_divisions, y_divisions):
    """Generate a search grid based on geofence coordinates."""
    print("Generating grid...")
    
    bottom_left, bottom_right, top_right, top_left = geofence
    lat_start, lon_start = bottom_left
    lat_end, lon_end = top_right

    lat_step = (lat_end - lat_start) / y_divisions
    lon_step = (lon_end - lon_start) / x_divisions

    grid = []
    for i in range(y_divisions + 1):
        row = []
        for j in range(x_divisions + 1):
            lat = lat_start + i * lat_step
            lon = lon_start + j * lon_step
            row.append((lat, lon))
        grid.append(row)
    
    return grid

def serpentine_path_grid(grid):
    print("Serpentine path started")
    for i, row in enumerate(grid):
        if i % 2 == 0:
            # Move from left to right
            print("Move from left to right")
            for point in row:
                lat, lon = point
                go_to_location(lat, lon, altitude)
                time.sleep(0.1)
        else:
            # Move from right to left
            print("Move from right to left")
            for point in reversed(row):
                lat, lon = point
                go_to_location(lat, lon, altitude)
                time.sleep(0.1)

def arm_and_takeoff(target_altitude):
    """Arm the drone and take off to the target altitude."""
    print("Arming motors...")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    
    while not vehicle.armed:
        print("Waiting for arming...")
        time.sleep(1)

    print("Taking off...")
    vehicle.simple_takeoff(target_altitude)

    while True:
        altitude_now = vehicle.location.global_relative_frame.alt
        print(f"Altitude: {altitude_now:.2f} meters")

        if altitude_now >= target_altitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

def drop_payload():
    """Trigger servo to drop payload."""
    print("Dropping payload...")
    msg = vehicle.message_factory.command_long_encode(
        0, 0, mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0, 9, 2000, 0, 0, 0, 0, 0  
    )
    vehicle.send_mavlink(msg)
    time.sleep(2)
    print("Payload dropped")

def scan_qr_code(target=None):
    """Scan for QR codes using a webcam."""
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Could not open webcam.")
        return None

    target_location = None
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to capture frame")
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        qr_codes = decode(gray)

        cv2.imshow('QR Code Scanner', frame)
        for qr in qr_codes:
            qr_data = qr.data.decode('utf-8')
            print(f"QR Code Detected: {qr_data}")
            if target:
                if qr_data == target:
                    print("Target QR code detected.")
                    target_location = vehicle.location.global_relative_frame
            cap.release()
            cv2.destroyAllWindows()
            return qr_data

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cap.release()
    cv2.destroyAllWindows()
    return target_location

def go_to_location(latitude, longitude, altitude):
    """Navigate to a GPS location."""
    print(f"Going to Latitude: {latitude}, Longitude: {longitude}, Altitude: {altitude}")
    target_location = LocationGlobalRelative(latitude, longitude, altitude)
    vehicle.simple_goto(target_location, groundspeed=2.8)

    while True:
        current_location = vehicle.location.global_relative_frame
        distance_to_target = get_distance_metres(current_location, target_location)
        print(f"Distance to waypoint: {distance_to_target:.2f} meters")
    
        if distance_to_target <= 1.0:
            print("Reached waypoint.")
            break
        time.sleep(1)

def get_distance_metres(location1, location2):
    """Calculate the distance between two GPS coordinates in meters."""
    coords_1 = (location1.lat, location1.lon)
    coords_2 = (location2.lat, location2.lon)
    return geodesic(coords_1, coords_2).meters

def mission():
    """Execute the full drone mission."""
    print("Mission Begins...")
    start_time = time.time()
    print(f"Initial QR Data: {initial_qr_data}")
    # Generate the grid
    grid = generate_grid(geofence, x_divisions, y_divisions)

    # Takeoff
    arm_and_takeoff(altitude)
    time.sleep(2)

    # Follow the grid path
    serpentine_path_grid(grid)
    time.sleep(2)

    print("Returning to Launch...")
    vehicle.mode = VehicleMode("RTL")
    
    while vehicle.armed:
        time.sleep(1)
    
    end_time = time.time()
    print(f"Mission Time: {end_time - start_time:.2f} seconds")
    print("Mission complete.")

def initialise():
    """Initialize QR scanning and countdown before mission."""
    global vehicle, initial_qr_data
    initial_qr_data = scan_qr_code()
    if not initial_qr_data:
        print("No QR code detected, aborting mission")
    
    input("Press Enter to start mission...")

    # Connect to the vehicle at startup
    print("Connecting to vehicle...")
    vehicle = connect('/dev/tty/ACM0', baud=92100, wait_ready=True, timeout=60) # Replace the connection string with the appropriate port and baudrate

if __name__ == "__main__":
    initialise()
    
    if initial_qr_data:
        mission()
    else:
        print("Mission aborted due to missing QR data.")
        if vehicle:
            vehicle.close()
