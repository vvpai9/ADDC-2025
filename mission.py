import time
import cv2
import numpy as np
from pyzbar.pyzbar import decode
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
from geopy.distance import geodesic
import threading

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
target_location = None
prev_qr = None


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
    """Follow a serpentine path over the grid."""
    for i, row in enumerate(grid):
        path = row if i % 2 == 0 else reversed(row)
        for point in path:
            lat, lon = point
            go_to_location(lat, lon, altitude)
            scan_qr_code()
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

def send_ned_velocity(velocity_x, velocity_y,velocity_z):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,  # time_boot_ms (not used)
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,  # frame
        0b10111000111,  # type_mask (only speeds enabled)
        0, 0, 0,  # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z,  # x, y, z velocity in m/s
        0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    vehicle.send_mavlink(msg)
    vehicle.flush()

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


def scan_qr_code():
    """Continuously scan for QR codes and track bounding boxes."""
    global prev_qr, target_location
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        return
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to capture frame")
            break
        
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        qr_codes = decode(gray)
        
        for qr in qr_codes:
            pts = np.array(qr.polygon, np.int32).reshape((-1, 1, 2))
            cv2.polylines(frame, [pts], True, (0, 255, 0), 2)
            qr_data = qr.data.decode('utf-8')
            
            if qr_data != prev_qr:
                print(f"QR Code Detected: {qr_data}")
                prev_qr = qr_data
                
                if qr_data == initial_qr_data:
                    print("Target QR code detected")
                    target_location = vehicle.location.global_relative_frame
        
        cv2.imshow('QR Code Scanner', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cap.release()
    cv2.destroyAllWindows()


def go_to_location(latitude, longitude, altitude):
    """Navigate to a GPS location."""
    print(f"Going to Latitude: {latitude}, Longitude: {longitude}, Altitude: {altitude}")
    target_location = LocationGlobalRelative(latitude, longitude, altitude)
    vehicle.simple_goto(target_location, groundspeed=2.8)

    while True:
        current_location = vehicle.location.global_relative_frame
        if get_distance_metres(current_location, target_location) <= 1.0:
            break
        time.sleep(1)


def get_distance_metres(location1, location2):
    """Calculate distance between two GPS coordinates."""
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

    detection_thread = threading.Thread(target=scan_qr_code, name="QR Detection")   
    detection_thread.start()
    # Follow the grid path
    serpentine_path_grid(grid)
    time.sleep(2)
    
    if target_location:
        print("Moving to Target...")
        go_to_location(target_location.lat, target_location.lon, altitude)
        time.sleep(2)
        for i in range (10): # Descend from 15 m to 5 m
            send_ned_velocity(0,0,1) # Descend 1 m every second 
            time.sleep(1)            
        drop_payload()
        time.sleep(2)
    else:
        print("Target not found")
    
    print("Returning to Launch...")
    vehicle.mode = VehicleMode("RTL")
    
    while vehicle.armed:
        time.sleep(1)
    
    detection_thread.join()
    print(f"Mission Time: {time.time() - start_time:.2f} seconds")


def initialise():
    """Initialize QR scanning and drone connection."""
    global vehicle, initial_qr_data
    initial_qr_data = scan_qr_code()
    if not initial_qr_data:
        print("No QR code detected, aborting mission")
        return
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
