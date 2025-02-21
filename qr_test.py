import cv2
from pyzbar.pyzbar import decode
import numpy as np

def scan_qr_code():
    # Initialize the webcam (0 for default camera)
    cap = cv2.VideoCapture(0)
    
    if not cap.isOpened():
        print("Error: Could not open the camera.")
        return

    print("Starting QR Code Scanner. Press 'q' to exit.")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame.")
            break

        # Convert frame to grayscale for better detection
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Detect QR codes in the frame
        qr_codes = decode(gray)

        for qr in qr_codes:
            # Extract bounding box points
            points = np.array([qr.polygon], np.int32)
            points = points.reshape((-1, 1, 2))

            # Draw the bounding box
            cv2.polylines(frame, [points], True, (0, 255, 0), 3)

            # Extract QR code data
            qr_data = qr.data.decode('utf-8')
            qr_type = qr.type

            # Display the data on the frame
            text = f"{qr_data} ({qr_type})"
            x, y, w, h = qr.rect
            cv2.putText(frame, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)

        # Display the frame with QR codes
        cv2.imshow("QR Code Scanner", frame)

        # Exit loop when 'q' key is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the camera and close windows
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    scan_qr_code()