# ADDC-2025
Proposed approach for SAEISS ADDC 2025

This repository contains a proposed approach towards the problem statement of 3rd Autonomous Drone Development Challenge (ADDC) 2025 conducted by SAE India Southern Section (SAEISS). The mission profile and the rulebook is uploaded above.

This code makes use of ```pyzbar``` library to scan the QR Codes provided and ```dronekit``` library to connect to the drone. You would need ```Python 3.9``` or higher versions to run the code.

Libraries required:
```
pip install dronekit
pip install geopy
pip install MAVProxy
pip install opencv-python
pip install pyserial
pip install pyzbar
```

In terminal, run the following code:
```
python3 mission.py
```

# Autonomous Approach Methodology
This approach assumes that there are multiple QR codes on the field. A QR Code will be provided before takeoff, which will be scanned and the data will be stored. The drone will then divide the field into grids based on the divisions specified using the GPS co-ordinates of the boundaries of the field and follow a serpentine path to cover the entire field, while actively scanning for QR Codes and storing their data. The drone will then store the location of the QR code whose data matches with the initial QR code provided at the launch site. After completing the path, the drone moves to the target QR code, descents to 5 m altitude and drop the payload and then Return to Launch (RTL).