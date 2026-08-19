#!/usr/bin/env python3
"""Upload LittleFS filesystem via OTA using espota protocol"""
import subprocess
import os
import sys

# PlatformIO espota.py script path
espota_script = r"C:\Users\krist\.platformio\packages\framework-arduinoespressif32\tools\espota.py"
fs_image = r".pio\build\esp32s3-OTA-Upload\littlefs.bin"
host = "192.168.0.211"
port = 3232
auth = "metasense"

if not os.path.exists(fs_image):
    print(f"Error: Filesystem image not found: {fs_image}")
    sys.exit(1)

if not os.path.exists(espota_script):
    print(f"Error: espota.py not found: {espota_script}")
    # Try alternative path
    espota_script = r"C:\Users\krist\.platformio\packages\tool-esptool\esptool.py"
    if not os.path.exists(espota_script):
        print("Error: Neither espota.py nor esptool.py found")
        sys.exit(1)

print(f"Uploading filesystem from: {fs_image}")
print(f"Target: {host}:{port}")
print(f"Auth: {auth}")

# Build espota upload command
# Note: Use -s flag for filesystem images (SPIFFS/LittleFS)
cmd = [
    "python.exe",
    espota_script,
    "-i", host,
    "-p", str(port),
    "-a", auth,
    "-f", fs_image,
    "-s"  # Use -s flag for SPIFFS/LittleFS image
]

print(f"Command: {' '.join(cmd)}")
print("Starting upload...\n")

# Run the upload
result = subprocess.run(cmd, capture_output=False)
sys.exit(result.returncode)
