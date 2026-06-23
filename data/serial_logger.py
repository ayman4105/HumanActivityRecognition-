import serial                         # Import serial library to read data from ESP32
import csv                            # Import csv library to write CSV files
import time                           # Import time library for delays
import os                             # Import os library to create folders

SERIAL_PORT = "/dev/ttyACM0"          # Set ESP32 serial port, change if needed
BAUD_RATE = 115200                    # Set baud rate, must match Arduino Serial.begin()
LABEL = "left_tilt"                        # Set current activity label
OUTPUT_FOLDER = "dataset"             # Set output folder name
OUTPUT_FILE = f"{OUTPUT_FOLDER}/{LABEL}.csv"  # Set output CSV file path

EXPECTED_HEADER = "timestamp,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z,label"  # Expected CSV header

os.makedirs(OUTPUT_FOLDER, exist_ok=True)  # Create dataset folder if it does not exist

ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)  # Open serial port
time.sleep(2)                                           # Wait for ESP32 reset after serial open

print(f"Saving data to: {OUTPUT_FILE}")                 # Print output file path
print("Press Ctrl+C to stop recording.")                # Tell user how to stop

with open(OUTPUT_FILE, "w", newline="") as file:        # Open CSV file for writing
    writer = csv.writer(file)                           # Create CSV writer object

    writer.writerow(["timestamp", "acc_x", "acc_y", "acc_z", "gyro_x", "gyro_y", "gyro_z", "label"])  # Write CSV header

    try:
        while True:                                     # Keep reading forever
            line = ser.readline().decode(errors="ignore").strip()  # Read one line from ESP32

            if not line:                                # Skip empty lines
                continue                                # Go to next loop

            if line.startswith("WHO_AM_I"):             # Skip debug line
                continue                                # Go to next loop

            if line.startswith("Calibrating"):          # Skip calibration line
                continue                                # Go to next loop

            if line.startswith("Calibration"):          # Skip calibration messages
                continue                                # Go to next loop

            if line.startswith("DATASET"):              # Skip dataset start message
                continue                                # Go to next loop

            if line.startswith("Current"):              # Skip current label message
                continue                                # Go to next loop

            if line == EXPECTED_HEADER:                 # Skip ESP32 printed header
                continue                                # Go to next loop

            parts = line.split(",")                     # Split CSV row by comma

            if len(parts) != 8:                         # Check row has 8 columns
                continue                                # Skip invalid row

            writer.writerow(parts)                      # Save valid row to CSV file
            file.flush()                                # Save immediately to disk

            print(line)                                 # Print saved row on terminal

    except KeyboardInterrupt:                           # Handle Ctrl+C
        print("\nRecording stopped.")                   # Print stop message

    finally:
        ser.close()                                     # Close serial port