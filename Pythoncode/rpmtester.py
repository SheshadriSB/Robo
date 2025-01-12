import serial
import time
import pandas as pd
import os

# Configure the serial port
port = "/dev/ttyUSB0"  # Change to the correct port
baudrate = 115200
ser = serial.Serial(port, baudrate, timeout=1)

# Reset the ESP32 by toggling DTR/RTS
ser.setDTR(False)
ser.setRTS(False)
time.sleep(0.1)
ser.setDTR(True)
ser.setRTS(True)
time.sleep(2)  # Wait for ESP32 to reboot

data = []

print("Collecting data...")
try:
    while True:
        line = ser.readline().decode().strip()
        if not line or "ets " in line or "rst: " in line:
            continue  # Skip boot messages

        if "Starting data collection" in line:
            print(line)
        elif "Data collection finished" in line:
            print(line)
            break
        else:
            try:
                freq, duty, run, rpm = map(float, line.split(","))
                data.append((freq, duty, run, rpm))
                print(f"Freq: {freq} Hz, Duty: {duty}%, Run: {run}, RPM: {rpm}")
            except ValueError:
                print(f"Unexpected line: {line}")  # Debugging any other unexpected lines

except KeyboardInterrupt:
    print("Data collection interrupted.")

ser.close()

# Convert the data to a DataFrame
df = pd.DataFrame(data, columns=["Frequency", "DutyCycle", "Run", "RPM"])

# Create a CSV file
output_csv = "pwm_vs_rpm1.csv"
df.to_csv(output_csv, index=False)
print(f"Data saved to {output_csv}")

