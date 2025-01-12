import serial
import time
import pandas as pd
import matplotlib.pyplot as plt
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

# Create an output directory
output_dir = "plots"
os.makedirs(output_dir, exist_ok=True)

# Plot graphs for each frequency and run
unique_frequencies = sorted(df["Frequency"].unique())
unique_runs = sorted(df["Run"].unique())

for freq in unique_frequencies:
    for run in unique_runs:
        subset = df[(df["Frequency"] == freq) & (df["Run"] == run)]
        plt.figure(figsize=(8, 6))
        plt.plot(subset["DutyCycle"], subset["RPM"], marker="o", label=f"Run {int(run)}")
        plt.title(f"Frequency: {int(freq)} Hz, Run: {int(run)}")
        plt.xlabel("Duty Cycle (%)")
        plt.ylabel("RPM")
        plt.grid()
        plt.legend()

        # Save the plot
        filename = os.path.join(output_dir, f"Freq_{int(freq)}Hz_Run_{int(run)}.png")
        plt.savefig(filename, dpi=300)
        print(f"Saved plot: {filename}")
        plt.close()

print("All plots saved successfully!")
