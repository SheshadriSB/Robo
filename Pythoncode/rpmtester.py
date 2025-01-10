import serial
import time
import matplotlib.pyplot as plt
import seaborn as sns

# Set up the serial connection
serial_port = "/dev/ttyUSB1"  # Replace with your Arduino's port
baud_rate = 115200
arduino = serial.Serial(serial_port, baud_rate, timeout=1)

# Initialize data storage
pwm_data = []
rpm_data = []
peak_rpm = 0

def update_peak_rpm(current_rpm, peak_rpm):
    """Update the peak RPM if the current RPM is higher."""
    return max(current_rpm, peak_rpm)

def read_serial_data():
    """Read and parse data from the Arduino."""
    try:
        line = arduino.readline().decode('utf-8').strip()
        if line:
            # Assuming Arduino sends data as "PWM,RPM"
            pwm, rpm = map(float, line.split(","))
            return pwm, rpm
    except Exception as e:
        print(f"Error reading serial data: {e}")
    return None, None

def plot_graph(pwm_data, rpm_data, peak_rpm):
    """Plot RPM vs PWM with peak RPM annotation."""
    sns.set_theme(style="darkgrid")
    plt.figure(figsize=(10, 6))
    plt.plot(pwm_data, rpm_data, label="RPM vs PWM", color='blue', linewidth=2)
    plt.axhline(peak_rpm, color='red', linestyle='--', label=f"Peak RPM: {peak_rpm:.2f}")
    plt.xlabel("PWM Signal (%)")
    plt.ylabel("RPM")
    plt.title("Motor Speed (RPM) vs PWM Signal")
    plt.legend()
    plt.grid(True)
    plt.show()

try:
    print("Reading data from Arduino...")
    start_time = time.time()
    duration = 20  # Run for 60 seconds

    while time.time() - start_time < duration:
        pwm, rpm = read_serial_data()
        if pwm is not None and rpm is not None:
            print(f"PWM: {pwm}, RPM: {rpm}")
            pwm_data.append(pwm)
            rpm_data.append(rpm)
            peak_rpm = update_peak_rpm(rpm, peak_rpm)

    print("Data collection complete.")
    print(f"Peak RPM: {peak_rpm}")

    # Plot the graph
    plot_graph(pwm_data, rpm_data, peak_rpm)

except KeyboardInterrupt:
    print("\nData collection interrupted by user.")

finally:
    arduino.close()
    print("Serial connection closed.")
