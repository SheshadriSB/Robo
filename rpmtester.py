import serial
import matplotlib.pyplot as plt

# Configure serial port
ser = serial.Serial('/dev/ttyUSB0', 115200)  # Replace 'COM3' with your port
pwm_values = []
rpm_values = []

while len(pwm_values) < 100:  # Collect 100 data points
    if ser.in_waiting > 0:
        data = ser.readline().decode().strip()  # Read serial data
        try:
            pwm, rpm = map(float, data.split(","))
            pwm_values.append(pwm)
            rpm_values.append(rpm)
            print(f"PWM: {pwm}, RPM: {rpm}")
        except:
            pass  # Ignore invalid data

ser.close()

# Plot the data
plt.plot(pwm_values, rpm_values, marker='o')
plt.title('RPM vs PWM')
plt.xlabel('PWM Signal (0-255)')
plt.ylabel('Motor RPM')
plt.grid()
plt.show()
