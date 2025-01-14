import numpy as np
import matplotlib.pyplot as plt

# Extract the PWM and RPM data
pwm = data.iloc[:, 0]  # First column: PWM
rpm = data.iloc[:, 1]  # Second column: RPM

# Define the dead zone (PWM values below this are ineffective)
dead_zone_pwm = 50

# Remove the dead zone
valid_indices = pwm >= dead_zone_pwm
pwm_valid = pwm[valid_indices]
rpm_valid = rpm[valid_indices]

# Fit a logarithmic function to the RPM data after the dead zone
log_rpm = np.log(rpm_valid)
coefficients = np.polyfit(log_rpm, pwm_valid, 1)  # Linear fit in log space
a, b = coefficients

# Define a function to transform RPM to adjusted PWM for linear mapping
def adjust_pwm(rpm_values):
    log_rpm = np.log(rpm_values)
    adjusted_pwm = a * log_rpm + b
    return adjusted_pwm

# Apply the transformation to all RPM values
adjusted_pwm = adjust_pwm(rpm_valid)

# Save the adjusted PWM and original RPM to a new CSV file
adjusted_data = pd.DataFrame({'Adjusted_PWM': adjusted_pwm, 'RPM': rpm_valid})
adjusted_file_path = '/mnt/data/adjusted_pwm_rpm.csv'
adjusted_data.to_csv(adjusted_file_path, index=False)

# Plot the adjusted PWM vs. RPM to verify linearity
plt.figure(figsize=(8, 6))
plt.plot(rpm_valid, adjusted_pwm, marker='o', label='Adjusted PWM')
plt.title('Adjusted PWM vs. RPM')
plt.xlabel('RPM')
plt.ylabel('Adjusted PWM')
plt.grid()
plt.legend()
plt.show()

adjusted_file_path
