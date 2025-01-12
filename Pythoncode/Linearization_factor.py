import numpy as np
import matplotlib.pyplot as plt

# Provided data
pwm = np.array([126, 130, 134, 138, 142, 146, 150, 154, 158, 162, 166, 170, 174, 178, 182, 186, 190, 194, 198, 202, 206, 210, 214])
rpm = np.array([480, 485.71, 488.57, 494.29, 497.14, 500, 502.86, 505.71, 508.57, 511.43, 511.43, 514.29, 517.14, 517.14, 520, 520, 525.71, 522.86, 525.71, 528.57, 528.57, 531.43, 534.29])

# Exponential adjustment
def adjust_pwm(pwm, k=1.0, b=3.2):
    return k * (pwm ** b)

# Adjust parameters as needed
adjusted_pwm = adjust_pwm(pwm, k=0, b=0.355)

# Plot original vs adjusted
plt.figure(figsize=(10, 6))
plt.plot(pwm, rpm, label="Original RPM", marker="o")
plt.plot(adjusted_pwm, rpm, label="Adjusted PWM", marker="x")
plt.xlabel("PWM")
plt.ylabel("RPM")
plt.legend()
plt.grid(True)
plt.title("PWM Adjustment for Linearized RPM")
plt.show()