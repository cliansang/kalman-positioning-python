# Example using simulated data

import numpy as np
from KalmanFilter import KalmanFilter as kf
from Helpers_KF import initConstVelocityKF as initConstVel 
from Helpers_KF import initConstAccelerationKF as initConstAcc
import matplotlib.pyplot as plt 

sim_data = np.linspace(1, 10, 100)
const = np.ones((100))

# simulate a rectangle shape path in 2D 
sim_x_true = np.concatenate((const, sim_data, 10* const), axis=0)
sim_y_true = np.concatenate((sim_data, 10* const, 10 - sim_data), axis=0)
sim_z_true = np.linspace(1,10, len(sim_x_true))

sim_x_noi = sim_x_true + np.random.normal(0, 0.8, len(sim_x_true)) # mu, sigma
sim_y_noi = sim_y_true + np.random.normal(0, 0.8, len(sim_x_true)) # mu, sigma
sim_z_noi = sim_z_true + np.random.normal(0, 0.8, len(sim_x_true))

# Suppose const velocity motion model is used
# A = np.zeros((6, 6)) # place holder for system state Matrix
# H = np.zeros((3, 6))  

# For Constant Acceleration Motion model
A = np.zeros((9, 9)) # place holder for system state Matrix
H = np.zeros((3, 9))  

# Object creation of the KF 
sim_kf = kf(A, H, 0)

# Initialization of the parameter for KF
# A, B, H, Q, R, P_0, x_0 = initConstVel()
A, B, H, Q, R, P_0, x_0 = initConstAcc()
sim_kf.assignSystemParameters(A, B, H, Q, R, P_0, x_0)

# place holders 
sim_x_kf = []
sim_y_kf = []
sim_z_kf = []

# Usage of the KF class
for data in range(len(sim_x_noi)):
    meas = np.array([sim_x_noi[data], sim_y_noi[data], sim_z_noi[data] ])  # suppose Z-axis is zero for 2D samples
    meas.shape = (len(meas), 1)    
    sim_kf.performKalmanFilter(np.array(meas), 0)
    sim_x_kf.append(sim_kf.x_m[0])
    sim_y_kf.append(sim_kf.x_m[1])
    sim_z_kf.append(sim_kf.x_m[2])


# Visualization of the results
plt.plot(sim_x_true, sim_y_true, label ='True', color='g')
plt.scatter(sim_x_noi, sim_y_noi, label='Measurement')
plt.scatter(sim_x_kf, sim_y_kf, label="KF")
plt.xlabel("Value on the X-axis / m")
plt.ylabel("Value on the Y-axis / m")
plt.title("Demonstration of KF in 2D using simulated data")
plt.legend()
plt.grid(True)
plt.show()

# on the Z-axis
plt.plot(sim_z_true, label="True")
plt.plot(sim_z_noi, label = "Measurement")
plt.plot(sim_z_kf, label="KF")
plt.title("Data on the Z-axis")
plt.ylabel("Values on the Z-axis")
plt.legend()
plt.show()    