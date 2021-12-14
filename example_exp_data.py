import numpy as np
from KalmanFilter import KalmanFilter as kf
from Helpers_KF import initConstVelocityKF as initConstVel 
from Helpers_KF import initConstAccelerationKF as initConstAcc
import matplotlib.pyplot as plt 

# place holders
kf_list = []
log_raw_xyz = []
log_kf_xyz = []

# The data are logged from real UWB hardware from Decawave called DWM1001 
with open("exp_data/minicom_dwm1001_usb_data_static.cap") as ser_dwm1001:    
    for serData in ser_dwm1001.readlines():
        serDataList = [x.strip() for x in serData.strip().split(',') ]  # split by comma 
        
        # For extracting data in the serial List (supose multiple data such as 'POS' and 'DIST' are available)
        if "POS" in serDataList[0] :  # extract 'POS' data only 
            tag_id = int(serDataList[1])  # IDs in 0 - 15
            tag_macID = (serDataList[2], 'UTF8')  # IDs in 'HEX' with 16-bits            
            pos_ser_x = float(serDataList[3])
            pos_ser_y = float(serDataList[4])
            pos_ser_z = float(serDataList[5])            
            pos_ser_list = [pos_ser_x, pos_ser_y, pos_ser_z]
            
            # Discard the raw data if there exists 'nan' in the measured serial logged data
            if (np.isnan(pos_ser_list).any()):
#                 print("Serial data includes NaN!")
                pass
            else:
                pos_ser_xyz = np.array(pos_ser_list)
                pos_ser_xyz.shape = (len(pos_ser_xyz), 1)   # force to be a column vector                  
           
            # Kalman Filter on multiple mobile UWB nodes 
            if tag_macID not in kf_list:
                kf_list.append(tag_macID)
                
                # For Constant Velocity Motion Model 
#                 A = np.zeros((6, 6))                            # place holder for system state Matrix
#                 H = np.zeros((3, 6))                           # place holder for meas. to state transition matrix                
                # For constant acceleration motion model
                A = np.zeros((9,9))
                H = np.zeros((3,9))                
                
                # create the KF object
                kf_list[tag_id] = kf(A, H, tag_macID)  

            # Initialization should be done only once
            if (kf_list[tag_id].isKalmanInitialized == False):
#                 A, B, H, Q, R, P_0, x_0 = initConstVel()       # use Const. Velocity Model 
                A, B, H, Q, R, P_0, x_0 = initConstAcc()         # use Const. Acceleration Model 
                kf_list[tag_id].assignSystemParameters(A, B, H, Q, R, P_0, x_0)
                kf_list[tag_id].isKalmanInitialized = True

            # Usage of the Kalman Filter        
            kf_list[tag_id].performKalmanFilter(np.array((pos_ser_xyz)), 0)
            
            if tag_id == 0:    # Append data  from specific ID 
                log_raw_xyz.append(pos_ser_xyz)
                log_kf_xyz.append(kf_list[tag_id].x_m)
   
    # Data to visualize 
    viz_raw_xyz = np.array(log_raw_xyz)
    viz_kf_xyz = np.array(log_kf_xyz)
    
# Visualization of the results using matplotlib 
plt.scatter(viz_raw_xyz[:, 0], viz_raw_xyz[:, 1], label="Raw location data")
plt.scatter(viz_kf_xyz[:, 0], viz_kf_xyz[:, 1], label="KF-based Data")
plt.title("UWB Location Data in 2D")
plt.xlabel("Values on the  X-axis / m")
plt.ylabel("Values on the Y-axis / m")
plt.grid(True)
plt.legend()
plt.show()

# Histogram of the raw location data
plt.hist(viz_raw_xyz[:, 0], label ="X-axis")
plt.hist(viz_raw_xyz[:, 1], label="Y-axis")
plt.hist(viz_raw_xyz[:, 2], label="Z-axis")
plt.title("Histogram on the raw location data")
plt.xlabel("Measured Distances / m")
plt.ylabel("Frequency")
plt.grid(True)
plt.legend()
plt.show()

# Histogram using KF
plt.hist(viz_kf_xyz[:, 0], label="X-axis")
plt.hist(viz_kf_xyz[:, 1], label="Y-axis")
plt.hist(viz_kf_xyz[:, 2], label="Z-axis")
plt.title("Histogram of location data using KF")
plt.xlabel("Measured Distances / m")
plt.ylabel("Frequency")
plt.grid(True)
plt.legend()
plt.show()









    
   

