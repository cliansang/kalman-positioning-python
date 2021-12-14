# kalman-python-positioning
This repo express the Kalman filter implementation in Python for wireless positioning and navigation system especially for UWB-based localization. The Kalman class was originally written for smoothing the location data achieved from UWB-based positioning in [our previous repo](https://github.com/cliansang/uwb-tracking-ros). The example use-case of the filter includes both the simulated data and the real-world data extracted from the UWB-hardwares namely DWM1001 from Decawave. For wireless positioning and navigation system using Kalman-based filters, a motion (or) kinematic model is necessary to provide. In this repo, we provide both the constant velocity and acceleartion motion models, which are widely used in point localization such as wireless positioning systems. 

The use-case and system integration (not necessarily in Python) of Kalman related filters in UWB-based positioning and navigation can also be found in our papers as follow:  
(i) [A Bidirectional Object Tracking and Navigation System using a True-Range Multilateration Method](https://ieeexplore.ieee.org/document/8911811)  
(ii) [A Comparative Study of UWB-based True-Range Positioning Algorithms using Experimental Data](https://ieeexplore.ieee.org/document/8970249)

The dependency of the Kalman class is NumPy while matplotlib is used for the visualization of the sample data. simply run ``` python example_sim_data.py ``` or the experimental data one to see the sample data results. The following image show the sample result based on simulated data.  

![KF_sample](https://user-images.githubusercontent.com/18302290/146093193-2b4a2a97-7437-4b26-bcee-91062ea860d2.jpeg)
