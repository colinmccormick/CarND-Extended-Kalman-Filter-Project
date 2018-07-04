# Extended Kalman Filter Project
Udacity Self-Driving Car Engineer Nanodegree Program: Term 2
[Master project repo](https://github.com/udacity/CarND-Extended-Kalman-Filter-Project)

This project implements a [Kalman filter ](https://en.wikipedia.org/wiki/Kalman_filter) that uses lidar and radar measurements to track a moving vehicle. Because two types of sensor measurements are used, it is an example of [sensor fusion](https://en.wikipedia.org/wiki/Sensor_fusion). Since one of the sensors (radar) involves a nonlinear update process, we use an Extended Kalman Filter in that case. This project uses the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases).

## Project structure

The project consists of the following primary components:
* src/main.cpp - Responsible for interfacing with the simulator and reading (simulated) lidar and radar measurement data
* src/FusionEKF.cpp, .h - Responsible for holding the measurement projection and covariance matrices (H and R), initializing a Kalman Filter object, and passing measurement data to it
* src/kalman_filter.cpp, .h - Responsible for the actual Kalman Filter calculations
* src/tools.cpp, .h - Responsible for calculation of RMSE and the Jacobian matrix

## Kalman filter concepts

Our algorithm represents our best current knowledge of the position and velocity of the vehicle being tracked as a point in 4-dimensional phase space, with two spatial dimensions (x,y) and two velocity dimensions (vx,vy). We also track the uncertainty in this knowledge using a covariance value (the P matrix).

When a new measurement arrives, the Kalman filter first predicts where it thinks the tracked object is at the current time in phase space (using the F matrix). It then projects this prediction into the space of the measurement (using the H matrix). For lidar measurements this is straightforward - these are Cartesian (x,y) measurements only so H simply drops the velocity components. For radar measurements this is more complex, since these are in polar coordinates and have one velocity dimension (radial). For this case H is effectively the Jacobian that relates Cartesian coordinates to polar coordinates.

Once the prediction has been projected into the measurement space, an error can be calculated (the y vector) and then the Kalamn filter equations can be used to update our "best current knowledge" of the vehicle's phase space position. These equations also give us an updated estimated of the corresponding uncertainty.

One important conceptual point to absorb is that we don't treat measurements as absolute truth about the vehicle's position (or velocity). That's because measurements have error, and also because that would effectively be throwing away all previous information we had about the vehicle's state. Instead, the Kalman filter equations trade off the relative uncertainties of our current knowledge of the vehicle's state and the measurement to arrive at an updated "state of knowledge" about the vehicle's position and velocity.

A second important conceptual point is that our knowledge of the vehicle's velocity is mostly inferred from position measurements - other than radial velocity from radar measurements, we never directly measure velocity. However, we can infer this velocity information through the Kalman filter process, illustrating one its key features.

## Project comments

A lot of the code was given to us in the [master repo](https://github.com/udacity/CarND-Extended-Kalman-Filter-Project). The primary tasks were to complete the code in FusionEKF.cpp, kalman_filter.cpp, and tools.cpp.

Implementing the RMSE calculation and Jacobian calculation in tools.cpp was fairly straightforward and drew directly on the relevant lessons.

Implementing several parts of FusionEKF.cpp was more challenging. Conceptually, this object holds several of the Kalman matrices directly, essentially those related to the measurement process (the projection and covariance matrices H and R). It also initializes and holds a Kalman filter object, manages its initialization with the first measurement point, and calls its functions as appropriate for each measurement type that is received.

One point of confusion is the apparent redundancy of the measurement projection matrix H. This is supposed to project the value of the estimated state (in phase space) into the measurement space. This is a constant for lidar, and depends on the state x for radar. It's not clear why FusionEKF has a data member Hj that is simply overwritten and then copied to the ekf_.H_ data member - this seems unnecessary.

The only somewhat tricky issue I encountered was handling the polar angle variable in the extended Kalman filter (KalmanFilter::UpdateEKF). I manually convert the Cartesian coordinates of the predicted state into polar, and then calculate the difference between the predicted state and the measured state. Per suggestions, I then test whether the difference in polar angle falls outside the range [-pi,pi]. If so, I adjust by multiples of 2 pi until it it within range. Once this is implemented correctly the algorithm can handle all the sensor measurement data without any issues.

## Results

My filter achieves RMSE of [0.0973,0.0855,0.4513,0.4399] for [px,py,vx,vy], well within the project rubric requirements. It also meets the rubric requirements for Dataset 2.
