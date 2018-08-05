# **CarND Term 2 Project 1 Writeup** 
# **Extended Kalman Filter** 

#### This repository contains a c++ implementation of an extended kalman filter. This filter is able to perform sensor fusion by combining incoming laser measurements and radar measurements. The repository is structured so that it works together with the [simulation environment provided by Udacity](https://github.com/udacity/self-driving-car-sim/releases). (The link between the environment and the code is via [_uWebSocketIO_](https://github.com/uNetworking/uWebSockets), and already provided by Udacity).
[iLectureOnline](http://www.ilectureonline.com/lectures/subject/SPECIAL%20TOPICS/26/190)

[//]: # (Image References)

[EXMPLE]: ./output_images/car_noncar_samples.jpg "Training data examples"
[FEAT]:   ./output_images/features.jpg "Features example extracted from images"
[HOTBB]:  ./output_images/hot_bboxes.jpg "Positively identified windows in an image"
[HEAT]:   ./output_images/heat_map.jpg "Heat map based on detections"
[ANN]:    ./output_images/test_image_ann.jpg "Thresholded heat map based annotations"

---

## Important scripts

This writeup will give an overview of the extended kalman filter implementation by going through 4 important scripts:

#### [1. main.cpp](#main.cpp),
#### [2. tools.cpp](#2.-tools.cpp),
#### [3. FusionEKF.cpp](#3.-FusionEKF.cpp), and
#### [4. kalman_filter.cpp](#4.-kalman_filter.cpp)

### 1. main.cpp

### 2. tools.cpp

#### CalculateRMSE()

    VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,const vector<VectorXd> &ground_truth) {}
    
#### CalculateJacobian()

    MatrixXd Tools::CalculateJacobian(const VectorXd& x_state, const MatrixXd& prev_Hj_) {}

### 3. FusionEKF.cpp


### 4. kalman_filter.cpp

## Conclusion

## Note

Currently, upon a restart or a dataset switch in the simulation environment, main.cpp does **not** forward any flag/event. This means that upon restart of the environment, the kalman filter object is **not** re-initialized. This results in the following behaviour:

* Not reinitializing them actually makes the performance worse on a second run as compared to a first run
* Not reinitializing upon switching environments can actually make the kalman filter code crash

I filed an issue in the original udacity repository asking if and how this would be possible.

![alt text][EXMPLE]

