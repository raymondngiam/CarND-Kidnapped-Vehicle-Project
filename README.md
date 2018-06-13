# Udacity Self Driving Car Nanodegree
## Kidnapped Vehicle Project 

### Overview

This is a project for Udacity's Self Driving Car Nanodegree. A self driving vehicle has been kidnapped and transported to a new location. Luckily it has a map of this location, a (noisy) GPS estimate of its initial location, and lots of (noisy) sensor and control data. The objective of this project is to implement a 2 dimensional particle filter in C++. The particle filter will be given a map and some initial localization information (analogous to what a GPS would provide). At each time step the filter will also get observation and control data.

---

### Video Demo

<img src="/images/demo.gif" width="800">

The green lines are ground truth laser observations of the vehicle, the blue lines are laser observations from the most probable particle.

---

### Implementation Summary

**Overview**

The overall particle filter localization pipeline is summarized in the diagram below:

<img src="/images/pipeline.png" width="600">

**Initialization**

The particle filter is initialized by sampling from a Gaussian distribution around the initial GPS estimates. The GPS measurements include the x coordinate, y coordinate (both in m) and the theta (orientation) of vehicle in radian. The standard deviations sigma_x, sigma_y, sigma_theta of the Gaussian sampling are based on the GPS specification.

The number of particles to be sampled is a parameter to be determined empirically. The larger the number of particles, the particle filter will be closer to representing the Bayesian posterior distribution. However it comes at a cost of long processing time and preventing localization in real time. On the other hand, if the number of particles is too few, we will not have enough particles to cover all the high likelihood state space. The number of particles we used here is 30.

**Prediction Step**

The motion model used, i.e. bicycle model is as illustrated below:

<img src="/images/bicycle_model.png" width="600">

The motion prediction for each time step, delta t, based on control inputs velocity and yaw rate, is governed by the following dynamics:

![](https://latex.codecogs.com/gif.latex?%5Cbegin%7Bpmatrix%7Dx_%7Bf%7D%5C%5Cy_%7Bf%7D%5C%5C%5Ctheta_%7Bf%7D%5Cend%7Bpmatrix%7D%3D%5Cbegin%7Bpmatrix%7Dx_%7B0%7D%5C%5Cy_%7B0%7D%5C%5C%5Ctheta_%7B0%7D%5Cend%7Bpmatrix%7D%2B%5Cbegin%7Bpmatrix%7Dvcos%28%5Ctheta_%7B0%7D%29%5CDelta%20t%5C%5Cvsin%28%5Ctheta_%7B0%7D%29%5CDelta%20t%5C%5C0%5Cend%7Bpmatrix%7D) when ![](https://latex.codecogs.com/gif.latex?\dot{\theta}=0)

![](https://latex.codecogs.com/gif.latex?%5Cbegin%7Bpmatrix%7Dx_%7Bf%7D%5C%5Cy_%7Bf%7D%5C%5C%5Ctheta_%7Bf%7D%5Cend%7Bpmatrix%7D%3D%5Cbegin%7Bpmatrix%7Dx_%7B0%7D%5C%5Cy_%7B0%7D%5C%5C%5Ctheta_%7B0%7D%5Cend%7Bpmatrix%7D%2B%5Cbegin%7Bpmatrix%7D%5Cfrac%7Bv%7D%7B%5Cdot%7B%5Ctheta%7D%7D%5Bsin%28%5Ctheta_%7B0%7D%2B%5Cdot%7B%5Ctheta%7D%5CDelta%20t%29%29-sin%28%5Ctheta_%7B0%7D%29%5D%5C%5C%5Cfrac%7Bv%7D%7B%5Cdot%7B%5Ctheta%7D%7D%5Bcos%28%5Ctheta_%7B0%7D%29-cos%28%5Ctheta_%7B0%7D%2B%5Cdot%7B%5Ctheta%7D%5CDelta%20t%29%29%5D%5C%5C%5Cdot%7B%5Ctheta%7D%5CDelta%20t%5Cend%7Bpmatrix%7D) when ![](https://latex.codecogs.com/gif.latex?\dot{\theta}\neq0)

To account for the uncertainty in motion, we will also add Gaussian noise to the x, y, and theta states. 

![](https://latex.codecogs.com/gif.latex?%5Cbegin%7Bpmatrix%7Dx_%7Bf%2Cnoisy%7D%5C%5Cy_%7Bf%2Cnoisy%7D%5C%5C%5Ctheta_%7Bf%2Cnoisy%7D%5Cend%7Bpmatrix%7D%3D%5Cbegin%7Bpmatrix%7Dx_%7Bf%7D%5C%5Cy_%7Bf%7D%5C%5C%5Ctheta_%7Bf%7D%5Cend%7Bpmatrix%7D%2B%5Cbegin%7Bpmatrix%7D%5Cmathcal%7BN%7D%280%2C%5Csigma_%7Bx%7D%5E%7B2%7D%29%5C%5C%5Cmathcal%7BN%7D%280%2C%5Csigma_%7By%7D%5E%7B2%7D%29%5C%5C%5Cmathcal%7BN%7D%280%2C%5Csigma_%7B%5Ctheta%7D%5E%7B2%7D%29%5Cend%7Bpmatrix%7D)

The prediction step is applied to all the particles.

**Update Step**

Coordinate frame transformation is required to convert the sensor observations in vehicle coordinate system into the same reference frame as the map landmarks. This is given by a homogeneous transformation as follows:


After the coordinate frame transformation, nearest neighbor data association is then performed to pick the right corresponding landmark and observation pairs. The figure below illustrates general concept of nearest neighbor data association.

<img src="/images/data_association.png" width="600">

The sensor observations is incorporated into the particle filter by updating the weight of each particle with the likelihood of the sensor observations. Likelihood of individual sensor observations is calculated via a multivariate Gaussian probability density function. 

By assuming individual sensor measurements are independent to each other, the likelihood of all the measurements are combined by taking their product, resulting the following equation:


**Resample**

