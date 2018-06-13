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

The particle filter is initialized by sampling from a Gaussian distribution around the initial GPS estimates. The GPS measurements include the x coordinate, y coordinate (both in m) and the theta (orientation) of vehicle in radian. The standard deviations ![](https://latex.codecogs.com/gif.latex?\sigma_{x}), ![](https://latex.codecogs.com/gif.latex?\sigma_{y}), ![](https://latex.codecogs.com/gif.latex?\sigma_{\theta}) of the Gaussian sampling are based on the GPS specification.

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

![](https://latex.codecogs.com/gif.latex?%5Cbegin%7Bpmatrix%7Dx_%7Bmap%7D%5C%5Cy_%7Bmap%7D%5C%5C1%5Cend%7Bpmatrix%7D%3D%5Cbegin%7Bpmatrix%7Dcos%5Ctheta_%7Bp%7D%26-sin%5Ctheta_%7Bp%7D%26x_%7Bp%7D%5C%5Csin%5Ctheta_%7Bp%7D%26cos%5Ctheta_%7Bp%7D%26y_%7Bp%7D%5C%5C0%260%261%5Cend%7Bpmatrix%7D%5Cbegin%7Bpmatrix%7Dx_%7Bcar%7D%5C%5Cy_%7Bcar%7D%5C%5C1%5Cend%7Bpmatrix%7D)

where ![](https://latex.codecogs.com/gif.latex?(x_{p},y_{p},\theta_{p})^{T}) is the particle or vehicle state in the map coordinate system, ![](https://latex.codecogs.com/gif.latex?(x_{map},y_{map})^{T}) is the measurement in map coordinate system and ![](https://latex.codecogs.com/gif.latex?(x_{car},y_{car})^{T}) is the measurement in vehicle coordinate system.

After the coordinate frame transformation, nearest neighbor data association is then performed to pick the right corresponding landmark and observation pairs. The figure below illustrates general concept of nearest neighbor data association.

<img src="/images/data_association.png" width="600">

The sensor observations is incorporated into the particle filter by updating the weight of each particle with the likelihood of the sensor observations. Likelihood of individual lidar observations is calculated via a bivariate Gaussian probability density function with zero correlation between the x and y measurement, ![](https://latex.codecogs.com/gif.latex?\sigma_{xy}=sigma_{yx}=0).

![](https://latex.codecogs.com/gif.latex?P(x,y;\mu_{x},\mu_{y},\sigma_{x}^{2},\sigma_{y}^{2})=\frac{1}{2\pi\sigma_{x}\sigma_{y}}e^{-\frac{1}{2}(\frac{(x-\mu_{x})^{2}}{\sigma_{x}^{2}}+\frac{(y-\mu_{y})^{2}}{\sigma_{y}^{2}})})

where ![](https://latex.codecogs.com/gif.latex?(x,y)^{T}) is a sensor observation in map coordinate system, ![](https://latex.codecogs.com/gif.latex?(\mu_{x},\mu_{y})^{T}) is the associated landmark coordinate in map coordinate sytem, ![](https://latex.codecogs.com/gif.latex?\sigma_{x}^{2}) and ![](https://latex.codecogs.com/gif.latex?\sigma_{y}^{2}) are the variance of the sensor measurements.

By assuming individual sensor measurements are independent to each other, the likelihood of all the measurements are combined by taking their product, and this likelihood is assigned as the weight of each individual particle.

![](https://latex.codecogs.com/gif.latex?w=\prod_{i=0}^{m}P(x_{i},y_{i};\mu_{x,i},\mu_{y,i},\sigma_{x}^{2},\sigma_{y}^{2}))

where ![](https://latex.codecogs.com/gif.latex?m) is the number of sensor observations at a time step.

**Resample**

