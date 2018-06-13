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

The number of particles to be sampled is a parameter to be determined empirically. The larger the number of particles, the particle filter will be closer to representing the Bayesian posterior distribution. However it comes at a cost of long processing time and preventing localization in real time. On the other hand, if the number of particles is too few, we will not have enough particles to cover all the high likelihood state space. The number of particles we used here is 100.

**Prediction Step**

The motion model used, i.e. bicycle model is as illustrated below:


The motion prediction for each time step, delta t, based on control inputs velocity and yaw rate, is governed by the following dynamics:

Yaw rate != 0

Yaw rate == 0

To account for the uncertainty in the control inputs, we will also add Gaussian noise to the velocity and yaw rate. 

The prediction step is applied to all the particles.

**Update Step**

Coordinate frame transformation is required to convert the sensor observations in vehicle coordinate system into the same reference frame as the map landmarks. This is given by a homogeneous transformation as follows:


After the coordinate frame transformation, data association is then performed to pick the right corresponding landmark and observa


**Resample**

