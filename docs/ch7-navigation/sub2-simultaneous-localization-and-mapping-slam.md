---
sidebar_position: 2
---

# Simultaneous Localization and Mapping (SLAM)

## Overview

Simultaneous Localization and Mapping (SLAM) is one of the most fundamental problems in robotics, addressing how a robot can build a map of an unknown environment while simultaneously localizing itself within that map. This chicken-and-egg problem is essential for autonomous navigation in previously unexplored environments.

## The SLAM Problem

### Mathematical Formulation

The SLAM problem can be formulated as estimating the robot's trajectory and the map of landmarks simultaneously:
```
P(x_t, m | z_1:t, u_1:t)
```
Where:
- x_t is the robot trajectory up to time t
- m is the map of landmarks
- z_1:t are the observations up to time t
- u_1:t are the control inputs up to time t

### The Data Association Problem

SLAM must solve the data association problem: determining which sensor measurements correspond to which landmarks in the map. This is challenging because:
- The map is initially unknown
- Sensor measurements are noisy
- Multiple similar landmarks may exist

## SLAM Approaches

### Filtering Approaches

**Extended Kalman Filter (EKF) SLAM**
- Maintains a mean and covariance for the joint state (robot pose + landmarks)
- Linearizes the motion and observation models
- Computational complexity grows quadratically with the number of landmarks

**Particle Filter SLAM (Rao-Blackwellized Particle Filter)**
- Uses particles to represent robot pose uncertainty
- Each particle maintains its own map estimate
- Better handles non-linearities and multi-modal distributions

### Smoothing Approaches

**Graph-Based SLAM**
- Formulates SLAM as a graph optimization problem
- Nodes represent robot poses, edges represent constraints
- More accurate than filtering approaches
- Can handle loop closures naturally

**Bundle Adjustment**
- Originally from computer vision
- Optimizes both camera poses and 3D point positions
- Computationally intensive but highly accurate

## Feature-Based SLAM

### Feature Extraction

Feature-based SLAM identifies and tracks distinctive landmarks in the environment:
- **Corners**: Geometric features in 2D/3D
- **Edges**: Boundaries between surfaces
- **Natural landmarks**: SIFT, SURF, ORB features
- **Artificial landmarks**: fiducial markers

### Feature Management

- **Initialization**: Adding new features to the map
- **Tracking**: Maintaining correspondence between features
- **Culling**: Removing features that are no longer observable

## Direct SLAM

### Direct Methods

Direct SLAM methods work with raw sensor data rather than extracted features:
- **Dense SLAM**: Uses all available pixels in images
- **Semi-dense SLAM**: Uses pixels with high gradient information
- **Lidar SLAM**: Uses raw point cloud data

### Advantages and Disadvantages

**Advantages:**
- No need for feature extraction
- Can work in textureless environments
- Potentially more accurate

**Disadvantages:**
- Computationally intensive
- Requires photometric calibration
- Sensitive to lighting changes

## Visual SLAM

### Monocular SLAM

Monocular systems use a single camera:
- **Scale ambiguity**: Cannot determine absolute scale
- **Initialization**: Requires initial motion to estimate scale
- **Representations**: Keyframe-based or direct methods

### Stereo and RGB-D SLAM

Stereo systems provide metric scale information:
- **ORB-SLAM**: Feature-based approach with loop closure
- **LSD-SLAM**: Direct method for large-scale environments
- **RTAB-Map**: RGB-D SLAM with appearance-based loop closure

## LiDAR SLAM

### 2D LiDAR SLAM

For ground robots operating in planar environments:
- **Hector SLAM**: Grid-based approach using scan matching
- **Gmapping**: EKF-based approach
- **Cartographer**: Real-time SLAM with submapping

### 3D LiDAR SLAM

For full 3D environments:
- **LOAM (Lidar Odometry and Mapping)**: Extracts features from 3D point clouds
- **LeGO-LOAM**: Lightweight version optimized for ground vehicles
- **LOAM_Livox**: Adapted for high-resolution LiDAR sensors

## Multi-Sensor SLAM

### Sensor Fusion

Combining multiple sensor modalities improves robustness:
- **Visual-Inertial SLAM**: Combines cameras with IMUs
- **LiDAR-Inertial SLAM**: Combines LiDAR with IMUs
- **Visual-LiDAR SLAM**: Combines visual and LiDAR data

### Complementary Sensors

Different sensors provide complementary information:
- Cameras: Rich visual information, poor in low-light
- LiDAR: Accurate geometric information, textureless
- IMU: High-frequency motion data, drifts over time

## Loop Closure

### Recognition and Verification

Loop closure detection is crucial for correcting accumulated errors:
- **Appearance-based**: Recognizing previously visited locations
- **Topological**: Detecting when the robot returns to a known area
- **Geometric verification**: Confirming the geometric consistency of matches

### Optimization

After loop closure detection, the full trajectory and map are optimized:
- **Pose Graph Optimization**: Optimizes robot poses
- **Bundle Adjustment**: Optimizes poses and landmarks simultaneously

## Challenges in SLAM

### Computational Complexity

SLAM algorithms must balance accuracy with real-time performance:
- **Sparsification**: Reducing the number of landmarks
- **Submapping**: Dividing the map into local submaps
- **Marginalization**: Removing old information to maintain efficiency

### Degenerate Cases

SLAM can fail in certain environments:
- **Symmetric environments**: Difficult to distinguish locations
- **Textureless environments**: Few distinctive features
- **Dynamic environments**: Moving objects affecting measurements

### Robustness

SLAM systems must handle:
- Sensor noise and outliers
- Dynamic objects in the environment
- Changing lighting conditions
- Sensor failures

## Evaluation Metrics

### Accuracy Metrics

- **ATE (Absolute Trajectory Error)**: Difference between estimated and ground truth trajectory
- **RPE (Relative Pose Error)**: Error in relative pose estimates
- **Map accuracy**: Difference between estimated and ground truth map

### Efficiency Metrics

- **Processing time**: Computation time per frame
- **Memory usage**: Storage required for the map
- **Scalability**: Performance as environment size increases

## Summary

SLAM is a fundamental capability for autonomous robots operating in unknown environments. Modern approaches combine multiple sensors and optimization techniques to achieve robust performance.

In the next section, we'll explore global and local path planning, which uses the maps created by SLAM for navigation.