---
sidebar_position: 2
---

# Appendix B: Troubleshooting Guide

This appendix provides solutions to common issues encountered when working with Physical AI systems and the technologies covered in this textbook.

## Common ROS2 Installation and Runtime Issues

### Problem: ROS2 installation fails with missing dependencies

**Solution**: Ensure you're using Ubuntu 22.04 LTS and have updated your package lists:
```bash
sudo apt update && sudo apt upgrade
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update
```

### Problem: ROS2 commands not found after installation

**Solution**: Source the ROS2 setup script in your shell:
```bash
source /opt/ros/humble/setup.bash
# Or add to your ~/.bashrc file permanently:
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### Problem: Permission denied when running ROS2 commands

**Solution**: Check that you're in the correct workspace and have proper permissions:
```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

## Simulation Performance Optimization Tips

### Problem: Gazebo simulation running slowly

**Solution**:
1. Reduce the complexity of your robot model
2. Disable unnecessary sensors in your URDF
3. Reduce physics update rate in world file:
```xml
<physics type='ode'>
  <max_step_size>0.01</max_step_size>
  <real_time_update_rate>100</real_time_update_rate>
</physics>
```

### Problem: Unity simulation performance issues

**Solution**:
1. Lower rendering quality in Unity's Quality Settings
2. Reduce the number of active objects in the scene
3. Use occlusion culling for complex environments
4. Consider using Unity's built-in profiler to identify bottlenecks

## Network Configuration for Multi-Robot Systems

### Problem: Robots cannot communicate over network

**Solution**:
1. Ensure all robots are on the same network
2. Verify ROS_DOMAIN_ID is the same across all systems:
```bash
export ROS_DOMAIN_ID=0
```
3. Check firewall settings to allow ROS2 traffic
4. Verify network connection with ping command

## Camera and Sensor Calibration Procedures

### Camera Calibration Steps

1. Print or download a calibration pattern (checkerboard)
2. Use the camera_calibration package:
```bash
ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.108 image:=/camera/image_raw
```
3. Follow on-screen instructions to move the calibration pattern
4. Save the calibration file when completed

### LiDAR Calibration

1. Ensure LiDAR is properly mounted and secured
2. Use calibration tools specific to your LiDAR model
3. Verify TF transforms between LiDAR and robot base

## Robot Hardware Maintenance Guidelines

### Regular Maintenance Schedule

- **Daily**: Check battery levels and connection cables
- **Weekly**: Inspect wheels and drive systems for wear
- **Monthly**: Calibrate sensors and check mechanical joints
- **Quarterly**: Deep battery calibration and firmware updates

### Troubleshooting Hardware Issues

- If robot is not responding, check power connections first
- For erratic movement, verify encoder calibration
- For sensor issues, check for physical obstructions or damage
- Always power down robot before performing maintenance