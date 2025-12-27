# Chapter 5: Hands-on Labs & Troubleshooting

## Clear Explanation

This chapter provides practical hands-on labs to reinforce the concepts covered in the previous chapters and offers troubleshooting techniques for common issues encountered when working with the NVIDIA Isaac platform. The labs are designed to give students practical experience with Isaac Sim, synthetic data generation, Isaac ROS integration, and deployment scenarios, while the troubleshooting section addresses real-world problems that arise during development and deployment.

The hands-on approach allows students to apply theoretical knowledge to practical scenarios, building confidence and expertise in using the Isaac platform for robotics development. The troubleshooting section provides a systematic approach to identifying and resolving common issues, which is crucial for effective robotics development.

## Subsections

### 5.1 Lab 1: Isaac Sim Robot Simulation Setup

**Objective**: Create a complete simulation environment with a differential drive robot in Isaac Sim.

**Requirements**:
- Isaac Sim installation
- ROS2 Humble Hawksbill
- Python 3.8+
- Basic knowledge of USD and URDF

**Step-by-step Instructions**:

1. **Create a new ROS2 package for the robot**:
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python isaac_sim_demo --dependencies rclpy geometry_msgs sensor_msgs std_msgs
```

2. **Set up Isaac Sim environment**:
```python
# In your Python script
import omni
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.prims import get_prim_at_path
import carb

# Initialize Isaac Sim
omni.kit.commands.execute("RequestQuit", save_changes=False)
my_world = World(stage_units_in_meters=1.0)

# Access robot assets
assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets. Please check your installation.")
else:
    # Add a pre-built robot to the scene
    robot_path = assets_root_path + "/Isaac/Robots/Carter/carter_vda5.usd"
    add_reference_to_stage(
        usd_path=robot_path,
        prim_path="/World/Robot"
    )
    
    # Add ground plane
    my_world.scene.add_default_ground_plane()
    
    # Reset the world
    my_world.reset()
```

3. **Create ROS2 interface for the robot**:
```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Image
from cv_bridge import CvBridge
import numpy as np

class IsaacRobotController(Node):
    def __init__(self):
        super().__init__('isaac_robot_controller')
        
        # Create CV bridge for image conversion
        self.cv_bridge = CvBridge()
        
        # Subscribe to velocity commands
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Publish sensor data
        self.laser_pub = self.create_publisher(LaserScan, 'scan', 10)
        self.camera_pub = self.create_publisher(Image, 'camera/image_raw', 10)
        
        # Timer for sensor publishing
        self.timer = self.create_timer(0.1, self.publish_sensor_data)
        
        # Robot state
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        
    def cmd_vel_callback(self, msg):
        self.linear_velocity = msg.linear.x
        self.angular_velocity = msg.angular.z
        
    def publish_sensor_data(self):
        # Publish mock laser scan data
        laser_msg = LaserScan()
        laser_msg.header.stamp = self.get_clock().now().to_msg()
        laser_msg.header.frame_id = 'laser_scanner'
        laser_msg.angle_min = -np.pi/2
        laser_msg.angle_max = np.pi/2
        laser_msg.angle_increment = np.pi/180  # 1 degree
        laser_msg.range_min = 0.1
        laser_msg.range_max = 10.0
        laser_msg.ranges = [2.0 + np.random.uniform(-0.1, 0.1) for _ in range(181)]
        
        self.laser_pub.publish(laser_msg)
        
        # Publish mock camera image
        # In a real implementation, this would come from Isaac Sim sensor
        img_data = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        img_msg = self.cv_bridge.cv2_to_imgmsg(img_data, encoding="bgr8")
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.header.frame_id = 'camera'
        
        self.camera_pub.publish(img_msg)

def main(args=None):
    rclpy.init(args=args)
    controller = IsaacRobotController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 5.2 Lab 2: Isaac ROS Perception Pipeline

**Objective**: Set up and test Isaac ROS perception pipeline with GPU acceleration.

**Step-by-step Instructions**:

1. **Launch Isaac ROS stereo processing**:
```bash
# Terminal 1: Launch Isaac Sim with a stereo camera
ros2 launch isaac_sim_examples stereo_camera.launch.py

# Terminal 2: Launch Isaac ROS stereo rectification
ros2 launch isaac_ros_stereo_image_rectification stereo_image_rectification.launch.py \
    left_image_topic:=/front_stereo_camera/left/image_raw \
    right_image_topic:=/front_stereo_camera/right/image_raw \
    left_camera_info_url:=file://$(ros2 pkg prefix isaac_ros_stereo_image_rectification)/share/isaac_ros_stereo_image_rectification/config/camera_info_left.yaml \
    right_camera_info_url:=file://$(ros2 pkg prefix isaac_ros_stereo_image_rectification)/share/isaac_ros_stereo_image_rectification/config/camera_info_right.yaml
```

2. **Run disparity estimation**:
```bash
# Terminal 3: Launch Isaac ROS stereo disparity
ros2 launch isaac_ros_stereo_disparity stereo_disparity_node.launch.py
```

### 5.3 Lab 3: Synthetic Data Generation Pipeline

**Objective**: Create a synthetic dataset using Isaac Sim for object detection.

**Step-by-step Instructions**:

1. **Set up the synthetic data generation script**:
```python
import omni
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.synthetic_utils import SyntheticDataHelper
from omni.isaac.core.utils.prims import create_prim
from omni.isaac.core.utils.nucleus import get_assets_root_path
import numpy as np
import cv2
import os
import random

# Initialize world
my_world = World(stage_units_in_meters=1.0)
sd_helper = SyntheticDataHelper()

# Create objects for the scene
def create_random_object():
    # Randomly place objects in the scene
    x = random.uniform(-2.0, 2.0)
    y = random.uniform(-2.0, 2.0)
    z = random.uniform(0.5, 2.0)
    
    object_types = ["Cylinder", "Cube", "Sphere"]
    obj_type = random.choice(object_types)
    
    create_prim(
        prim_path=f"/World/Object_{len([x for x in World.get_current_stage().GetPrimAtPath('/World').GetChildren() if 'Object' in str(x)])}",
        prim_type=obj_type,
        position=[x, y, z],
        scale=[0.2, 0.2, 0.2]
    )

# Add multiple random objects to the scene
for i in range(10):
    create_random_object()

# Add ground plane and lighting
my_world.scene.add_default_ground_plane()

# Enable ground truth annotations
sd_helper.enable_ground_truth_annotations([
    "bounding_box_2d_tight",
    "instance_segmentation",
    "depth_linear"
])

def capture_synthetic_data(num_frames=100):
    # Create output directories
    os.makedirs("synthetic_dataset/images", exist_ok=True)
    os.makedirs("synthetic_dataset/annotations", exist_ok=True)
    
    for i in range(num_frames):
        # Randomize camera position
        camera_x = random.uniform(-1.0, 1.0)
        camera_y = random.uniform(-1.0, 1.0)
        camera_z = random.uniform(1.0, 2.0)
        
        # In a real implementation, you would update the camera position here
        # update_camera_position([camera_x, camera_y, camera_z])
        
        # Step the world
        my_world.step(render=True)
        
        # Capture RGB image
        rgb_buffer = sd_helper.get_rgb_buffer()
        cv2.imwrite(f"synthetic_dataset/images/frame_{i:05d}.png", 
                   cv2.cvtColor(rgb_buffer, cv2.COLOR_RGB2BGR))
        
        # Capture annotations
        bbox_data = sd_helper.get_bounding_box_2d_tight()
        seg_data = sd_helper.get_segmentation()
        depth_data = sd_helper.get_depth_linear()
        
        # Save annotations
        np.save(f"synthetic_dataset/annotations/bbox_{i:05d}.npy", bbox_data)
        cv2.imwrite(f"synthetic_dataset/annotations/seg_{i:05d}.png", seg_data)
        np.save(f"synthetic_dataset/annotations/depth_{i:05d}.npy", depth_data)
        
        print(f"Captured frame {i+1}/{num_frames}")

# Run the data capture
capture_synthetic_data()
```

### 5.4 Common Isaac Sim Issues and Solutions

**Problem 1: Isaac Sim fails to start or crashes**
- **Cause**: GPU driver issues, insufficient memory, or incorrect installation
- **Solution**: Verify GPU compatibility and driver version; increase available memory; reinstall Isaac Sim

**Problem 2: USD files don't load properly**
- **Cause**: Incompatible USD format or missing dependencies
- **Solution**: Ensure USD files are compatible with Isaac Sim's supported format; install required USD dependencies

**Problem 3: Sensor data appears incorrect or delayed**
- **Cause**: Incorrect sensor configuration or simulation timing issues
- **Solution**: Verify sensor positions and properties; adjust simulation frequency; check sensor calibration

### 5.5 Common Isaac ROS Issues and Solutions

**Problem 1: Isaac ROS nodes fail to launch**
- **Cause**: Missing dependencies, incorrect GPU setup, or incompatible CUDA version
- **Solution**: Verify Isaac ROS installation; check CUDA and GPU driver compatibility; reinstall if necessary

**Problem 2: Performance is slower than expected**
- **Cause**: CPU bottleneck, memory limitations, or suboptimal algorithm parameters
- **Solution**: Verify GPU usage; optimize parameters; check for CPU bottlenecks in the pipeline

**Problem 3: Integration with standard ROS2 nodes fails**
- **Cause**: Message type incompatibilities or timing issues
- **Solution**: Verify message type compatibility; check timing and synchronization; use appropriate adapters or bridges

[DIAGRAM: Troubleshooting Flowchart - Decision tree for common Isaac platform issues]

## Example Snippets

### Isaac Sim Debugging Script
```python
import omni
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.prims import get_prim_at_path
import carb

def debug_isaac_sim():
    """Debugging script for Isaac Sim issues"""
    try:
        # Initialize world
        my_world = World(stage_units_in_meters=1.0)
        print("✓ Isaac Sim initialized successfully")
        
        # Check if assets root path is accessible
        assets_root_path = get_assets_root_path()
        if assets_root_path:
            print(f"✓ Assets root path: {assets_root_path}")
        else:
            print("✗ Could not find Isaac Sim assets")
            return
        
        # Add default ground plane
        my_world.scene.add_default_ground_plane()
        print("✓ Ground plane added successfully")
        
        # Reset the world
        my_world.reset()
        print("✓ World reset successfully")
        
        # Step the simulation
        my_world.step(render=False)
        print("✓ Simulation step executed successfully")
        
        # Clean up
        my_world.clear()
        print("✓ Isaac Sim debug completed successfully")
        
    except Exception as e:
        print(f"✗ Isaac Sim debug failed: {str(e)}")
        carb.log_error(f"Isaac Sim debug error: {str(e)}")

if __name__ == "__main__":
    debug_isaac_sim()
```

### Isaac ROS Performance Monitor
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float32
import time

class PerformanceMonitor(Node):
    def __init__(self):
        super().__init__('performance_monitor')
        
        # Publishers for performance metrics
        self.fps_pub = self.create_publisher(Float32, 'fps', 10)
        self.processing_time_pub = self.create_publisher(Float32, 'processing_time', 10)
        
        # Subscribers for performance tracking
        self.image_sub = self.create_subscription(
            Image,
            'input_image',
            self.image_callback,
            10
        )
        
        # Performance tracking variables
        self.frame_count = 0
        self.start_time = time.time()
        self.last_image_time = 0
        
    def image_callback(self, msg):
        current_time = time.time()
        
        # Calculate FPS
        self.frame_count += 1
        elapsed = current_time - self.start_time
        if elapsed > 0:
            fps = self.frame_count / elapsed
        else:
            fps = 0.0
            
        # Calculate processing time
        if self.last_image_time > 0:
            processing_time = current_time - self.last_image_time
        else:
            processing_time = 0.0
            
        self.last_image_time = current_time
        
        # Publish metrics
        fps_msg = Float32()
        fps_msg.data = float(fps)
        self.fps_pub.publish(fps_msg)
        
        proc_time_msg = Float32()
        proc_time_msg.data = float(processing_time)
        self.processing_time_pub.publish(proc_time_msg)
        
        # Log performance if it drops below threshold
        if fps < 10.0:  # Below 10 FPS
            self.get_logger().warn(f'Performance degradation detected: {fps:.2f} FPS')

def main(args=None):
    rclpy.init(args=args)
    monitor = PerformanceMonitor()
    
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    
    monitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Isaac ROS Troubleshooting Commands
```bash
# Check Isaac ROS packages installation
dpkg -l | grep isaac-ros

# Verify GPU availability
nvidia-smi

# Check Isaac ROS nodes
ros2 component types | grep isaac_ros

# Launch with verbose logging
ros2 launch isaac_ros_visual_slam visual_slam.launch.py log_level:=debug

# Check GPU memory usage during Isaac ROS operation
watch -n 1 nvidia-smi

# Verify Isaac ROS message types
ros2 interface show isaac_ros_visual_slam_msgs/msg/IsaacROSVisualSLAMResult
```

## Diagram Placeholders

[DIAGRAM: Troubleshooting Flowchart - Step-by-step decision tree for diagnosing Isaac platform issues]

[DIAGRAM: Performance Comparison - Isaac ROS vs Traditional ROS Performance]

[DIAGRAM: Integration Testing - Verifying the complete Isaac platform pipeline]

## Summary

This chapter provided hands-on labs for the Isaac platform, with practical examples for setting up simulation environments, implementing perception pipelines, and generating synthetic data. We also covered common troubleshooting techniques for resolving issues in both Isaac Sim and Isaac ROS environments.

The labs demonstrated the complete workflow from creating simulation environments to implementing perception and navigation pipelines, providing practical experience with the Isaac platform. The troubleshooting section provided systematic approaches to diagnosing and resolving common issues, which are essential skills for effective robotics development using the Isaac platform.

With the completion of this module, you now have a comprehensive understanding of the NVIDIA Isaac platform, with practical skills in simulation, synthetic data generation, and accelerated perception and navigation using Isaac tools.