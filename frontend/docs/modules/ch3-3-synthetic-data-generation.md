# Chapter 3: Synthetic Data Generation

## Clear Explanation

Synthetic data generation is a cornerstone of modern AI development in robotics, particularly when real-world data is scarce, expensive to collect, or potentially dangerous to gather. Isaac Sim provides powerful tools for generating high-quality synthetic datasets that can be used to train perception models, navigation algorithms, and other AI components for robotic systems.

The process involves creating photo-realistic simulations of the robot's environment and generating corresponding ground truth annotations for various tasks such as object detection, segmentation, depth estimation, and pose estimation. By varying environmental conditions, lighting, and scene configurations, synthetic data generation can produce diverse datasets that improve the robustness and generalizability of AI models.

## Subsections

### 3.1 Introduction to Synthetic Data in Robotics

Synthetic data generation addresses several challenges in robotics:

- **Data Scarcity**: Acquiring sufficient labeled real-world data for training
- **Safety**: Training models for dangerous scenarios without physical risk
- **Cost**: Reducing the expense of data collection campaigns
- **Variety**: Creating diverse scenarios that would be difficult to encounter in real life
- **Ground Truth**: Automatically generating accurate annotations for training

### 3.2 Isaac Sim Synthetic Data Pipeline

The synthetic data generation pipeline in Isaac Sim includes:

**1. Scene Generation**: Creating diverse and realistic environments
- Randomized lighting conditions
- Varied textures and materials
- Different times of day and weather conditions
- Multiple environmental layouts

**2. Annotation Generation**: Automatically creating ground truth labels
- Bounding boxes for object detection
- Pixel-wise segmentation masks
- 3D bounding boxes for 3D object detection
- Depth maps and surface normals
- Instance segmentation masks

**3. Domain Randomization**: Varying simulation parameters to increase robustness
- Material properties and textures
- Lighting conditions and shadows
- Camera angles and positions
- Environmental conditions

### 3.3 Configuring Synthetic Data Generation

**Setting up a Basic Synthetic Data Pipeline:**
```python
import omni
from omni.isaac.synthetic_utils import SyntheticDataHelper
from omni.isaac.synthetic_utils.sensors import *

# Initialize synthetic data helper
sd_helper = SyntheticDataHelper()

# Configure camera parameters
camera_config = {
    "resolution": (1920, 1080),
    "focal_length": 15.0,
    "sensor_tilt": 0.0,
    "focus_distance": 10.0,
    "f_stop": 0.0
}

# Enable ground truth rendering
sd_helper.enable_ground_truth_annotations([
    "bounding_box_2d_tight",
    "instance_segmentation",
    "depth_linear",
    "normal"
])
```

### 3.4 Data Annotation Types

Isaac Sim supports multiple annotation formats:

**Semantic Segmentation**: Pixel-wise labeling of object classes
**Instance Segmentation**: Pixel-wise labeling differentiating individual objects
**Bounding Boxes**: 2D rectangular regions around objects
**3D Bounding Boxes**: 3D cuboids around objects with pose information
**Depth Maps**: Per-pixel distance measurements
**Surface Normals**: Surface orientation information

### 3.5 Domain Randomization Techniques

Domain randomization helps reduce the reality gap by introducing variations during training:

- **Material Randomization**: Varying textures, colors, and surface properties
- **Lighting Randomization**: Changing light positions, colors, and intensities
- **Weather Randomization**: Simulating different atmospheric conditions
- **Camera Randomization**: Varying camera parameters and positions

[DIAGRAM: Synthetic Data Generation Process - From simulation to annotated dataset]

## Example Snippets

### Basic Synthetic Data Generation Script
```python
import omni
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.synthetic_utils import SyntheticDataHelper
import numpy as np
import cv2
import os

# Initialize the world
my_world = World(stage_units_in_meters=1.0)

# Add robot and objects to the scene
assets_root_path = get_assets_root_path()
robot_path = assets_root_path + "/Isaac/Robots/Carter/carter_vda5.usd"
add_reference_to_stage(usd_path=robot_path, prim_path="/World/Robot")

# Set up camera
camera_path = "/World/Robot/base_link/camera"
my_world.scene.add_sensor("camera", camera_path)

# Initialize synthetic data helper
sd_helper = SyntheticDataHelper()

def capture_synthetic_data(num_images=100):
    for i in range(num_images):
        # Randomize environment
        randomize_environment()
        
        # Step the world
        my_world.step(render=True)
        
        # Capture RGB image
        rgb_image = sd_helper.get_rgb_buffer()
        
        # Capture ground truth annotations
        bbox_data = sd_helper.get_bounding_box_2d_tight()
        seg_data = sd_helper.get_segmentation()
        depth_data = sd_helper.get_depth_linear()
        
        # Save data
        save_data(rgb_image, bbox_data, seg_data, depth_data, i)

def save_data(rgb, bbox, seg, depth, idx):
    # Save RGB image
    cv2.imwrite(f"images/rgb_{idx:05d}.png", cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
    
    # Save annotations
    np.save(f"annotations/bbox_{idx:05d}.npy", bbox)
    cv2.imwrite(f"annotations/seg_{idx:05d}.png", seg)
    np.save(f"annotations/depth_{idx:05d}.npy", depth)

def randomize_environment():
    # Randomize object positions
    # Randomize lighting
    # Randomize textures and materials
    pass

# Start data generation
capture_synthetic_data()
```

### Domain Randomization Implementation
```python
import random
from omni.isaac.core.utils.prims import get_prim_at_path
from omni.isaac.core.materials import PreviewSurface

def randomize_materials():
    """Randomize materials in the scene"""
    materials = ["/World/Materials/Material1", "/World/Materials/Material2"]
    
    for mat_path in materials:
        material = get_prim_at_path(mat_path)
        if material:
            # Randomize color
            color = [random.random(), random.random(), random.random()]
            material.GetAttribute("inputs:diffuse_tint").Set(color)
            
            # Randomize roughness
            roughness = random.uniform(0.1, 1.0)
            material.GetAttribute("inputs:roughness").Set(roughness)

def randomize_lights():
    """Randomize lighting in the scene"""
    # Get directional light
    light_prim = get_prim_at_path("/World/DistantLight")
    
    if light_prim:
        # Randomize direction
        azimuth = random.uniform(0, 2 * 3.14159)
        elevation = random.uniform(0, 3.14159/2)
        
        # Convert to cartesian coordinates
        direction = [
            math.cos(azimuth) * math.cos(elevation),
            math.sin(azimuth) * math.cos(elevation),
            math.sin(elevation)
        ]
        light_prim.GetAttribute("xformOp:orient").Set(direction)
        
        # Randomize intensity
        intensity = random.uniform(1000, 5000)
        light_prim.GetAttribute("inputs:intensity").Set(intensity)

def randomize_camera_position():
    """Randomize camera position around the robot"""
    camera_path = "/World/Robot/base_link/camera"
    camera_prim = get_prim_at_path(camera_path)
    
    if camera_prim:
        # Randomize position around robot
        radius = random.uniform(0.8, 1.2)
        angle = random.uniform(0, 2 * 3.14159)
        
        x = radius * math.cos(angle)
        y = radius * math.sin(angle)
        z = random.uniform(0.5, 1.0)
        
        camera_prim.GetAttribute("xformOp:translate").Set([x, y, z])
```

### Synthetic Dataset Verification
```python
import json
import cv2
import numpy as np

def verify_synthetic_dataset(dataset_path):
    """Verify the quality and integrity of synthetic dataset"""
    # Load annotation info
    with open(f"{dataset_path}/annotations.json", 'r') as f:
        annotations = json.load(f)
    
    issues = []
    
    for annotation in annotations:
        image_path = f"{dataset_path}/images/{annotation['image']}"
        
        # Check if image exists
        if not os.path.exists(image_path):
            issues.append(f"Missing image: {image_path}")
            continue
        
        # Load image
        img = cv2.imread(image_path)
        height, width = img.shape[:2]
        
        # Verify bounding boxes are within image bounds
        for bbox in annotation['bboxes']:
            x1, y1, x2, y2 = bbox['bbox']
            if x1 < 0 or y1 < 0 or x2 > width or y2 > height:
                issues.append(f"Bounding box out of bounds in {image_path}")
            
            if x1 >= x2 or y1 >= y2:
                issues.append(f"Invalid bounding box in {image_path}")
    
    # Report verification results
    if issues:
        print("Dataset verification issues found:")
        for issue in issues:
            print(f"  - {issue}")
    else:
        print("Dataset verification passed: All data appears valid")
    
    return len(issues) == 0
```

## Diagram Placeholders

[DIAGRAM: Synthetic Data Pipeline - Showing the complete flow from simulation to dataset generation]

[DIAGRAM: Domain Randomization - How variations in simulation improve real-world performance]

[DIAGRAM: Annotation Types - Different types of ground truth annotations available in Isaac Sim]

## Summary

This chapter explored synthetic data generation in Isaac Sim, covering its importance in robotics, the pipeline for generating datasets, and techniques for improving model robustness through domain randomization. You learned how to configure and implement synthetic data generation pipelines, with practical examples for various annotation types. The next chapter will cover Isaac ROS integration and implementation of key algorithms.