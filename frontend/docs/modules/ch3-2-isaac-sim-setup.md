# Chapter 2: Isaac Sim Environment & Setup

## Clear Explanation

Isaac Sim is NVIDIA's advanced robotics simulation environment built on the Omniverse platform. It provides a physically accurate and visually realistic environment for testing and validating robotic applications before deployment to real hardware. Isaac Sim excels in creating complex scenarios with photorealistic rendering, accurate physics simulation, and high-fidelity sensor models, making it ideal for developing perception systems, training AI models, and testing navigation algorithms.

The simulation environment supports various robot types, from wheeled mobile robots to manipulators and humanoids. It provides realistic sensor simulation including cameras, LiDAR, IMU, GPS, and force/torque sensors. The platform also supports domain randomization techniques that help bridge the sim-to-real gap by training models on varied environments and conditions.

## Subsections

### 2.1 Isaac Sim Architecture

Isaac Sim builds upon NVIDIA's Omniverse platform, leveraging:

- **PhysX Physics Engine**: Provides accurate rigid body dynamics and collision detection
- **RTX Rendering**: Enables photorealistic rendering with ray tracing and global illumination
- **USD (Universal Scene Description)**: Standard format for 3D scene representation and collaboration
- **Omniverse Kit**: Modular framework for building custom simulation applications
- **Microservices Architecture**: Scalable backend services for large-scale simulations

### 2.2 Installation and Setup

**System Requirements:**
- NVIDIA GPU with ray-tracing capabilities (RTX series recommended)
- 16GB+ RAM for complex scenes
- Ubuntu 20.04/22.04 or Windows 10/11
- NVIDIA driver 470 or higher

**Installation Options:**
1. **Docker Container (Recommended)**:
```bash
# Pull Isaac Sim Docker image
docker pull nvcr.io/nvidia/isaac-sim:latest

# Run Isaac Sim with GPU support
docker run --gpus all -it --rm \
    --network=host \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --privileged \
    --name=isaac-sim \
    nvcr.io/nvidia/isaac-sim:latest
```

2. **Native Installation**:
- Download from NVIDIA Developer Zone
- Follow the native installation guide for your platform
- Install required dependencies (Python, CUDA, etc.)

### 2.3 Creating Robot Models for Isaac Sim

Isaac Sim uses USD (Universal Scene Description) format for 3D models. Robot models can be imported from URDF files or created directly in Isaac Sim:

**Converting URDF to USD:**
```bash
# Use URDF Importer extension in Isaac Sim
# Or use the command line tool:
python -m omni.isaac.urdf_importer --config=robot_config.yaml
```

**Basic Robot Model Structure in USD:**
```
Robot
├── BaseLink
│   ├── Chassis
│   ├── Sensors
│   └── Actuators
└── Joints
    ├── Wheel_Left
    └── Wheel_Right
```

### 2.4 Configuring Sensors in Isaac Sim

Isaac Sim provides realistic sensor simulation for various types:

**Camera Configuration:**
```python
# Create RGB camera
from omni.isaac.sensor import Camera

camera = Camera(
    prim_path="/World/Robot/Camera",
    frequency=30,
    resolution=(640, 480)
)

# Configure intrinsic parameters
camera.config_intrinsic_matrix(
    focal_length_x=600,
    focal_length_y=600,
    principal_point_x=320,
    principal_point_y=240
)
```

**LiDAR Configuration:**
```python
# Create 2D LiDAR
from omni.isaac.range_sensor import _range_sensor

lidar_interface = _range_sensor.acquire_lidar_sensor_interface()
lidar_config = {
    "class": "OrbbecAstra",
    "name": "front_2d_lidar",
    "rotation": [0, 0, 0],
    "position": [0.2, 0, 0.1]
}
```

### 2.5 Environment Setup and Scene Creation

Creating realistic environments in Isaac Sim involves multiple components:

- **Ground Planes**: Configured with appropriate friction and material properties
- **Obstacles**: Static and dynamic objects with physics properties
- **Lighting**: HDRI lighting for photorealistic rendering
- **Weather Conditions**: Configurable environmental conditions

[DIAGRAM: Isaac Sim Interface - Showing the main components and user interface of Isaac Sim]

## Example Snippets

### Python API for Isaac Sim Setup
```python
import omni
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.nucleus import get_assets_root_path

# Initialize the world
my_world = World(stage_units_in_meters=1.0)

# Add robot to the scene
assets_root_path = get_assets_root_path()
if assets_root_path is None:
    print("Could not find Isaac Sim assets. Please check your installation.")
else:
    # Add a pre-built robot
    robot_path = assets_root_path + "/Isaac/Robots/Carter/carter_vda5.usd"
    add_reference_to_stage(
        usd_path=robot_path,
        prim_path="/World/Robot"
    )

# Configure simulation settings
my_world.scene.add_default_ground_plane()
my_world.reset()

# Start simulation
for i in range(1000):
    my_world.step(render=True)
```

### Custom Environment Setup
```python
from pxr import Gf, UsdGeom, Sdf
from omni.isaac.core.utils.prims import create_prim
from omni.isaac.core.utils.stage import get_current_stage

# Create a custom environment
stage = get_current_stage()

# Create a static box obstacle
create_prim(
    prim_path="/World/Obstacle",
    prim_type="Cube",
    position=[2.0, 0.0, 0.5],
    orientation=[0, 0, 0, 1],
    scale=[1, 1, 1]
)

# Add textured ground plane
ground_prim = UsdGeom.Xform.Define(stage, Sdf.Path("/World/Ground"))
```

### Domain Randomization Setup
```python
from omni.isaac.core.utils.carb import set_carb_setting

# Enable domain randomization for training
set_carb_setting("domain_randomization", "enabled", True)

# Randomize lighting conditions
set_carb_setting("domain_randomization", "lighting_variation", 0.3)

# Randomize material properties
set_carb_setting("domain_randomization", "material_variation", 0.2)

# Randomize robot appearance
set_carb_setting("domain_randomization", "robot_visual_variation", 0.1)
```

## Diagram Placeholders

[DIAGRAM: Isaac Sim Architecture - Showing the interconnection between USD, PhysX, RTX, and Omniverse components]

[DIAGRAM: Robot Model Import Process - From URDF to USD in Isaac Sim]

[DIAGRAM: Simulation Environment Setup - How to configure scenes, lighting, and objects]

## Summary

This chapter covered the setup and configuration of Isaac Sim, NVIDIA's advanced robotics simulation environment. You learned about the architecture, installation process, robot model import procedures, sensor configuration, and environment setup. The next chapter will explore synthetic data generation techniques in Isaac Sim for training AI models.