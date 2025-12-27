---
sidebar_position: 4
---

# 3.2 Gazebo Simulation Environment Setup

## Chapter 3: Simulation Environments - Gazebo and Unity

Gazebo is a powerful 3D simulation environment that provides realistic physics simulation and sensor modeling for robotics applications. It's widely used in the robotics community due to its tight integration with ROS2 and its ability to simulate complex environments and robot interactions.

## Installing Gazebo Garden

Gazebo Garden is the latest stable version at the time of writing. Here's how to install it on Ubuntu 22.04:

### System Requirements
- Ubuntu 22.04 LTS
- Graphics card with OpenGL 2.1+ support
- Minimum 4GB RAM (8GB+ recommended)
- At least 2GB of free disk space

### Installation Steps

```bash
# 1. Update your system package index
sudo apt update

# 2. Install Gazebo Garden
sudo apt install gazebo-garden

# 3. Install ROS2 Gazebo packages
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control

# 4. Verify installation
gz --version
```

## Gazebo Architecture

Gazebo consists of several key components:

- **Gazebo Server (gzserver)**: The core simulation engine that handles physics, rendering, and simulation time
- **Gazebo Client (gzclient)**: The graphical user interface that visualizes the simulation
- **Gazebo Plugins**: Dynamic libraries that extend Gazebo's functionality
- **Transport Layer**: Facilitates communication between Gazebo components

### Basic Gazebo Commands

```bash
# Launch Gazebo with a specific world file
gz sim -r empty.sdf

# Launch Gazebo with GUI
gz sim -g

# Launch with specific world file
gz sim -r -v 1 my_world.sdf
```

## Launching Your First Simulation

### Starting with an Empty World

```bash
# Launch Gazebo with empty world
gz sim -r -v 1 empty.sdf
```

### Exploring the Gazebo Interface

The Gazebo interface consists of several key elements:

1. **Menu Bar**: Contains file operations, simulation controls, and plugins
2. **Toolbar**: Provides quick access to common tools
3. **Scene Tab**: Shows the 3D simulation environment
4. **Layer Tabs**: Shows model editor, world statistics, and connections
5. **Status Bar**: Displays simulation time and performance metrics

### Essential Controls

- **Right-click + drag**: Rotate view
- **Middle-click + drag**: Pan view
- **Scroll wheel**: Zoom in/out
- **Ctrl + Right-click + drag**: Move objects in the world
- **Spacebar**: Pause/resume simulation

## Creating a Simple World

World files in Gazebo are defined using SDF (Simulation Description Format). Here's a minimal example:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="simple_world">
    <!-- Include a ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>
    
    <!-- Include a light source -->
    <include>
      <uri>model://sun</uri>
    </include>
    
    <!-- Define a simple box -->
    <model name="box">
      <pose>0 0 0.5 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
          <material>
            <ambient>1 0 0 1</ambient>
            <diffuse>1 0 0 1</diffuse>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </collision>
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>1</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>1</iyy>
            <iyz>0</iyz>
            <izz>1</izz>
          </inertia>
        </inertial>
      </link>
    </model>
  </world>
</sdf>
```

Save this as `simple_world.sdf` and launch with:
```bash
gz sim -r simple_world.sdf
```

## Gazebo Simulation Parameters

### Physics Configuration

Physics parameters can be configured in the world file:

```xml
<physics type="ode">
  <max_step_size>0.001</max_step_size>
  <real_time_update_rate>1000</real_time_update_rate>
  <gravity>0 0 -9.8</gravity>
  <ode>
    <solver>
      <type>quick</type>
      <iters>10</iters>
      <sor>1.3</sor>
    </solver>
    <constraints>
      <cfm>0</cfm>
      <erp>0.2</erp>
      <contact_max_correcting_vel>100</contact_max_correcting_vel>
      <contact_surface_layer>0.001</contact_surface_layer>
    </constraints>
  </ode>
</physics>
```

### Simulation Speed Control

- **Real-time**: Simulation runs at the same speed as real time
- **Faster than real-time**: Simulation runs faster (e.g., 2x real-time speed)
- **Slower than real-time**: Simulation runs slower than real time
- **Paused**: Simulation is stopped

## Integrating with ROS2

Gazebo integrates seamlessly with ROS2 through the Gazebo ROS packages:

### Launching Gazebo with ROS2

```bash
# Terminal 1: Start Gazebo
ros2 launch gazebo_ros gazebo.launch.py

# Terminal 2: Spawn a model in Gazebo
ros2 run gazebo_ros spawn_entity.py -entity my_robot -file /path/to/robot.urdf -x 0 -y 0 -z 1
```

### Common ROS2 Gazebo Topics

- `/clock`: Simulation time
- `/gazebo/model_states`: States of all models in the simulation
- `/gazebo/link_states`: States of all links in the simulation
- `/gazebo/set_model_state`: Set model state
- `/gazebo/set_link_state`: Set link state

## Troubleshooting Common Issues

### Gazebo Won't Start

1. **Check graphics drivers**: Ensure you have compatible graphics drivers installed
2. **Verify installation**: Run `gz --version` to confirm installation
3. **Check permissions**: Ensure proper permissions for temporary directories

### Performance Issues

1. **Reduce model complexity**: Simplify visual and collision meshes
2. **Lower physics update rate**: Increase max_step_size in physics config
3. **Disable unnecessary sensors**: Remove unused sensors from URDF

### ROS2 Integration Issues

1. **Verify packages**: Ensure ros-humble-gazebo-* packages are installed
2. **Check environment**: Source ROS2 setup before launching
3. **Network issues**: Verify ROS_DOMAIN_ID and network configuration

## Best Practices

1. **Start Simple**: Begin with basic worlds and gradually add complexity
2. **Optimize Performance**: Balance realism with simulation speed
3. **Use Templates**: Leverage existing world files as templates
4. **Document Parameters**: Keep track of physics and environment settings
5. **Version Control**: Use Git to track changes to world files

## Summary

Setting up Gazebo properly is crucial for effective robotics simulation. This section covered the installation process, basic interface navigation, world creation, and integration with ROS2. Having a solid foundation in Gazebo setup will enable you to create complex simulated environments for testing and validating your Physical AI systems. The next step is learning to create robot models for use in these simulations.

## Exercises

1. Install Gazebo Garden on your system and verify the installation
2. Launch Gazebo with the default empty world and explore the interface
3. Create a simple world file with a few different objects and load it in Gazebo
4. Research and document the differences between ODE, Bullet, and Simbody physics engines