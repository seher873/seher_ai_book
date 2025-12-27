---
sidebar_position: 3
---

# Chapter 3: Simulation Environments - Gazebo and Unity

## Chapter Purpose

This chapter explores simulation platforms essential for developing and testing physical AI systems safely and cost-effectively. Students will learn to create and configure robot models in simulation environments, implement physics-based interactions, and understand the transfer of skills from simulation to real-world deployment.

## Learning Outcomes

After completing this chapter, you will be able to:
- Create and configure robot models in simulation environments
- Implement physics-based interactions in simulated worlds
- Debug and optimize robot behaviors in simulation before real-world deployment
- Compare and contrast Gazebo and Unity simulation environments for different use cases

## Prerequisites

Before starting this chapter, you should:
- Have completed Chapter 1 (ROS2 basics)
- Understand robot modeling concepts from Chapter 2
- Have basic programming skills in Python and C++

## 3.1 Introduction to Robotic Simulation

Robotic simulation is a fundamental tool for developing and testing physical AI systems. Simulation provides:

- A safe environment to test algorithms without risk to hardware
- Faster iteration cycles than physical testing
- Controlled conditions to validate specific scenarios
- Cost-effective development before real-world deployment

Simulation is especially important for Physical AI systems because:

1. **Safety**: Testing navigation and manipulation in a safe environment
2. **Cost**: Reducing wear on expensive hardware and components
3. **Speed**: Rapidly iterating on algorithms without physical constraints
4. **Repeatability**: Exact reproduction of test conditions
5. **Scalability**: Testing with multiple robot instances simultaneously

### Types of Simulation

There are different levels of simulation fidelity:

- **Kinematic Simulation**: Focuses on motion without physics
- **Dynamic Simulation**: Includes mass, friction, and forces
- **Sensor Simulation**: Models sensor data generation
- **High-Fidelity Simulation**: Detailed physics and rendering

## 3.2 Gazebo Simulation Environment Setup

Gazebo is the standard simulation environment for ROS2, offering:

- Tight integration with ROS2 communication
- Physics engines (ODE, Bullet, Simbody)
- Sensor simulation (cameras, LiDAR, IMU)
- Plugin architecture for custom behaviors

### Installing Gazebo Garden

```bash
# Update package list
sudo apt update

# Install Gazebo Garden
sudo apt install gazebo-garden
```

### Basic Gazebo Components

Gazebo consists of:

- **Gazebo Server**: Physics engine and simulation core
- **Gazebo Client**: Visualization and user interface
- **Gazebo Plugins**: Extend functionality via ROS2 interfaces

### Starting Gazebo

```bash
# Start Gazebo server in background
gz sim -s

# Or launch with a specific world
gz sim -r -v 1 empty.sdf
```

## 3.3 Creating Robot Models for Gazebo

Robot models in Gazebo use the URDF (Unified Robot Description Format) with additional Gazebo-specific tags.

### Basic URDF Structure

```xml
<?xml version="1.0"?>
<robot name="my_robot">
  <!-- Links define the physical components -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>
  
  <!-- Joints connect links -->
  <joint name="base_joint" type="fixed">
    <parent link="base_link"/>
    <child link="sensor_mount"/>
  </joint>
  
  <link name="sensor_mount"/>
</robot>
```

### Adding Gazebo-Specific Elements

```xml
<gazebo reference="sensor_mount">
  <sensor name="camera" type="camera">
    <pose>0 0 0 0 0 0</pose>
    <camera>
      <horizontal_fov>1.047</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
      </image>
      <clip>
        <near>0.1</near>
        <far>100</far>
      </clip>
    </camera>
  </sensor>
</gazebo>
```

## 3.4 Physics Modeling and Environment Creation

Physics modeling in Gazebo involves configuring physical properties and environmental conditions.

### Physics Engine Configuration

```xml
<physics type="ode">
  <max_step_size>0.001</max_step_size>
  <real_time_update_rate>1000</real_time_update_rate>
  <gravity>0 0 -9.8</gravity>
</physics>
```

### Creating Custom Worlds

World files define the environment:

```xml
<sdf version="1.7">
  <world name="my_world">
    <!-- Include models from Gazebo database -->
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <include>
      <uri>model://sun</uri>
    </include>
    
    <!-- Place custom robot -->
    <include>
      <uri>model://my_robot</uri>
      <pose>0 0 0 0 0 0</pose>
    </include>
    
    <!-- Physics configuration -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_update_rate>1000</real_time_update_rate>
      <gravity>0 0 -9.8</gravity>
    </physics>
  </world>
</sdf>
```

## 3.5 Unity ML-Agents Toolkit for Robotics

Unity ML-Agents provides a framework for developing embodied AI agents using Unity's 3D engine.

### Key Features

- **High-quality 3D rendering**: Realistic visual environments
- **Physics simulation**: Built-in physics engine
- **RL integration**: Direct integration with ML frameworks
- **Robotics API**: Specialized tools for robotics

### Installation and Setup

```bash
# Install Unity Hub and Unity 2022.3 LTS
# Create new 3D project
# Import ML-Agents package via Package Manager
```

### Basic Unity Robot Setup

```csharp
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;

public class RobotAgent : Agent
{
    public override void OnEpisodeBegin()
    {
        // Reset robot to initial state
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // Add sensor observations
        sensor.AddObservation(GetComponentsInChildren<CameraSensor>());
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        // Process actions to control robot
        var continuousActions = actions.ContinuousActions;
        // Apply continuousActions to robot joints
    }
}
```

## 3.6 Comparing Gazebo vs Unity for Specific Use Cases

### When to Choose Gazebo

**Advantages:**
- Native ROS2 integration
- Realistic physics simulation
- Established robotics ecosystem
- Sensor simulation capabilities

**Best for:**
- ROS2-based development
- Real-robot testing and deployment
- Sensor-focused applications
- Academic and research environments

### When to Choose Unity ML-Agents

**Advantages:**
- High-quality rendering
- Advanced graphics capabilities
- Strong ML framework integration
- Cross-platform deployment

**Best for:**
- Visual perception tasks
- Reinforcement learning
- User interaction scenarios
- High-fidelity visualization

### Sim-to-Real Transfer Considerations

Transferring skills from simulation to reality requires:

1. **Domain Randomization**: Varying visual and physical properties
2. **System Identification**: Accurately modeling real robot dynamics
3. **Robust Control**: Developing controllers that handle model errors
4. **Progressive Domain Adaptation**: Gradually reducing simulation fidelity

## Key Concepts

- **URDF (Unified Robot Description Format)**: XML format for defining robot models
- **SDF (Simulation Description Format)**: XML format for describing simulation environments
- **Gazebo**: 3D simulation environment integrated with ROS2
- **Unity ML-Agents**: Toolkit for creating embodied AI agents in Unity
- **Domain Randomization**: Technique to improve sim-to-real transfer by randomizing simulation parameters
- **Sim-to-Real Transfer**: Process of applying skills learned in simulation to real robots

## Summary

This chapter covered the essential simulation environments for developing Physical AI systems. You learned about Gazebo's integration with ROS2 and Unity ML-Agents for advanced reinforcement learning applications. The choice of simulation environment depends on specific requirements for physics accuracy, visual fidelity, and integration with existing tools. Understanding both environments provides flexibility for different robotics tasks and helps bridge the sim-to-real gap.

## Exercises

1. Create a simple differential drive robot model in URDF and simulate it in Gazebo
2. Implement a basic navigation task using Gazebo simulation
3. Set up a Unity environment with ML-Agents and train a simple agent
4. Compare the performance of a control algorithm in both simulation environments