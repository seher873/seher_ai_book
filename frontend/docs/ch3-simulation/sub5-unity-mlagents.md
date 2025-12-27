---
sidebar_position: 7
---

# 3.5 Unity ML-Agents Toolkit for Robotics

## Chapter 3: Simulation Environments - Gazebo and Unity

Unity ML-Agents Toolkit (ML-Agents) is a powerful platform that enables the use of Unity as a simulation environment for developing AI agents. It's particularly useful for tasks requiring high-fidelity visual rendering, complex environments, and reinforcement learning applications.

## Introduction to Unity ML-Agents

ML-Agents is an open-source Unity plugin that enables:

- **Reinforcement Learning**: Training intelligent agents using various RL algorithms
- **Imitation Learning**: Learning behaviors from expert demonstrations
- **High-Fidelity Simulation**: Realistic visual and physics simulation
- **Robotics Applications**: Specialized tools for robotics and embodied AI

### Key Features of ML-Agents

- **Multiple Training Algorithms**: Support for PPO, SAC, BC, and more
- **Flexible Observation Space**: Sensors to capture environment state
- **Action Space Configuration**: Continuous and discrete action spaces
- **Curriculum Learning**: Progressive difficulty training
- **Multi-Agent Support**: Training multiple agents simultaneously

## Setting Up Unity ML-Agents

### Prerequisites

- Unity 2022.3 LTS or newer
- Python 3.10+ with pip
- Git installed on your system

### Installation Steps

1. **Install Unity ML-Agents package**:
   - Open Unity Hub and create a new 3D project
   - Go to Window → Package Manager
   - Install "ML-Agents Toolkit" from the Package Manager
   - Or install via Git URL: `https://github.com/Unity-Technologies/ml-agents.git?path=/com.unity.ml-agents`

2. **Install Python packages**:
   ```bash
   pip install mlagents
   ```

## Understanding ML-Agents Architecture

ML-Agents connects Unity environments to Python for training through a communication bridge:

- **Unity Environment**: The simulation world running in Unity
- **Python Communication**: Bridge for sending observations and receiving actions
- **Training Framework**: Python-based RL algorithms (using TensorFlow)
- **Trained Model**: ONNX format model that can run in Unity

### Core Components

1. **Agent**: The entity that learns to interact with the environment
2. **Academy**: Manages the simulation time and environment parameters
3. **Environment Parameter**: Allows dynamic changes to environment properties
4. **Decision Requester**: Requests actions from external decision makers

## Creating a Basic ML-Agents Robot

Let's create a simple robot agent in Unity ML-Agents:

### 1. Setting Up the Agent Script

```csharp
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using UnityEngine;

public class RobotAgent : Agent
{
    [Header("Robot-specific Parameters")]
    public Transform target;
    public float speed = 10f;

    // Reference to the robot's Rigidbody
    private Rigidbody rBody;
    
    void Start()
    {
        rBody = GetComponent<Rigidbody>();
    }

    public override void OnEpisodeBegin()
    {
        // Reset robot and target positions
        this.rBody.velocity = Vector3.zero;
        this.rBody.angularVelocity = Vector3.zero;
        
        // Randomize positions
        transform.position = new Vector3(Random.Range(-5f, 5f), 0.5f, Random.Range(-5f, 5f));
        target.position = new Vector3(Random.Range(-4f, 4f), 0.5f, Random.Range(-4f, 4f));
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // Add observations: robot position and target position
        sensor.AddObservation(transform.position);
        sensor.AddObservation(target.position);
        
        // Add velocity of the robot
        sensor.AddObservation(rBody.velocity.x);
        sensor.AddObservation(rBody.velocity.z);
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        // Get action values (assuming continuous actions: x and z movement)
        float moveX = actions.ContinuousActions[0];
        float moveZ = actions.ContinuousActions[1];
        
        // Apply movement
        Vector3 controlSignal = Vector3.zero;
        controlSignal.x = moveX;
        controlSignal.z = moveZ;
        
        rBody.AddForce(controlSignal * speed);
        
        // Reward function: closer to target = higher reward
        float distanceToTarget = Vector3.Distance(transform.position, target.position);
        
        // Give a small time penalty to encourage reaching the target faster
        SetReward(-distanceToTarget * 0.01f);
        
        // Give bonus reward for reaching target
        if (distanceToTarget < 1.0f)
        {
            SetReward(2.0f);
            EndEpisode();
        }
        
        // End episode if robot moves too far away
        if (distanceToTarget > 20f)
        {
            SetReward(-1.0f);
            EndEpisode();
        }
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        // Manual control for testing (WASD keys)
        var continuousActionsOut = actionsOut.ContinuousActions;
        continuousActionsOut[0] = Input.GetAxis("Horizontal");
        continuousActionsOut[1] = Input.GetAxis("Vertical");
    }
}
```

### 2. Setting up the Unity Environment

1. **Create a 3D scene** with a plane as the ground
2. **Add the robot object** (e.g., a capsule) with:
   - Rigidbody component
   - Collider component
   - RobotAgent script attached
3. **Add a target object** (e.g., a sphere) to serve as the goal
4. **Add an Academy** in the scene to manage the simulation

### 3. Academy Setup

Create an Academy script to manage the environment:

```csharp
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;

public class RobotAcademy : Academy
{
    public override void OnEnvironmentReset()
    {
        // Called when the environment resets
    }
}
```

Attach this script to an empty GameObject in your scene and set it as the Academy in the ML-Agents configuration.

## ML-Agents Configuration

The training configuration is specified in a YAML file (e.g., `config/ppo/robot_config.yaml`):

```yaml
behaviors:
  RobotAgent:
    trainer_type: ppo
    hyperparameters:
      batch_size: 1024
      buffer_size: 4096
      learning_rate: 3.0e-4
      beta: 5.0e-3
      epsilon: 0.2
      lambd: 0.95
      num_epoch: 3
      learning_rate_schedule: linear
    network_settings:
      normalize: false
      hidden_units: 256
      num_layers: 2
    max_steps: 500000
    time_horizon: 64
    summary_freq: 10000
```

## Training the Agent

To train the agent:

```bash
# Train the agent
mlagents-learn config/ppo/robot_config.yaml --run-id=robot_run01

# Visualize training progress
tensorboard --logdir=summaries
```

## Advanced ML-Agents for Robotics

### Sensor Integration

ML-Agents supports various sensors that can be used to provide observations:

```csharp
public override void CollectObservations(VectorSensor sensor)
{
    // Ray Perceptions - simulate LiDAR
    AddVectorObs(Physics.RaycastAll(transform.position, transform.forward, 10f));
    
    // Camera Sensor - for visual input
    AddVectorObs(target.position - transform.position);
    
    // Vector Observations - for joint states, velocities, etc.
    AddVectorObs(rBody.velocity);
    AddVectorObs(transform.rotation);
}
```

### Multi-Agent Environments

For robotics applications with multiple robots:

```csharp
public class MultiRobotAcademy : Academy
{
    public List<RobotAgent> robotAgents;
    
    public override void OnEnvironmentReset()
    {
        // Reset all agents
        foreach (var agent in robotAgents)
        {
            agent.OnEpisodeBegin();
        }
    }
}
```

### Robotics-Specific Training Approaches

#### 1. Imitation Learning
Train from expert demonstrations:

```csharp
// Record expert behaviors
public void RecordExpertTrajectory()
{
    // Manual control of robot and record state-action pairs
}
```

#### 2. Curriculum Learning
Progressively increase difficulty:

```csharp
// Adjust environment parameters based on training progress
public override void OnEnvironmentStep()
{
    if (GetCumulativeReward() > threshold)
    {
        // Increase difficulty
        targetMoveSpeed *= 1.1f;
    }
}
```

## Unity vs Gazebo for Robotics

### Unity ML-Agents Advantages

- **Visual Quality**: High-fidelity rendering for computer vision tasks
- **Game Engine Features**: Advanced graphics, lighting, and environment rendering
- **Reinforcement Learning Integration**: Built-in support for RL algorithms
- **Flexibility**: Can create any 3D environment imaginable
- **Cross-Platform**: Deploy to multiple platforms

### Unity ML-Agents Disadvantages

- **Robotics Integration**: Less native ROS integration compared to Gazebo
- **Physics Fidelity**: Less accurate for certain robotic applications
- **Community**: Smaller robotics-specific community
- **Sensor Simulation**: Less realistic sensor simulation out-of-the-box

### When to Choose Unity ML-Agents

Unity ML-Agents is ideal when:

- Visual perception is a key component
- You need complex, high-fidelity environments
- Reinforcement learning is the primary approach
- You need deployment to multiple platforms
- Visual quality is important for your application

## Best Practices for ML-Agents Robotics

### 1. Environment Design

- **Simplify Early**: Start with simple environments and gradually increase complexity
- **Balance Fidelity**: Match simulation fidelity to your problem requirements
- **Diverse Scenarios**: Include multiple environment configurations to improve generalization

### 2. Reward Engineering

- **Sparse vs Dense Rewards**: Balance sparse rewards (end goal) with dense rewards (progress)
- **Avoid Reward Hacking**: Design rewards that align with the actual desired behavior
- **Progressive Rewards**: Provide intermediate rewards to guide learning

### 3. Performance Optimization

- **Batch Training**: Use vectorized environments for faster training
- **Curriculum Learning**: Gradually increase environment difficulty
- **Hyperparameter Tuning**: Systematically tune hyperparameters for best results

## Deploying Trained Models

Trained models can be deployed back into Unity for inference:

```csharp
// Using the trained model in Unity
public class RobotInference : Agent
{
    public BehaviorParameters behaviorParams;
    
    public override void OnActionReceived(ActionBuffers actions)
    {
        // The behavior will be executed based on the trained model
        // instead of manual computation
    }
}
```

## Summary

Unity ML-Agents provides a powerful platform for developing and training embodied AI systems with high-fidelity visual rendering. It's particularly well-suited for applications involving computer vision, reinforcement learning, and complex environments. When combined with Gazebo for physics-accurate simulations and ROS integration, ML-Agents offers a comprehensive simulation toolkit for Physical AI systems.

## Exercises

1. Create a simple Unity ML-Agents environment with a robot navigating to a goal
2. Train the robot using PPO algorithm and analyze the training progress
3. Implement a more complex task such as obstacle avoidance or manipulation
4. Compare training results between Unity and Gazebo for the same task