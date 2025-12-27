# Chapter 3: Unity Robotics Simulation

## Clear Explanation

Unity is a powerful game development engine that has been adapted for robotics simulation through its Unity Robotics package and ML-Agents toolkit. Unlike Gazebo's physics-focused approach, Unity emphasizes high-quality visualization and flexible environment creation, making it ideal for applications requiring photorealistic rendering, complex environments, or reinforcement learning with visual inputs.

Unity's strength lies in its visual editor, asset store, and graphics capabilities, which enable the creation of rich, immersive environments. The Unity Robotics package provides seamless integration with ROS2, allowing developers to control robots in Unity using standard ROS2 messages while taking advantage of Unity's visualization and environment creation tools.

## Subsections

### 3.1 Unity Robotics Package Overview

The Unity Robotics package provides:

- **ROS-TCP-Connector**: Communication layer between Unity and ROS2
- **Robotics Library**: Components and tools for robotics development
- **Sample Environments**: Pre-built scenarios for testing
- **ROS Message Types**: Unity implementations of common ROS2 messages
- **Editor Extensions**: Tools for easier robotics development

### 3.2 Setting Up Unity for Robotics

**Prerequisites:**
- Unity Hub and Unity Editor (2021.3 LTS or later recommended)
- Unity Robotics Package
- ROS2 (Humble Hawksbill or later)
- Python 3.8 or later

**Installation Steps:**

1. Create a new 3D project in Unity
2. Install the Unity Robotics package through Package Manager
3. Install the Unity ROS-TCP-Connector via Package Manager
4. Import the Robotics Library from the Unity Asset Store or GitHub

### 3.3 Creating Robot Models in Unity

In Unity, robot models are created as standard 3D objects with special robotic components:

**Basic Robot Setup:**
```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Geometry;

public class UnityRobotController : MonoBehaviour
{
    // ROS Connection
    private ROSConnection ros;
    
    // Robot components
    public GameObject[] wheels;
    public float wheelRadius = 0.1f;
    public float maxAngularVelocity = 10.0f;
    
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        // Subscribe to command topic
        ros.Subscribe<TwistMsg>("/cmd_vel", CmdVelCallback);
    }
    
    void CmdVelCallback(TwistMsg cmdVel)
    {
        // Convert linear/angular velocities to wheel velocities
        float linearVelocity = cmdVel.linear.x;
        float angularVelocity = cmdVel.angular.z;
        
        // Calculate individual wheel velocities
        float leftWheelVel = linearVelocity - (angularVelocity * 0.5f); // 0.5 is half axle length
        float rightWheelVel = linearVelocity + (angularVelocity * 0.5f);
        
        // Apply wheel rotation (simplified)
        foreach (GameObject wheel in wheels)
        {
            wheel.transform.Rotate(Vector3.right, leftWheelVel * Time.deltaTime);
        }
    }
}
```

### 3.4 Environment Creation in Unity

Unity's visual editor makes it easy to create complex and visually rich environments:

**Creating a Basic Environment:**
1. Create a ground plane for the robot to navigate on
2. Add static obstacles (cubes, spheres, etc.)
3. Create lighting with Directional Light
4. Use the Terrain tool for outdoor environments
5. Apply materials and textures for realism

**Example Environment Setup Script:**
```csharp
using UnityEngine;

public class RobotEnvironment : MonoBehaviour
{
    public Material floorMaterial;
    public Material obstacleMaterial;
    
    [System.Serializable]
    public struct ObstacleInfo
    {
        public Vector3 position;
        public Vector3 size;
        public GameObject obstaclePrefab;
    }
    
    public ObstacleInfo[] obstacles;
    
    void Start()
    {
        CreateEnvironment();
    }
    
    void CreateEnvironment()
    {
        // Create ground plane
        GameObject ground = GameObject.CreatePrimitive(PrimitiveType.Plane);
        ground.transform.position = Vector3.zero;
        ground.transform.localScale = new Vector3(10, 1, 10);
        ground.GetComponent<Renderer>().material = floorMaterial;
        
        // Create obstacles
        foreach (ObstacleInfo info in obstacles)
        {
            GameObject obstacle;
            if (info.obstaclePrefab != null)
            {
                obstacle = Instantiate(info.obstaclePrefab, info.position, Quaternion.identity);
            }
            else
            {
                obstacle = GameObject.CreatePrimitive(PrimitiveType.Cube);
                obstacle.transform.position = info.position;
                obstacle.transform.localScale = info.size;
                obstacle.GetComponent<Renderer>().material = obstacleMaterial;
            }
        }
    }
}
```

### 3.5 Unity ML-Agents Toolkit

The ML-Agents toolkit enables reinforcement learning in Unity environments:

**Key Components:**
- **Academy**: Manages training and simulation parameters
- **Agent**: Individual entity that learns to perform tasks
- **Brain**: Decision-making component (can be trained or scripted)
- **Environment Parameter**: Dynamic parameters for training

**Agent Implementation:**
```csharp
using Unity.ML-Agents;
using Unity.ML-Agents.Sensors;
using Unity.ML-Agents.Actuators;

public class UnityRobotAgent : Agent
{
    private Rigidbody rBody;
    public Transform target;
    public float moveForce = 100f;
    
    void Start()
    {
        rBody = GetComponent<Rigidbody>();
    }
    
    public override void OnEpisodeBegin()
    {
        // Reset agent position and target
        this.rBody.velocity = Vector3.zero;
        this.rBody.angularVelocity = Vector3.zero;
        
        // Move target to random position
        target.position = new Vector3(
            Random.Range(-5f, 5f),
            0.5f,
            Random.Range(-5f, 5f)
        );
    }
    
    public override void CollectObservations(VectorSensor sensor)
    {
        // Observe the relative position of the target
        sensor.AddObservation(target.position - this.transform.position);
        
        // Observe the agent's velocity
        sensor.AddObservation(rBody.velocity.x);
        sensor.AddObservation(rBody.velocity.z);
    }
    
    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        // Actions: [0] - X movement, [1] - Z movement
        Vector3 controlSignal = Vector3.zero;
        controlSignal.x = actionBuffers.ContinuousActions[0];
        controlSignal.z = actionBuffers.ContinuousActions[1];
        
        rBody.AddForce(controlSignal * moveForce);
        
        // Rewards
        float distanceToTarget = Vector3.Distance(this.transform.position, target.position);
        
        // Reached target
        if (distanceToTarget < 1.5f)
        {
            SetReward(1.0f);
            EndEpisode();
        }
        // Fell off platform
        else if (this.transform.position.y < -1.0f)
        {
            SetReward(-1.0f);
            EndEpisode();
        }
        // Small negative reward for each step to encourage efficiency
        else
        {
            SetReward(-0.01f);
        }
    }
    
    public override void Heuristic(in ActionBuffers actionsOut)
    {
        // Manual control for testing
        var continuousActionsOut = actionsOut.ContinuousActions;
        continuousActionsOut[0] = Input.GetAxis("Horizontal");
        continuousActionsOut[1] = Input.GetAxis("Vertical");
    }
}
```

[DIAGRAM: Unity ML-Agents Architecture - Showing Academy, Agents, and Training Process]

### 3.6 ROS2 Integration with Unity

The Unity-ROS2 bridge enables communication between Unity and ROS2 systems:

- **Message Types**: Unity implementations of ROS2 message types
- **Service Calls**: Support for ROS2 services
- **Action Servers**: Support for ROS2 actions
- **TF Broadcasting**: Transform frame publishing
- **Topic Publishing/Subscribing**: Standard ROS2 topics

## Example Snippets

### Unity Package Manager Installation
```bash
# In the Unity project directory:
# Add the Unity Robotics package through the Package Manager UI
# Or via manifest.json modification:
{
  "dependencies": {
    "com.unity.robotics.ros-tcp-connector": "https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector",
    "com.unity.robotics.urdf-importer": "https://github.com/Unity-Technologies/URDF-Importer.git?path=/com.unity.robotics.urdf-importer"
  }
}
```

### Basic ROS2 Publisher in Unity
```csharp
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Std;

public class UnityROSPublisher : MonoBehaviour
{
    private ROSConnection ros;
    public string topicName = "/unity_sensor_data";
    
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
    }
    
    void Update()
    {
        // Publish sensor data every second
        if (Time.time % 1.0f < Time.deltaTime)
        {
            var sensorData = new StringMsg
            {
                data = $"Unity sensor reading at: {Time.time}"
            };
            
            ros.Publish(topicName, sensorData);
        }
    }
}
```

### Connecting Unity to ROS2 Network
```python
# Python ROS2 node to communicate with Unity
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class UnityBridge(Node):
    def __init__(self):
        super().__init__('unity_bridge')
        
        # Publisher to send commands to Unity
        self.cmd_publisher = self.create_publisher(
            Twist, 
            '/cmd_vel', 
            10
        )
        
        # Subscriber to receive data from Unity
        self.data_subscriber = self.create_subscription(
            String,
            '/unity_sensor_data',
            self.sensor_callback,
            10
        )
        
        # Timer to send commands
        self.timer = self.create_timer(0.1, self.send_command)
        
    def sensor_callback(self, msg):
        self.get_logger().info(f'Received from Unity: {msg.data}')
    
    def send_command(self):
        msg = Twist()
        msg.linear.x = 0.5  # Move forward
        msg.angular.z = 0.2  # Turn slightly right
        self.cmd_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    bridge = UnityBridge()
    
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    
    bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Unity Raycasting for Sensor Simulation
```csharp
using UnityEngine;

public class UnityLidarSimulation : MonoBehaviour
{
    public int rayCount = 360;
    public float maxDistance = 10.0f;
    public LayerMask obstacleLayer;
    
    void Update()
    {
        if (Input.GetKeyDown(KeyCode.Space))
        {
            SimulateLidarScan();
        }
    }
    
    void SimulateLidarScan()
    {
        float angleStep = 360.0f / rayCount;
        float currentAngle = 0.0f;
        
        for (int i = 0; i < rayCount; i++)
        {
            float angleInRadians = currentAngle * Mathf.Deg2Rad;
            Vector3 direction = new Vector3(
                Mathf.Cos(angleInRadians),
                0,
                Mathf.Sin(angleInRadians)
            );
            
            RaycastHit hit;
            if (Physics.Raycast(transform.position, direction, out hit, maxDistance, obstacleLayer))
            {
                Debug.DrawRay(transform.position, direction * hit.distance, Color.red);
                // Process distance measurement
                float distance = hit.distance;
            }
            else
            {
                Debug.DrawRay(transform.position, direction * maxDistance, Color.green);
                // Maximum range measurement
                float distance = maxDistance;
            }
            
            currentAngle += angleStep;
        }
    }
}
```

## Diagram Placeholders

[DIAGRAM: Unity Robotics Architecture - Showing the Unity-ROS2 bridge and communication layers]

[DIAGRAM: ML-Agents Training Process - Showing the reinforcement learning loop in Unity]

[DIAGRAM: Unity vs Gazebo Comparison - When to use each platform]

## Summary

This chapter introduced Unity Robotics simulation, highlighting its visual capabilities and integration with ROS2. You've learned to set up Unity for robotics applications, create robot models, build rich environments, and use the ML-Agents toolkit for reinforcement learning. The next chapter will cover ROS2 integration with both simulation platforms, showing how to connect simulation to the broader ROS2 ecosystem.

## Exercises

1. Set up a Unity project with the Robotics package and create a simple robot model.
2. Implement a basic robot controller in Unity that responds to ROS2 Twist messages.
3. Create an environment in Unity with obstacles and use ML-Agents to train a simple navigation task.

## Further Reading

1. Unity Robotics documentation: https://unity.com/solutions/robotics
2. ML-Agents toolkit documentation: https://github.com/Unity-Technologies/ml-agents
3. "Unity as a Robotics Simulation Platform" - academic paper
4. "ROS-TCP-Connector for Unity" - technical documentation

## Assessment Questions

1. What are the main components of the Unity Robotics package?
2. How does the ROS-TCP-Connector enable communication between Unity and ROS2?
3. What are the key differences between Unity and Gazebo for robotics simulation?
4. Explain the concept of curriculum learning in the context of ML-Agents.
5. Describe the process of integrating Unity with the ROS2 ecosystem.