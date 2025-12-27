# Chapter 5: Hands-on Labs & Troubleshooting

## Clear Explanation

This chapter provides practical hands-on labs to reinforce the concepts covered in the previous chapters and offers troubleshooting techniques for common issues encountered when working with Gazebo and Unity robotics simulations. The labs are designed to give students practical experience with both simulation platforms and their integration with ROS2, while the troubleshooting section addresses real-world problems that arise during development and deployment.

The hands-on approach allows students to apply theoretical knowledge to practical scenarios, building confidence and expertise in using digital twin technologies for robotics development. The troubleshooting section provides a systematic approach to identifying and resolving common issues, which is crucial for effective robotics development.

## Subsections

### 5.1 Lab 1: Gazebo Robot Simulation Setup

**Objective**: Create a complete simulation environment with a differential drive robot in Gazebo.

**Requirements**:
- ROS2 Humble Hawksbill
- Gazebo Garden
- Python 3.8+
- Basic knowledge of URDF

**Step-by-step Instructions**:

1. **Create a new ROS2 package for the robot**:
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake my_gazebo_robot --dependencies rclcpp geometry_msgs std_msgs sensor_msgs gazebo_ros_pkgs
```

2. **Create the robot URDF file** (`my_gazebo_robot/urdf/my_robot.urdf`):
```xml
<?xml version="1.0"?>
<robot name="my_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0.0 0.0 1.0 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.3 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5"/>
      <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
    </inertial>
  </link>

  <!-- Left wheel -->
  <link name="wheel_left">
    <visual>
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>
      <material name="black">
        <color rgba="0.0 0.0 0.0 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Right wheel -->
  <link name="wheel_right">
    <visual>
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>
      <material name="black">
        <color rgba="0.0 0.0 0.0 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Joints -->
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_left"/>
    <origin xyz="-0.15 -0.2 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_right"/>
    <origin xyz="-0.15 0.2 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <!-- Gazebo plugins -->
  <gazebo>
    <plugin name="differential_drive" filename="libgazebo_ros_diff_drive.so">
      <ros>
        <namespace>/my_robot</namespace>
        <remapping>cmd_vel:=cmd_vel</remapping>
        <remapping>odom:=odom</remapping>
      </ros>
      <update_rate>30</update_rate>
      <left_joint>left_wheel_joint</left_joint>
      <right_joint>right_wheel_joint</right_joint>
      <wheel_separation>0.4</wheel_separation>
      <wheel_diameter>0.2</wheel_diameter>
      <max_wheel_torque>20</max_wheel_torque>
      <max_wheel_acceleration>1.0</max_wheel_acceleration>
      <publish_odom>true</publish_odom>
      <publish_odom_tf>true</publish_odom_tf>
      <publish_wheel_tf>true</publish_wheel_tf>
      <odometry_frame>odom</odometry_frame>
      <robot_base_frame>base_link</robot_base_frame>
    </plugin>
  </gazebo>
</robot>
```

3. **Create a launch file** (`my_gazebo_robot/launch/robot_sim.launch.py`):
```python
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Launch configuration
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_name = LaunchConfiguration('world_name', default='empty.world')
    
    # Package directories
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_my_robot = get_package_share_directory('my_gazebo_robot')
    
    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        ),
        launch_arguments={
            'world': os.path.join(pkg_my_robot, 'worlds', world_name),
            'gui': 'true'
        }.items()
    )
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[os.path.join(pkg_my_robot, 'urdf', 'my_robot.urdf')]
    )
    
    # Spawn robot
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'my_robot',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1'
        ],
        output='screen'
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('world_name', default_value='empty.world'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        gazebo,
        robot_state_publisher,
        spawn_entity
    ])
```

4. **Create a simple controller** (`my_gazebo_robot/src/controller.cpp`):
```cpp
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

class RobotController : public rclcpp::Node
{
public:
    RobotController() : Node("robot_controller")
    {
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/my_robot/cmd_vel", 10);
        
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), 
            std::bind(&RobotController::control_cycle, this));
    }

private:
    void control_cycle()
    {
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x = 0.5;  // Move forward at 0.5 m/s
        msg.angular.z = 0.2; // Turn right at 0.2 rad/s
        
        cmd_vel_publisher_->publish(msg);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotController>());
    rclcpp::shutdown();
    return 0;
}
```

5. **Build and run the simulation**:
```bash
cd ~/ros2_ws
colcon build --packages-select my_gazebo_robot
source install/setup.bash
ros2 launch my_gazebo_robot robot_sim.launch.py
```

### 5.2 Lab 2: Unity Robot Simulation with ML-Agents

**Objective**: Set up a Unity environment with ML-Agents to train a robot to navigate to a target.

**Requirements**:
- Unity 2021.3 LTS or later
- Unity ML-Agents Toolkit
- Python 3.8+
- ML-Agents Python package

**Step-by-step Instructions**:

1. **Install ML-Agents in Unity**:
   - Open Package Manager in Unity
   - Install ML-Agents package
   - Install Python package: `pip install mlagents`

2. **Create a basic environment**:
   - Create a plane as the ground
   - Add a 3D cube as the robot
   - Add a sphere as the target
   - Create a simple navigation script

3. **Implement the Unity robot agent** (`UnityRobotAgent.cs`):
```csharp
using Unity.ML-Agents;
using Unity.ML-Agents.Sensors;
using Unity.ML-Agents.Actuators;
using UnityEngine;

public class UnityRobotAgent : Agent
{
    [Header("Robot Movement")]
    public float moveForce = 100f;
    public float rotationSpeed = 100f;
    
    [Header("Target")]
    public Transform target;
    public float targetReachedDistance = 1.5f;

    private Rigidbody rBody;
    private Vector3 initialPosition;
    private Quaternion initialRotation;
    
    void Start()
    {
        rBody = GetComponent<Rigidbody>();
        initialPosition = transform.position;
        initialRotation = transform.rotation;
    }
    
    public override void OnEpisodeBegin()
    {
        // Reset agent position
        transform.position = initialPosition;
        transform.rotation = initialRotation;
        rBody.velocity = Vector3.zero;
        rBody.angularVelocity = Vector3.zero;
        
        // Move target to random position
        target.position = new Vector3(
            Random.Range(-5f, 5f),
            0.5f,
            Random.Range(-5f, 5f)
        );
    }
    
    public override void CollectObservations(VectorSensor sensor)
    {
        // Observe position relative to target
        sensor.AddObservation(target.position - transform.position);
        
        // Observe agent velocity
        sensor.AddObservation(rBody.velocity);
        
        // Observe agent orientation
        sensor.AddObservation(transform.rotation);
    }
    
    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        // Extract actions
        float forwardMovement = actionBuffers.ContinuousActions[0];
        float rotationMovement = actionBuffers.ContinuousActions[1];
        
        // Apply forces to move the agent
        Vector3 forwardForce = transform.forward * forwardMovement * moveForce * Time.fixedDeltaTime;
        rBody.AddForce(forwardForce);
        
        // Apply torque for rotation
        float rotationTorque = rotationMovement * rotationSpeed * Time.fixedDeltaTime;
        rBody.AddTorque(Vector3.up * rotationTorque);
        
        // Calculate distance to target
        float distanceToTarget = Vector3.Distance(transform.position, target.position);
        
        // Provide rewards
        if (distanceToTarget < targetReachedDistance)
        {
            // Positive reward for reaching target
            SetReward(1.0f);
            EndEpisode();
        }
        else if (transform.position.y < -1.0f)
        {
            // Negative reward for falling off
            SetReward(-1.0f);
            EndEpisode();
        }
        else
        {
            // Small negative reward to encourage efficiency
            SetReward(-0.01f);
        }
    }
    
    public override void Heuristic(in ActionBuffers actionsOut)
    {
        // Manual control for testing
        var continuousActionsOut = actionsOut.ContinuousActions;
        continuousActionsOut[0] = Input.GetAxis("Vertical"); // Forward/back
        continuousActionsOut[1] = Input.GetAxis("Horizontal"); // Left/right turn
    }
}
```

4. **Training Configuration** (`trainer_config.yaml`):
```yaml
default_training:
  trainer_type: ppo
  hyperparameters:
    batch_size: 128
    buffer_size: 2048
    learning_rate: 0.0003
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

5. **Train the agent**:
```bash
mlagents-learn trainer_config.yaml --run-id=my_robot_navigation
```

### 5.3 Common Gazebo Issues and Solutions

**Problem 1: Robot falls through the ground**
- **Cause**: Incorrect inertial properties or collision geometry
- **Solution**: Check that the `inertial` tag has appropriate mass and inertia values; ensure collision geometry matches visual geometry

**Problem 2: Robot doesn't respond to commands**
- **Cause**: Missing or incorrect Gazebo plugins
- **Solution**: Verify that the differential drive plugin is properly configured with correct joint names and parameters

**Problem 3: Simulation runs too slow**
- **Cause**: High physics update rate or complex models
- **Solution**: Adjust the `max_step_size` in the world file or reduce model complexity

**Problem 4: Sensor data is incorrect**
- **Cause**: Sensor plugin not properly configured
- **Solution**: Check sensor parameters and verify sensor topics are being published

### 5.4 Common Unity Issues and Solutions

**Problem 1: Robot doesn't move in Unity**
- **Cause**: ROS connection not established or incorrect topic names
- **Solution**: Verify ROS TCP Connector settings and check topic names in Unity and ROS2

**Problem 2: ML-Agents training is extremely slow**
- **Cause**: High-resolution rendering or complex environments
- **Solution**: Disable rendering with `--no-graphics` during training or reduce environment complexity

**Problem 3: Unity crashes during training**
- **Cause**: Memory leaks or inefficient code
- **Solution**: Review code for proper resource management and consider limiting concurrent environments

### 5.5 Performance Optimization Techniques

**For Gazebo:**
- Reduce physics update rate if precision allows
- Use simpler collision meshes
- Limit the number of active sensors
- Use plugins only when necessary

**For Unity:**
- Use occlusion culling for large environments
- Optimize lighting and shadows
- Use lower-resolution textures if rendering quality permits
- Pool objects instead of instantiating/destructing frequently

[DIAGRAM: Troubleshooting Flowchart - Decision tree for common simulation issues]

## Example Snippets

### Gazebo Debugging Commands
```bash
# List all topics published by Gazebo
gz topic -l

# Echo a specific topic in Gazebo
gz topic -e -t /world/default/model/robot_name/joint_state

# List all models in the simulation
gz model -m

# Check the physics engine status
gz physics -i
```

### Unity Performance Monitor Script
```csharp
using UnityEngine;
using System.Collections.Generic;

public class PerformanceMonitor : MonoBehaviour
{
    [Header("Performance Settings")]
    public float updateInterval = 0.5f;
    
    private float accum = 0;
    private int frames = 0;
    private float timeleft;
    private float fps;
    
    private List<float> frameTimes = new List<float>();
    private const int frameTimeSamples = 60;
    
    void Start()
    {
        timeleft = updateInterval;
    }
    
    void Update()
    {
        float currentFrameTime = Time.unscaledDeltaTime;
        accum += currentFrameTime;
        frames++;
        
        // Add to frame time history
        frameTimes.Add(currentFrameTime);
        if (frameTimes.Count > frameTimeSamples)
            frameTimes.RemoveAt(0);
        
        timeleft -= Time.unscaledDeltaTime;
        
        if (timeleft <= 0.0f)
        {
            fps = frames / accum;
            accum = 0.0f;
            timeleft = updateInterval;
        }
        
        // Log performance metrics when needed
        if (accum % 10.0f < Time.unscaledDeltaTime)  // Every 10 seconds
        {
            Debug.Log($"FPS: {fps:F1}, Avg Frame Time: {GetAverageFrameTime():F3}s");
        }
    }
    
    float GetAverageFrameTime()
    {
        if (frameTimes.Count == 0) return 0;
        
        float sum = 0;
        foreach (float t in frameTimes)
            sum += t;
        
        return sum / frameTimes.Count;
    }
    
    void OnGUI()
    {
        int w = Screen.width, h = Screen.height;
        
        GUIStyle style = new GUIStyle();
        Rect rect = new Rect(0, 0, w, h * 2 / 100);
        style.alignment = TextAnchor.UpperLeft;
        style.fontSize = h * 2 / 100;
        style.normal.textColor = new Color(0.0f, 0.0f, 0.5f, 1.0f);
        string text = $"{fps:F1} FPS";
        GUI.Label(rect, text, style);
    }
}
```

### ROS2 Bridge Debugging Script
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math

class SimulationDebugger(Node):
    def __init__(self):
        super().__init__('simulation_debugger')
        
        # Subscribers for simulation data
        self.odom_sub = self.create_subscription(
            Odometry,
            '/my_robot/odom',
            self.odom_callback,
            10
        )
        
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/my_robot/scan',
            self.scan_callback,
            10
        )
        
        # Publisher for direct robot control
        self.cmd_pub = self.create_publisher(
            Twist,
            '/my_robot/cmd_vel',
            10
        )
        
        # Parameters
        self.position_threshold = 0.05  # meters
        self.last_position = (0.0, 0.0)
        self.no_movement_count = 0
        
        # Timer for regular checks
        self.timer = self.create_timer(2.0, self.periodic_check)
        
    def odom_callback(self, msg):
        current_pos = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )
        
        # Check if robot is moving
        distance_moved = math.sqrt(
            (current_pos[0] - self.last_position[0])**2 +
            (current_pos[1] - self.last_position[1])**2
        )
        
        if distance_moved < self.position_threshold:
            self.no_movement_count += 1
        else:
            self.no_movement_count = 0
        
        self.last_position = current_pos
        
        # If robot hasn't moved for a while, report it
        if self.no_movement_count > 5:  # 10 seconds of no movement
            self.get_logger().warn(
                f'Robot appears stuck at position ({current_pos[0]:.2f}, {current_pos[1]:.2f})'
            )
    
    def scan_callback(self, msg):
        # Check for potential collision issues
        min_range = min(msg.ranges)
        
        if min_range < 0.3:  # Less than 30cm to obstacle
            self.get_logger().info(f'Obstacle detected at {min_range:.2f}m')
    
    def periodic_check(self):
        self.get_logger().info('Simulation status check')
        # Add more checks here as needed
    
    def send_command(self, linear_x=0.0, angular_z=0.0):
        """Send direct velocity command to robot"""
        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.angular.z = angular_z
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    debugger = SimulationDebugger()
    
    try:
        rclpy.spin(debugger)
    except KeyboardInterrupt:
        debugger.get_logger().info('Shutting down simulation debugger')
    
    debugger.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Diagram Placeholders

[DIAGRAM: Troubleshooting Flowchart - Step-by-step decision tree for diagnosing simulation issues]

[DIAGRAM: Performance Comparison - Gazebo vs Unity performance characteristics]

[DIAGRAM: Integration Testing - Verifying the complete simulation pipeline]

## Summary

This chapter provided hands-on labs for both Gazebo and Unity simulation platforms, with practical examples for setting up robot models and implementing basic behaviors. We also covered common troubleshooting techniques for resolving issues in both environments and optimization strategies to ensure efficient simulation performance.

The labs demonstrated the complete workflow from creating robot models to implementing controllers and integrating with ROS2. The troubleshooting section provided systematic approaches to diagnosing and resolving common issues, which are essential skills for effective robotics development.

With the completion of this module, you now have a comprehensive understanding of digital twin technologies in robotics, with practical skills in both Gazebo and Unity simulation environments and their integration with the ROS2 ecosystem.

## Exercises

1. Implement the Gazebo robot simulation lab from this chapter in your local environment.
2. Create and train a simple ML-Agent in Unity for a navigation task.
3. Set up a complete simulation pipeline with perception, planning, and control components.

## Further Reading

1. "Simulation-Based Testing for Safety Validation of Autonomous Robots" - research paper
2. Gazebo troubleshooting guide: http://gazebosim.org/tutorials?tut=troubleshooting
3. Unity performance optimization for simulation: Unity documentation
4. "Best Practices for Robot Simulation" - industry guidelines

## Assessment Questions

1. What are the most common issues encountered in Gazebo robot simulation?
2. How would you troubleshoot a robot that is not responding to ROS2 commands in simulation?
3. What are the performance optimization techniques for Unity robotics simulation?
4. Explain the steps to debug a ROS-Unity connection issue.
5. What is domain randomization and how does it help in simulation-to-reality transfer?