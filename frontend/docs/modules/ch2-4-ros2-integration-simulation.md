# Chapter 4: ROS 2 Integration with Simulation

## Clear Explanation

ROS 2 (Robot Operating System 2) integration with simulation platforms like Gazebo and Unity is critical for creating realistic and functional robot simulations. This integration allows simulation environments to communicate with real-world ROS 2 nodes, enabling the testing of complete robotic systems before deployment on physical hardware. The integration works by simulating sensors and actuators as ROS 2 nodes, publishing and subscribing to standard ROS 2 topics, services, and actions.

The ROS 2 simulation integration provides several benefits:
- Testing robotic applications in a safe environment
- Developing and debugging control algorithms without physical hardware
- Generating synthetic data for training AI models
- Validating system integration before deployment

## Subsections

### 4.1 ROS 2 with Gazebo Integration

The integration between ROS 2 and Gazebo is facilitated by the `gazebo_ros_pkgs` package, which provides plugins that connect Gazebo simulation to ROS 2 topics and services. Key components include:

- **gazebo_ros_control**: Provides hardware interface for ROS 2 controllers
- **gazebo_ros_diff_drive**: Simulates differential drive robots
- **gazebo_ros_harness**: Restrains models to a reference link
- **gazebo_ros_imu**: Simulates IMU sensors
- **gazebo_ros_camera**: Simulates cameras
- **gazebo_ros_laser**: Simulates laser scanners

### 4.2 Setting Up ROS 2 with Gazebo

**Installation:**
```bash
sudo apt update
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-gazebo-ros2-control
sudo apt install ros-humble-gazebo-dev
```

**Gazebo ROS Control Configuration:**
```yaml
# In your robot's config directory
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    velocity_controller:
      type: velocity_controllers/JointGroupVelocityController
```

**Robot URDF with Gazebo Plugins:**
```xml
<robot name="my_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- ROS Control plugin for Gazebo -->
  <gazebo>
    <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
      <parameters>$(find my_robot_description)/config/my_robot_controllers.yaml</parameters>
    </plugin>
  </gazebo>
  
  <!-- Differential drive plugin -->
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
      <wheel_separation>0.3</wheel_separation>
      <wheel_diameter>0.1</wheel_diameter>
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

### 4.3 Launch Files for Simulation

ROS 2 launch files simplify the process of starting simulation environments with all necessary nodes:

**Example launch file:**
```python
# launch/my_robot_gazebo.launch.py
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Launch configuration
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_name = LaunchConfiguration('world_name', default='small_room.world')
    
    # Get paths
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_my_robot_description = get_package_share_directory('my_robot_description')
    
    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        ),
        launch_arguments={
            'world': os.path.join(pkg_my_robot_description, 'worlds', world_name),
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
        arguments=[os.path.join(pkg_my_robot_description, 'urdf', 'my_robot.urdf')]
    )
    
    # Spawn robot in Gazebo
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
        DeclareLaunchArgument('world_name', default_value='small_room.world'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        gazebo,
        robot_state_publisher,
        spawn_entity
    ])
```

### 4.4 ROS 2 with Unity Integration

Unity integrates with ROS 2 through the ROS TCP Connector package, which establishes a communication bridge between Unity and ROS 2 systems. The integration allows Unity to publish and subscribe to ROS 2 topics, call services, and connect to actions.

**Unity ROS TCP Connector Setup:**
1. Install the ROS-TCP-Connector package via Unity Package Manager
2. Add the ROSConnection component to your scene
3. Configure the ROS settings in the Unity editor
4. Use the provided ROS message types in your scripts

### 4.5 Unity-ROS Integration Example

**Setting up ROS connection in Unity:**
```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Nav;

public class UnityROSDemo : MonoBehaviour
{
    ROSConnection ros;
    public string cmdVelTopic = "/cmd_vel";
    public string laserScanTopic = "/scan";
    
    // Robot parameters
    [Header("Robot Movement")]
    public float maxLinearSpeed = 1.0f;
    public float maxAngularSpeed = 1.0f;
    
    void Start()
    {
        // Get the ROS connection
        ros = ROSConnection.GetOrCreateInstance();
        
        // Connect to ROS
        ros.RegisterPublisher<TwistMsg>(cmdVelTopic);
        
        // Subscribe to laser scan
        ros.Subscribe<LaserScanMsg>(laserScanTopic, LaserScanCallback);
    }
    
    void Update()
    {
        // Get input and send to ROS
        float linear = Input.GetAxis("Vertical");  // W/S keys
        float angular = Input.GetAxis("Horizontal");  // A/D keys
        
        // Create and publish Twist message
        var twist = new TwistMsg();
        twist.linear = new Vector3Msg { x = linear * maxLinearSpeed };
        twist.angular = new Vector3Msg { z = angular * maxAngularSpeed };
        
        ros.Publish(cmdVelTopic, twist);
    }
    
    void LaserScanCallback(LaserScanMsg scan)
    {
        // Process laser scan data
        Debug.Log($"Received scan with {scan.ranges.Length} points");
        Debug.Log($"Min range: {scan.range_min}, Max range: {scan.range_max}");
    }
}
```

[DIAGRAM: ROS 2 Integration Architecture - Showing how ROS 2 nodes connect to both Gazebo and Unity simulations]

### 4.6 Performance Considerations

When integrating ROS 2 with simulation, several performance considerations must be addressed:

- **Update Rates**: Align simulation and ROS2 update rates to avoid bottlenecks
- **Message Frequency**: Control the frequency of published messages to prevent network congestion
- **Physics Parameters**: Configure physics engines to match real-time requirements
- **Resource Management**: Balance simulation quality with computational resources

## Example Snippets

### ROS 2 Node for Controlling Gazebo Robot
```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math

class GazeboController(Node):
    def __init__(self):
        super().__init__('gazebo_controller')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        # Robot state
        self.position = (0.0, 0.0)
        self.yaw = 0.0
        self.obstacle_detected = False
        
        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)
        
    def odom_callback(self, msg):
        self.position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )
        
        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)
        
    def scan_callback(self, msg):
        # Check for obstacles in front of robot
        front_scan = msg.ranges[len(msg.ranges) // 2]  # Middle range value
        self.obstacle_detected = front_scan < 1.0  # 1 meter threshold
        
    def control_loop(self):
        msg = Twist()
        
        if self.obstacle_detected:
            # Turn to avoid obstacle
            msg.linear.x = 0.0
            msg.angular.z = 0.5  # Turn right
        else:
            # Move forward
            msg.linear.x = 0.5
            msg.angular.z = 0.0
        
        self.cmd_vel_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    controller = GazeboController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Unity-ROS Bridge for Navigation
```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Nav;
using System.Collections.Generic;

public class UnityNavigation : MonoBehaviour
{
    ROSConnection ros;
    
    [Header("ROS Topics")]
    public string cmdVelTopic = "/cmd_vel";
    public string goalTopic = "/move_base_simple/goal";
    public string laserScanTopic = "/scan";
    
    [Header("Navigation Parameters")]
    public float linearSpeed = 1.0f;
    public float angularSpeed = 1.0f;
    public float obstacleThreshold = 1.0f;
    
    // Cached obstacle data
    private List<float> laserRanges = new List<float>();
    private bool obstacleDetected = false;
    
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        
        // Publishers and Subscribers
        ros.RegisterPublisher<PoseStampedMsg>(goalTopic);
        ros.Subscribe<LaserScanMsg>(laserScanTopic, OnLaserScan);
    }
    
    void OnLaserScan(LaserScanMsg scan)
    {
        laserRanges.Clear();
        laserRanges.AddRange(scan.ranges);
        
        // Check for obstacles in front (middle portion of scan)
        int midIndex = laserRanges.Count / 2;
        int window = laserRanges.Count / 10;  // Check 10% of ranges around center
        
        for (int i = midIndex - window; i < midIndex + window; i++)
        {
            if (i >= 0 && i < laserRanges.Count && 
                laserRanges[i] < obstacleThreshold && 
                !float.IsPositiveInfinity(laserRanges[i]))
            {
                obstacleDetected = true;
                return;
            }
        }
        
        obstacleDetected = false;
    }
    
    void Update()
    {
        // Simple navigation: avoid obstacles and move forward
        var twist = new TwistMsg();
        
        if (obstacleDetected)
        {
            // Turn to avoid obstacle
            twist.linear = new Vector3Msg { x = 0.0f };
            twist.angular = new Vector3Msg { z = angularSpeed };
        }
        else
        {
            // Move forward
            twist.linear = new Vector3Msg { x = linearSpeed };
            twist.angular = new Vector3Msg { z = 0.0f };
        }
        
        ros.Publish(cmdVelTopic, twist);
    }
    
    // Method to send navigation goal
    public void SendGoal(float x, float y, float theta)
    {
        var goal = new PoseStampedMsg();
        goal.header.frame_id = "map";
        goal.header.stamp = new builtin_interfaces.msg.Time();
        
        goal.pose.position.x = x;
        goal.pose.position.y = y;
        goal.pose.position.z = 0.0f;
        
        // Convert Euler angle to quaternion
        float cy = Mathf.Cos(theta * 0.5f);
        float sy = Mathf.Sin(theta * 0.5f);
        goal.pose.orientation.z = sy;
        goal.pose.orientation.w = cy;
        
        ros.Publish(goalTopic, goal);
    }
}
```

### TF Broadcasting in Unity
```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Tf2;
using System.Collections.Generic;

public class UnityTfBroadcaster : MonoBehaviour
{
    ROSConnection ros;
    public string tfTopic = "/tf";
    
    [Header("Robot Links")]
    public Transform baseLink;
    public Transform[] childLinks;  // e.g., wheels, sensors
    public string[] linkNames;      // corresponding names for TF tree
    
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<TFMessageMsg>(tfTopic);
        
        InvokeRepeating("BroadcastTransforms", 0.0f, 0.05f);  // 20 Hz
    }
    
    void BroadcastTransforms()
    {
        var tfMsg = new TFMessageMsg();
        
        // Create transforms for all robot links
        for (int i = 0; i < childLinks.Length; i++)
        {
            if (childLinks[i] != null && !string.IsNullOrEmpty(linkNames[i]))
            {
                var tf = CreateTransform(
                    childLinks[i], 
                    baseLink, 
                    "base_link", 
                    linkNames[i]
                );
                
                tfMsg.transforms.Add(tf);
            }
        }
        
        ros.Publish(tfTopic, tfMsg);
    }
    
    TFMessageMsg CreateTransform(
        Transform child, 
        Transform parent, 
        string parentFrameId, 
        string childFrameId)
    {
        var transformMsg = new TFMessageMsg();
        var transform = new geometry_msgs.msg.TransformStamped();
        
        transform.header.frame_id = parentFrameId;
        transform.header.stamp = new builtin_interfaces.msg.Time();
        
        // Calculate relative position and orientation
        var relativePos = parent.InverseTransformPoint(child.position);
        var relativeRot = Quaternion.Inverse(parent.rotation) * child.rotation;
        
        transform.child_frame_id = childFrameId;
        transform.transform.translation = new geometry_msgs.msg.Vector3Msg
        {
            x = relativePos.x,
            y = relativePos.y,
            z = relativePos.z
        };
        
        transform.transform.rotation = new geometry_msgs.msg.QuaternionMsg
        {
            x = relativeRot.x,
            y = relativeRot.y,
            z = relativeRot.z,
            w = relativeRot.w
        };
        
        transformMsg.transforms.Add(transform);
        return transformMsg;
    }
}
```

## Diagram Placeholders

[DIAGRAM: ROS 2 Integration Architecture - Detailed view of ROS 2 nodes connecting to both Gazebo and Unity simulations]

[DIAGRAM: TF Transform Tree - Showing the hierarchy of robot frames in simulation]

[DIAGRAM: Message Flow - From simulation sensors to ROS 2 processing nodes]

## Summary

This chapter covered the critical integration between ROS 2 and simulation environments, specifically Gazebo and Unity. You learned how to set up ROS 2 packages for Gazebo, configure robot models with appropriate plugins, create launch files for simulation, and integrate Unity with ROS 2 using the TCP connector. The next chapter will focus on hands-on labs and troubleshooting techniques to solidify your understanding of these simulation platforms.

## Exercises

1. Create a launch file that spawns a robot in Gazebo and starts the ROS2 controllers.
2. Set up a Unity scene that connects to ROS2 and publishes robot sensor data.
3. Implement a simple robot controller node that works with both Gazebo and Unity simulations.

## Further Reading

1. ROS2 with Gazebo documentation: https://github.com/ros-simulation/gazebo_ros_pkgs
2. Unity-ROS bridge tutorials: https://github.com/Unity-Technologies/ROS-TCP-Connector
3. "Robot Operating System 2 (ROS2): Concepts and Tools" - ROS2 documentation
4. "Simulation-based Testing for Robotic Systems" - academic survey

## Assessment Questions

1. Explain the role of gazebo_ros_pkgs in ROS2-Gazebo integration.
2. What are the key components of the Unity-ROS TCP connector?
3. How do you configure robot models in URDF for ROS2-Gazebo integration?
4. What are some performance considerations when integrating ROS2 with simulation?
5. Describe the process of setting up a TF tree for a simulated robot.