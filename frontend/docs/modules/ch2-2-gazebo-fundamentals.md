# Chapter 2: Gazebo Fundamentals

## Clear Explanation

Gazebo is a physics-based simulation environment that provides realistic sensor simulation and robot modeling capabilities. It's widely used in robotics research and development due to its accurate physics engines, extensive sensor models, and strong integration with ROS/ROS2. Gazebo enables the creation of complex robotic scenarios with realistic interactions between robots, objects, and environments.

The core of Gazebo consists of physics engines that accurately model rigid body dynamics, collisions, and contact forces. These engines work together with sensor models to simulate the robot's perception of its environment, making Gazebo ideal for testing perception algorithms, navigation strategies, and robot behaviors before deployment to real hardware.

## Subsections

### 2.1 Gazebo Architecture and Components

Gazebo's architecture consists of several key components:

- **Server (gazebo)**: The main simulation engine that handles physics simulation, rendering, and sensor updates
- **Client (gzclient)**: The user interface for visualizing and interacting with the simulation
- **Gazebo Model Database**: A repository of pre-built robot and environment models
- **Plugin Interface**: Extensible architecture for custom functionality
- **Transport Layer**: Communication system for data exchange between components

### 2.2 Installing and Setting Up Gazebo

To get started with Gazebo, follow these installation steps:

**Ubuntu Installation:**
```bash
sudo apt update
sudo apt install gazebo libgazebo-dev
```

**For ROS2 Integration:**
```bash
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-gazebo-ros2-control
sudo apt install ros-humble-gazebo-dev
```

### 2.3 Creating Robot Models for Gazebo

Robot models in Gazebo are defined using URDF (Unified Robot Description Format) or SDF (Simulation Description Format). URDF is commonly used with ROS and then converted to SDF for Gazebo simulation.

**Basic URDF Robot Model:**
```xml
<?xml version="1.0" ?>
<robot name="turtlebot3_burger">
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0.0 0.0 1.0 1.0"/>
      </material>
    </visual>
    
    <collision>
      <geometry>
        <cylinder length="0.1" radius="0.1"/>
      </geometry>
    </collision>
    
    <inertial>
      <mass value="1.0"/>
      <inertia 
        ixx="0.01" ixy="0.0" ixz="0.0" 
        iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="wheel_left_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_left_link"/>
    <origin xyz="0.0 0.1 0.0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <link name="wheel_left_link">
    <visual>
      <geometry>
        <cylinder length="0.02" radius="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0.0 0.0 0.0 1.0"/>
      </material>
    </visual>
  </link>
</robot>
```

### 2.4 Physics Engines in Gazebo

Gazebo supports multiple physics engines, each with different characteristics:

- **ODE (Open Dynamics Engine)**: Default engine, good for most applications
- **Bullet**: Robust collision detection, good for complex scenarios
- **Simbody**: Advanced multibody dynamics
- **DART**: Advanced contact handling and human biomechanics

The physics engine can be specified in the world file:
```xml
<physics name="default_physics" type="ode">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>
  <real_time_update_rate>1000.0</real_time_update_rate>
</physics>
```

### 2.5 Sensor Simulation

Gazebo provides plugins to simulate various sensors including:

- **Cameras**: RGB, depth, stereo cameras
- **LiDAR**: 2D and 3D laser scanners
- **IMU**: Inertial measurement units
- **GPS**: Global positioning systems
- **Force/Torque sensors**: For contact measurements

**Example camera sensor:**
```xml
<sensor name="camera" type="camera">
  <camera name="head">
    <horizontal_fov>1.047</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.1</near>
      <far>100</far>
    </clip>
  </camera>
  <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
    <frame_name>camera_frame</frame_name>
  </plugin>
</sensor>
```

[DIAGRAM: Gazebo Architecture - Showing server, client, physics engine, sensors, and plugins]

### 2.6 Gazebo Plugins

Gazebo's plugin architecture allows for custom behavior:

- **Model plugins**: Attach custom behavior to specific models
- **Sensor plugins**: Process sensor data or create custom sensors
- **World plugins**: Control world-level behavior
- **GUI plugins**: Extend the graphical interface

## Example Snippets

### Launching Gazebo with Custom World
```bash
# Launch Gazebo with a specific world file
gazebo my_world.world

# Launch with GUI disabled (server-only mode)
gzserver my_world.world

# Launch with custom physics parameters
gazebo -u my_world.world
```

### Creating a ROS2 Interface for Gazebo Robot
```xml
<!-- In your URDF file -->
<xacro:macro name="gazebo_ros_control" params="prefix">
  <gazebo>
    <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
      <robotNamespace>/$(arg prefix)</robotNamespace>
      <robotSimType>gazebo_ros_control/DefaultRobotHWSim</robotSimType>
    </plugin>
  </gazebo>
</xacro:macro>
```

### Controlling a Robot in Gazebo with ROS2
```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class GazeboRobotController(Node):
    def __init__(self):
        super().__init__('gazebo_robot_controller')
        
        # Create publisher for cmd_vel
        self.cmd_vel_pub = self.create_publisher(
            Twist, 
            '/cmd_vel', 
            10
        )
        
        # Create timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)
        
    def control_loop(self):
        msg = Twist()
        
        # Simple movement command
        msg.linear.x = 0.5  # Move forward at 0.5 m/s
        msg.angular.z = 0.2  # Turn right at 0.2 rad/s
        
        self.cmd_vel_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    controller = GazeboRobotController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Gazebo Model Plugin Example
```cpp
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

namespace gazebo
{
  class CustomModelPlugin : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      // Store the model pointer for later use
      this->model = _model;
      
      // Get parameters from SDF
      if (_sdf->HasElement("custom_param"))
        this->customParam = _sdf->Get<double>("custom_param");
      
      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&CustomModelPlugin::OnUpdate, this));
    }

    // Called by the world update start event
    public: void OnUpdate()
    {
      // Apply a small linear velocity to the model
      this->model->SetLinearVel(ignition::math::Vector3d(0.1, 0, 0));
    }

    // Pointer to the model
    private: physics::ModelPtr model;
    
    // Custom parameter
    private: double customParam;
    
    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(CustomModelPlugin)
}
```

## Diagram Placeholders

[DIAGRAM: Gazebo Architecture - Detailed view of server, client, physics engine, sensors, and plugins]

[DIAGRAM: URDF Model Structure - Showing links, joints, and visual/collision components]

[DIAGRAM: Robot Control Loop - From ROS2 commands to Gazebo simulation]

## Summary

This chapter provided a comprehensive introduction to Gazebo fundamentals, covering its architecture, installation, model creation, physics engines, and sensor simulation. You've learned how to create basic robot models with URDF, configure physics parameters, and set up ROS2 integration. The next chapter will explore Unity robotics simulation, comparing it with Gazebo's approach and showcasing its unique capabilities, particularly in visualization and ML-Agents integration.

## Exercises

1. Create a simple URDF model of a differential drive robot with two wheels and a chassis.
2. Set up a basic Gazebo world with your robot model and add a camera sensor to it.
3. Implement a simple ROS2 node to control the robot in simulation using Twist messages.

## Further Reading

1. Gazebo Classic documentation: http://gazebosim.org/documentation/
2. ROS2 with Gazebo tutorials: https://navigation.ros.org/tutorials/docs/get_back_to_real_world.html
3. "URDF/XML Robot Description Format" - official ROS documentation
4. "Physics-Based Simulation for Robotics" - academic course materials

## Assessment Questions

1. What are the main components of the Gazebo architecture?
2. Explain the difference between URDF and SDF formats in robotics simulation.
3. How does Gazebo simulate different physics engines? Name at least two.
4. Describe the process of integrating a Gazebo robot model with ROS2.
5. What are some common sensor types that can be simulated in Gazebo?