---
sidebar_position: 5
---

# 3.3 Creating Robot Models for Gazebo

## Chapter 3: Simulation Environments - Gazebo and Unity

Creating accurate and efficient robot models is essential for effective simulation in Gazebo. A well-designed robot model impacts simulation performance, accuracy, and the success of sim-to-real transfer of developed algorithms.

## Understanding URDF (Unified Robot Description Format)

URDF is the standard XML format for representing robot models in ROS and Gazebo. It describes the robot's physical and kinematic properties, including:

- **Links**: Rigid body components of the robot
- **Joints**: Connections between links with specific degrees of freedom
- **Visual elements**: How the robot appears in simulation
- **Collision elements**: Shapes used for collision detection
- **Inertial properties**: Mass, center of mass, and inertia tensor

### Basic URDF Structure

```xml
<?xml version="1.0"?>
<robot name="my_robot">
  <!-- Link definition -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
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
  
  <!-- Joint definition -->
  <joint name="base_to_wheel" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_link"/>
    <origin xyz="0 0.3 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>
  
  <link name="wheel_link">
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.1"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.1" radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.02"/>
    </inertial>
  </link>
</robot>
```

## Visual Elements in URDF

Visual elements define how the robot appears in the simulation. They include:

- **Geometry**: Shape of the visual element (box, cylinder, sphere, mesh)
- **Material**: Color and texture properties
- **Origin**: Position and orientation relative to the link frame

### Geometry Types

```xml
<!-- Box -->
<geometry>
  <box size="0.1 0.2 0.3"/>
</geometry>

<!-- Cylinder -->
<geometry>
  <cylinder radius="0.1" length="0.2"/>
</geometry>

<!-- Sphere -->
<geometry>
  <sphere radius="0.1"/>
</geometry>

<!-- Mesh -->
<geometry>
  <mesh filename="package://my_robot/meshes/base_link.stl" scale="1 1 1"/>
</geometry>
```

## Collision Elements in URDF

Collision elements define shapes used for collision detection. These should be as simple as possible for better performance while maintaining accuracy.

```xml
<collision>
  <!-- Simple box collision -->
  <geometry>
    <box size="0.1 0.2 0.3"/>
  </geometry>
</collision>
```

For complex robots, you can use multiple collision elements per link:

```xml
<link name="complex_link">
  <collision>
    <origin xyz="0.1 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="0.2 0.1 0.1"/>
    </geometry>
  </collision>
  <collision>
    <origin xyz="-0.1 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="0.2 0.1 0.1"/>
    </geometry>
  </collision>
</link>
```

## Inertial Properties

Accurate inertial properties are critical for realistic physics simulation. Each link requires:

- **Mass**: Scalar value in kilograms
- **Inertia tensor**: 6 independent values (ixx, ixy, ixz, iyy, iyz, izz)

```xml
<inertial>
  <mass value="1.0"/>
  <inertia 
    ixx="0.01" ixy="0.0" ixz="0.0"
    iyy="0.01" iyz="0.0" 
    izz="0.02"/>
</inertial>
```

## Joint Types in URDF

Different joint types enable various robot movements:

- **Fixed**: No movement between links
- **Continuous**: Continuous rotation (like a wheel)
- **Revolute**: Limited rotation (like a servo)
- **Prismatic**: Linear sliding movement
- **Planar**: Movement on a plane
- **Floating**: 6 DOF movement

```xml
<!-- Revolute joint with limits -->
<joint name="arm_joint" type="revolute">
  <parent link="shoulder_link"/>
  <child link="elbow_link"/>
  <origin xyz="0 0 0.5" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
</joint>

<!-- Continuous joint -->
<joint name="wheel_joint" type="continuous">
  <parent link="base_link"/>
  <child link="wheel_link"/>
  <origin xyz="0 0.3 0" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
</joint>
```

## Adding Gazebo-Specific Extensions

To make URDF models work properly in Gazebo, you need to add Gazebo-specific elements using the `<gazebo>` tag:

```xml
<!-- Add plugin to a link -->
<gazebo reference="wheel_link">
  <mu1>0.5</mu1>
  <mu2>0.5</mu2>
  <kp>1000000.0</kp>
  <kd>100.0</kd>
</gazebo>

<!-- Add a sensor -->
<gazebo reference="camera_mount">
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
    <plugin filename="libgazebo_ros_camera.so" name="camera_controller">
      <frame_name>camera_optical_frame</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

## Creating a Complete Differential Drive Robot

Let's create a complete URDF for a simple differential drive robot with camera and LiDAR:

```xml
<?xml version="1.0"?>
<robot name="diff_drive_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
  
  <!-- Base properties -->
  <xacro:property name="base_width" value="0.3"/>
  <xacro:property name="base_length" value="0.4"/>
  <xacro:property name="base_height" value="0.15"/>
  <xacro:property name="wheel_radius" value="0.05"/>
  <xacro:property name="wheel_width" value="0.05"/>
  <xacro:property name="wheel_x_offset" value="0.15"/>
  <xacro:property name="wheel_y_offset" value="0.18"/>
  
  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <origin xyz="0 0 ${base_height/2}" rpy="0 0 0"/>
      <geometry>
        <box size="${base_length} ${base_width} ${base_height}"/>
      </geometry>
      <material name="orange">
        <color rgba="1 0.5 0 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 ${base_height/2}" rpy="0 0 0"/>
      <geometry>
        <box size="${base_length} ${base_width} ${base_height}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5"/>
      <inertia 
        ixx="0.15" ixy="0.0" ixz="0.0"
        iyy="0.2" iyz="0.0" 
        izz="0.3"/>
    </inertial>
  </link>
  
  <!-- Wheels -->
  <xacro:macro name="wheel" params="prefix x_reflect y_reflect">
    <link name="${prefix}_wheel">
      <visual>
        <origin xyz="0 0 0" rpy="${pi/2} 0 0"/>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
      </visual>
      <collision>
        <origin xyz="0 0 0" rpy="${pi/2} 0 0"/>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="0.5"/>
        <inertia 
          ixx="0.001" ixy="0.0" ixz="0.0"
          iyy="0.001" iyz="0.0" 
          izz="0.002"/>
      </inertial>
    </link>
    
    <joint name="${prefix}_wheel_joint" type="continuous">
      <parent link="base_link"/>
      <child link="${prefix}_wheel"/>
      <origin xyz="${x_reflect*wheel_x_offset} ${y_reflect*wheel_y_offset} 0" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
    </joint>
  </xacro:macro>
  
  <xacro:wheel prefix="left" x_reflect="1" y_reflect="1"/>
  <xacro:wheel prefix="right" x_reflect="1" y_reflect="-1"/>
  
  <!-- Caster -->
  <link name="caster_wheel">
    <visual>
      <geometry>
        <sphere radius="${wheel_radius/2}"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <sphere radius="${wheel_radius/2}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia 
        ixx="0.0001" ixy="0.0" ixz="0.0"
        iyy="0.0001" iyz="0.0" 
        izz="0.0001"/>
    </inertial>
  </link>
  
  <joint name="caster_wheel_joint" type="fixed">
    <parent link="base_link"/>
    <child link="caster_wheel"/>
    <origin xyz="${-base_length/2} 0 ${-wheel_radius/2+base_height/2}" rpy="0 0 0"/>
  </joint>
  
  <!-- Sensors -->
  <link name="camera_link">
    <visual>
      <geometry>
        <box size="0.05 0.1 0.03"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.05 0.1 0.03"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia 
        ixx="0.0001" ixy="0.0" ixz="0.0"
        iyy="0.0001" iyz="0.0" 
        izz="0.0001"/>
    </inertial>
  </link>
  
  <joint name="camera_joint" type="fixed">
    <parent link="base_link"/>
    <child link="camera_link"/>
    <origin xyz="${base_length/2-0.02} 0 ${base_height/2+0.02}" rpy="0 0 0"/>
  </joint>
  
  <!-- Gazebo extensions -->
  <gazebo reference="base_link">
    <material>Gazebo/Orange</material>
  </gazebo>
  
  <gazebo reference="left_wheel">
    <material>Gazebo/Black</material>
    <mu1>1.0</mu1>
    <mu2>1.0</mu2>
  </gazebo>
  
  <gazebo reference="right_wheel">
    <material>Gazebo/Black</material>
    <mu1>1.0</mu1>
    <mu2>1.0</mu2>
  </gazebo>
  
  <!-- Camera sensor -->
  <gazebo reference="camera_link">
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
      <plugin filename="libgazebo_ros_camera.so" name="camera_controller">
        <frame_name>camera_optical_frame</frame_name>
        <min_depth>0.1</min_depth>
        <max_depth>100.0</max_depth>
      </plugin>
    </sensor>
  </gazebo>
  
  <!-- ROS2 Control plugin for differential drive -->
  <gazebo>
    <plugin filename="libgazebo_ros_diff_drive.so" name="diff_drive">
      <commandTopic>cmd_vel</commandTopic>
      <odometryTopic>odom</odometryTopic>
      <odometryFrame>odom</odometryFrame>
      <robotBaseFrame>base_link</robotBaseFrame>
      <publishOdomTF>true</publishOdomTF>
      <wheelDiameter>0.1</wheelDiameter>
      <wheelSeparation>0.36</wheelSeparation>
      <maxWheelTorque>20</maxWheelTorque>
      <maxWheelSpeed>10</maxWheelSpeed>
    </plugin>
  </gazebo>
</robot>
```

## Verifying Your Robot Model

### Checking URDF Syntax

```bash
# Check URDF syntax
check_urdf /path/to/robot.urdf

# Parse and display robot structure
urdf_to_graphiz /path/to/robot.urdf
```

### Visualizing the Model

```bash
# Use RViz to visualize the robot
ros2 run rviz2 rviz2

# Or use the robot_state_publisher to check
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:=$(cat /path/to/robot.urdf)
```

## Best Practices for Robot Modeling

1. **Start Simple**: Begin with basic geometric shapes, then add detail
2. **Accurate Inertial Properties**: Use CAD software to calculate mass and inertia
3. **Realistic Friction Values**: Set appropriate friction coefficients for realistic interaction
4. **Efficient Collision Geometry**: Use simple shapes for collision detection
5. **Modular Design**: Use Xacro macros for reusable components
6. **Realistic Sensor Models**: Include appropriate sensor noise and limitations

## Summary

Creating accurate robot models for Gazebo requires understanding URDF's structure and Gazebo's specific requirements. This section covered the essential components of robot models, including visual, collision, and inertial properties, joint types, and Gazebo-specific extensions. A well-designed robot model enables effective simulation and improves sim-to-real transfer of developed algorithms.

## Exercises

1. Create a URDF model for a simple wheeled robot with a camera sensor
2. Load your robot model in Gazebo and test its basic functionality
3. Modify the inertial properties and observe the effect on the robot's movement
4. Add a LiDAR sensor to your robot and verify its output in simulation