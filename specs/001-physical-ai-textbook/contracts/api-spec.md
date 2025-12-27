# API Specification: Physical AI Textbook Code Examples

## Overview
This document specifies the interfaces and APIs that are referenced in the Physical AI & Humanoid Robotics textbook. These APIs are used in code examples throughout the book.

## ROS2 Service Definitions

### Navigation Service
**Purpose**: Request robot navigation to a specific goal position

```yaml
# nav2_msgs/action/NavigateToPose
# Used in Chapter 7 - Navigation Systems
NavigateToPose:
  goal_service: NavigateToPose
  request:
    pose: geometry_msgs/PoseStamped
      header:
        stamp: builtin_interfaces/Time
        frame_id: string
      pose:
        position: geometry_msgs/Point
          x: float64
          y: float64
          z: float64
        orientation: geometry_msgs/Quaternion
          x: float64
          y: float64
          z: float64
          w: float64
  result:
    result_code: int8
      FAILURE = 0
      SUCCESS = 1
    message: string
```

### Perception Service
**Purpose**: Request object detection in a camera image

```yaml
# Custom service for perception examples
# Used in Chapter 2 - Robot Perception Systems
ObjectDetection:
  service_type: perception_msgs/ObjectDetection
  request:
    image: sensor_msgs/Image
      header:
        stamp: builtin_interfaces/Time
        frame_id: string
      height: uint32
      width: uint32
      encoding: string
      is_bigendian: uint8
      step: uint32
      data: uint8[]
  response:
    objects: Object[]
      label: string
      confidence: float32
      bbox: BoundingBox2D
        x_offset: uint32
        y_offset: uint32
        width: uint32
        height: uint32
      pose: geometry_msgs/Pose
        position: geometry_msgs/Point
          x: float64
          y: float64
          z: float64
        orientation: geometry_msgs/Quaternion
          x: float64
          y: float64
          z: float64
          w: float64
    processing_time: float64  # in seconds
```

## Isaac ROS Action Definitions

### Isaac Manipulation Action
**Purpose**: Perform manipulation tasks using Isaac platform

```yaml
# Isaac-specific action for manipulation
# Used in Chapter 6 - Robot Control and Manipulation
MoveToPose:
  goal_service: MoveToPose
  request:
    target_pose: geometry_msgs/PoseStamped
      header:
        stamp: builtin_interfaces/Time
        frame_id: string
      pose:
        position: geometry_msgs/Point
          x: float64
          y: float64
          z: float64
        orientation: geometry_msgs/Quaternion
          x: float64
          y: float64
          z: float64
          w: float64
    timeout: builtin_interfaces/Duration
  result:
    success: bool
    error_code: int32
    message: string
```

## Data Message Formats

### Sensor Fusion Message
**Purpose**: Unified message for multi-sensor perception data

```yaml
# Used in Chapter 2 - Robot Perception Systems
# Custom message type for sensor fusion examples
FusedSensorData:
  header:
    stamp: builtin_interfaces/Time
    frame_id: string
  objects: FusedObject[]  # Array of detected objects
    object_id: uint32
    object_type: string
    confidence: float32
    position_3d: geometry_msgs/Point
      x: float64
      y: float64
      z: float64
    position_2d: geometry_msgs/Point  # In camera frame
      x: float64
      y: float64
      z: float64  # Not used, kept for standardization
    velocity: geometry_msgs/Vector3
      x: float64
      y: float64
      z: float64
  sensor_status: SensorStatus[]  # Status of each sensor
    sensor_id: string
    sensor_type: string
    operational: bool
    data_quality: float32  # 0.0 to 1.0
```

## VLA (Vision Language Action) Interface

### Command Execution Interface
**Purpose**: Interface for executing natural language commands on robots

```yaml
# Used in Chapter 5 - Vision Language Action Models
ExecuteCommand:
  service_type: vla_msgs/ExecuteCommand
  request:
    command: string  # Natural language command
    context_image: sensor_msgs/Image  # Image of current scene
    robot_state: geometry_msgs/Pose  # Current robot state
  response:
    success: bool
    action_sequence: Action[]  # Planned sequence of actions
    confidence: float32  # Confidence in plan
    error_message: string  # If success is false
    execution_time: builtin_interfaces/Duration
```

## Lab Exercise Interfaces

### Lab Evaluation Service
**Purpose**: Interface for evaluating lab exercise completion

```yaml
# Used in lab exercises throughout the book
EvaluateLab:
  service_type: textbook_msgs/EvaluateLab
  request:
    lab_id: string  # Identifier for the specific lab
    student_solution: string  # Path to student's solution
    evaluation_criteria: string[]  # List of criteria to evaluate
  response:
    results: EvaluationResult[]
      criterion: string
      achieved: bool
      score: float32  # 0.0 to 1.0
      feedback: string
    overall_score: float32
    passed: bool
    feedback: string
```

## Validation Rules

1. All coordinate frames must follow ROS2 conventions (right-hand rule)
2. Timestamps must be synchronized across sensors when possible
3. Service response times must be less than 5 seconds for interactive applications
4. All pose estimates must include associated uncertainty measures
5. Error handling must be implemented in all examples

## Implementation Notes

- All examples in the textbook assume ROS2 Humble Hawksbill
- Coordinate frames: Robot base frame is "base_link", camera frame is "camera_color_optical_frame"
- Units: Distances in meters, angles in radians, time in seconds
- Coordinate system: X forward, Y left, Z up (ROS standard)

## Compatibility

- ROS2 Humble Hawksbill (minimum requirement)
- Gazebo Garden for simulation
- Python 3.10+ for interfaces
- Isaac ROS 3.x for Isaac-specific examples