---
sidebar_position: 16
---

# Lab 1.2: Creating Your First Publisher and Subscriber Nodes

## Objective

To create custom ROS2 publisher and subscriber nodes in both C++ and Python, building on the foundational knowledge of ROS2 concepts.

## Prerequisites

- ROS2 Humble Hawksbill installed
- Chapter 1 completed (especially sections 4-6)
- Basic knowledge of C++ or Python
- Understanding of ROS2 nodes, topics, and message passing

## Required Equipment/Software

- Ubuntu 22.04 with ROS2 Humble installed
- Text editor or IDE
- Terminal access

## Learning Goals

After completing this lab, you will be able to:
- Create a custom ROS2 package for your nodes
- Implement a publisher node in both C++ and Python
- Implement a subscriber node in both C++ and Python
- Build and run your custom ROS2 nodes
- Understand the structure of ROS2 packages and nodes

## Setup Instructions

Ensure you have completed Lab 1.1 and have ROS2 properly installed and configured.

## Procedure

### Step 1: Create a Workspace and Package (15 minutes)

1. Create a workspace directory:
   ```bash
   mkdir -p ~/ros2_labs_ws/src
   cd ~/ros2_labs_ws
   ```

2. Create a package for our lab:
   ```bash
   cd src
   ros2 pkg create --build-type ament_cmake my_robot_tutorial --dependencies rclcpp rclpy std_msgs
   ```

3. Navigate to the package directory:
   ```bash
   cd my_robot_tutorial
   ```

### Step 2: Create Publisher Node in C++ (20 minutes)

1. Create the source directory for C++ code:
   ```bash
   mkdir -p src
   ```

2. Create the publisher node (`src/my_publisher.cpp`):
   ```cpp
   #include "rclcpp/rclcpp.hpp"
   #include "std_msgs/msg/string.hpp"

   using namespace std::chrono_literals;

   class MinimalPublisher : public rclcpp::Node
   {
   public:
       MinimalPublisher()
       : Node("minimal_publisher"), count_(0)
       {
           publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
           timer_ = this->create_wall_timer(
               500ms, std::bind(&MinimalPublisher::timer_callback, this));
       }

   private:
       void timer_callback()
       {
           auto message = std_msgs::msg::String();
           message.data = "Hello World: " + std::to_string(count_++);
           RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
           publisher_->publish(message);
       }
       rclcpp::TimerBase::SharedPtr timer_;
       rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
       size_t count_;
   };

   int main(int argc, char * argv[])
   {
       rclcpp::init(argc, argv);
       rclcpp::spin(std::make_shared<MinimalPublisher>());
       rclcpp::shutdown();
       return 0;
   }
   ```

### Step 3: Create Subscriber Node in C++ (15 minutes)

1. Create the subscriber node (`src/my_subscriber.cpp`):
   ```cpp
   #include "rclcpp/rclcpp.hpp"
   #include "std_msgs/msg/string.hpp"

   class MinimalSubscriber : public rclcpp::Node
   {
   public:
       MinimalSubscriber()
       : Node("minimal_subscriber")
       {
           subscription_ = this->create_subscription<std_msgs::msg::String>(
               "topic", 10,
               [this](const std_msgs::msg::String::SharedPtr msg) {
                   RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
               });
       }

   private:
       rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
   };

   int main(int argc, char * argv[])
   {
       rclcpp::init(argc, argv);
       rclcpp::spin(std::make_shared<MinimalSubscriber>());
       rclcpp::shutdown();
       return 0;
   }
   ```

### Step 4: Create Publisher Node in Python (15 minutes)

1. Create the Python directory:
   ```bash
   mkdir -p my_robot_tutorial
   ```

2. Create the publisher node (`my_robot_tutorial/my_publisher.py`):
   ```python
   #!/usr/bin/env python3
   import rclpy
   from rclpy.node import Node
   from std_msgs.msg import String


   class MinimalPublisher(Node):

       def __init__(self):
           super().__init__('minimal_publisher')
           self.publisher_ = self.create_publisher(String, 'topic', 10)
           timer_period = 0.5  # seconds
           self.timer = self.create_timer(timer_period, self.timer_callback)
           self.i = 0

       def timer_callback(self):
           msg = String()
           msg.data = 'Hello World: %d' % self.i
           self.publisher_.publish(msg)
           self.get_logger().info('Publishing: "%s"' % msg.data)
           self.i += 1


   def main(args=None):
       rclpy.init(args=args)
       minimal_publisher = MinimalPublisher()
       rclpy.spin(minimal_publisher)
       minimal_publisher.destroy_node()
       rclpy.shutdown()


   if __name__ == '__main__':
       main()
   ```

### Step 5: Create Subscriber Node in Python (10 minutes)

1. Create the subscriber node (`my_robot_tutorial/my_subscriber.py`):
   ```python
   #!/usr/bin/env python3
   import rclpy
   from rclpy.node import Node
   from std_msgs.msg import String


   class MinimalSubscriber(Node):

       def __init__(self):
           super().__init__('minimal_subscriber')
           self.subscription = self.create_subscription(
               String,
               'topic',
               self.listener_callback,
               10)
           self.subscription  # prevent unused variable warning

       def listener_callback(self, message):
           self.get_logger().info('I heard: "%s"' % message.data)


   def main(args=None):
       rclpy.init(args=args)
       minimal_subscriber = MinimalSubscriber()
       rclpy.spin(minimal_subscriber)
       minimal_subscriber.destroy_node()
       rclpy.shutdown()


   if __name__ == '__main__':
       main()
   ```

### Step 6: Update CMakeLists.txt (10 minutes)

1. Open the `CMakeLists.txt` file in the package root and add the following executable definitions before the `ament_package()` line:

   ```cmake
   find_package(ament_cmake REQUIRED)
   find_package(rclcpp REQUIRED)
   find_package(rclpy REQUIRED)
   find_package(std_msgs REQUIRED)

   # C++ executables
   add_executable(my_publisher src/my_publisher.cpp)
   ament_target_dependencies(my_publisher rclcpp std_msgs)

   add_executable(my_subscriber src/my_subscriber.cpp)
   ament_target_dependencies(my_subscriber rclcpp std_msgs)

   # Install executables
   install(TARGETS
     my_publisher
     my_subscriber
     DESTINATION lib/${PROJECT_NAME}
   )

   # Install Python modules
   ament_python_install_package(${PROJECT_NAME})
   ```

### Step 7: Update package.xml (5 minutes)

1. Make sure your `package.xml` includes all necessary dependencies:

   ```xml
   <?xml version="1.0"?>
   <?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
   <package format="3">
     <name>my_robot_tutorial</name>
     <version>0.1.0</version>
     <description>My first ROS2 publisher and subscriber tutorial</description>
     <maintainer email="user@todo.todo">user</maintainer>
     <license>TODO</license>

     <buildtool_depend>ament_cmake</buildtool_depend>

     <depend>rclcpp</depend>
     <depend>rclpy</depend>
     <depend>std_msgs</depend>

     <test_depend>ament_lint_auto</test_depend>
     <test_depend>ament_lint_common</test_depend>

     <export>
       <build_type>ament_cmake</build_type>
     </export>
   </package>
   ```

### Step 8: Make Python Files Executable and Build (10 minutes)

1. Make Python files executable:
   ```bash
   cd ~/ros2_labs_ws/src/my_robot_tutorial
   chmod +x my_robot_tutorial/my_publisher.py
   chmod +x my_robot_tutorial/my_subscriber.py
   ```

2. Build the package:
   ```bash
   cd ~/ros2_labs_ws
   colcon build --packages-select my_robot_tutorial
   ```

3. Source the workspace:
   ```bash
   source install/setup.bash
   ```

### Step 9: Run Your Custom Nodes (15 minutes)

1. Open a new terminal and run the publisher:
   ```bash
   source ~/ros2_labs_ws/install/setup.bash
   ros2 run my_robot_tutorial my_publisher
   ```

2. In another terminal, run the subscriber:
   ```bash
   source ~/ros2_labs_ws/install/setup.bash
   ros2 run my_robot_tutorial my_subscriber
   ```

3. You should see the publisher sending messages and the subscriber receiving them.

4. Alternatively, use Python versions:
   ```bash
   # Terminal for publisher
   source ~/ros2_labs_ws/install/setup.bash
   ros2 run my_robot_tutorial my_publisher.py
   
   # Terminal for subscriber
   source ~/ros2_labs_ws/install/setup.bash
   ros2 run my_robot_tutorial my_subscriber.py
   ```

### Step 10: Experiment with Different Configurations (15 minutes)

1. Try changing the message rate in the publisher (change timer_period in Python or timer interval in C++)
2. Create a different message type (try Int32 instead of String)
3. Add more complex data to the message

## Expected Outcomes

After completing this lab, you will have:
- Created a custom ROS2 package with proper structure
- Implemented publisher and subscriber nodes in both C++ and Python
- Built and executed custom ROS2 nodes
- Observed the publish-subscribe communication pattern in action
- Gained experience with ROS2 development workflow

## Troubleshooting

### Issue: "command not found" for custom nodes
**Solution**: Ensure you've sourced the workspace after building:
```bash
source ~/ros2_labs_ws/install/setup.bash
```

### Issue: Nodes don't communicate
**Solution**: Verify both nodes are using the same topic name and message type

### Issue: Build errors
**Solution**: Check for typos in CMakeLists.txt and package.xml, ensure all dependencies are properly declared

### Issue: Python nodes not running
**Solution**: Ensure Python files are executable and have proper shebang lines

## Assessment Criteria

- [ ] Custom package created successfully with proper structure
- [ ] C++ publisher and subscriber nodes implemented and functional
- [ ] Python publisher and subscriber nodes implemented and functional
- [ ] Nodes successfully communicate messages
- [ ] Understanding demonstrated by explaining the node code structure
- [ ] Lab report submitted with code and output examples

## Extensions

For advanced students:
1. Create a launch file that runs both publisher and subscriber simultaneously
2. Add parameters to control message rate or content
3. Implement a service server in addition to the publisher/subscriber pattern
4. Add custom message types to your package

## Notes for Instructors

- Students may have different comfort levels with C++ vs Python; allow them to focus on their preferred language
- The lab requires understanding of both build systems (CMake for C++, setuptools for Python)
- Emphasize the similarities in structure between C++ and Python implementations
- Discuss the advantages of each language for different robotics applications