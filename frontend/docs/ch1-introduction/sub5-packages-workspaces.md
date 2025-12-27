---
sidebar_position: 6
---

# 1.5. Introduction to ROS2 Packages and Workspaces

## Understanding ROS2 Packages

A **ROS2 package** is the fundamental unit for organizing and distributing ROS2 software. It contains source code, data, and configuration files necessary to build and run specific functionality.

### Package Structure

A typical ROS2 package follows a standardized structure:

```
package_name/
├── CMakeLists.txt                 # Build configuration for CMake packages
├── package.xml                    # Package metadata and dependencies
├── src/                          # Source code files
├── include/                      # Header files (for C++)
├── scripts/                      # Script files (Python, shell, etc.)
├── launch/                       # Launch files for running multiple nodes
├── config/                       # Configuration files
├── test/                         # Unit and integration tests
├── msg/                          # Custom message definitions
├── srv/                          # Custom service definitions
├── action/                       # Custom action definitions
└── README.md                     # Package documentation
```

### Key Files

**package.xml**
- Contains package metadata (name, version, description)
- Lists dependencies
- Defines maintainers and authors
- Specifies build and execution dependencies

**CMakeLists.txt**
- Build configuration for CMake-based packages
- Defines targets, dependencies, and installation rules
- Specifies how to compile and link the software

### Package Manifest (package.xml)

The package.xml file is an XML file with important information:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_robot_package</name>
  <version>0.1.0</version>
  <description>A package for controlling my robot</description>
  <maintainer email="maintainer@todo.todo">maintainer</maintainer>
  <license>TODO</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>std_msgs</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

## ROS2 Workspaces

A **workspace** is a directory containing one or more packages that you want to build and work with together.

### Workspace Structure

A typical ROS2 workspace has this structure:

```
workspace_name/
├── src/              # Source packages (created by user)
├── build/            # Build output (created during build)
├── install/          # Installation space (created during build)
└── log/              # Build logs (created during build)
```

### Creating a Workspace

1. Create the workspace directory:
```bash
mkdir -p ~/ros2_workspace/src
cd ~/ros2_workspace
```

2. Create packages in the src directory:
```bash
cd src
ros2 pkg create --build-type ament_cmake my_robot_package
```

## Creating Packages

### Command Line Tool

Use the `ros2 pkg create` command to create new packages:

```bash
ros2 pkg create --build-type ament_cmake <package_name>
ros2 pkg create --build-type ament_python <package_name>
```

### Package Options

When creating packages, you can specify various options:

- `--dependencies`: List dependencies (e.g., `rclcpp`, `std_msgs`)
- `--maintainer-email`: Set maintainer email
- `--maintainer-name`: Set maintainer name
- `--description`: Set package description
- `--license`: Set license type

## Understanding Build Systems

### ament_cmake
- Uses CMake as the build system
- Suitable for C++ packages
- Most common build system

### ament_python
- Uses Python setuptools
- Suitable for Python packages
- Automatically creates setup.py

### colcon
- ROS2's build system that works with ament packages
- Builds multiple packages in parallel
- Handles dependencies between packages

## Managing Dependencies

### Package Dependencies

Packages can depend on other packages in different ways:

- **Build dependencies**: Needed to compile the package
- **Execution dependencies**: Needed to run the package
- **Test dependencies**: Needed to run tests

### Dependency Resolution

ROS2 resolves dependencies from:
- Other packages in the workspace
- System-level packages installed via apt
- Pre-compiled packages

## Build and Install Process

### Building with colcon

```bash
cd ~/ros2_workspace
colcon build
source install/setup.bash
```

### Build Options

Common colcon build options:
- `--packages-select <pkg1> <pkg2>`: Build only specific packages
- `--packages-up-to <pkg1>`: Build a package and its dependencies
- `--symlink-install`: Use symlinks instead of copying files
- `--event-handlers console_direct+`: Show all build output

## Using Multiple Workspaces

### Workspace Chaining

You can chain workspaces by sourcing them in order:
```bash
source /opt/ros/humble/setup.bash  # System ROS2
source ~/workspace1/install/setup.bash  # First workspace
source ~/workspace2/install/setup.bash  # Second workspace
```

Later workspaces override earlier ones when there are conflicts.

## Best Practices

### Package Design Principles

1. **Single Responsibility**: Each package should have one clear purpose
2. **Modularity**: Keep packages focused and well-defined
3. **Reusability**: Design packages that can be reused across projects
4. **Documentation**: Include clear documentation and examples

### Naming Conventions

- Use lowercase letters and underscores
- Use descriptive names that indicate functionality
- Avoid generic names like "utils" or "common"
- Follow ROS2 naming conventions

### Directory Organization

- Keep source code organized and well-structured
- Separate public APIs from internal implementation
- Use consistent folder naming across packages
- Document the purpose of major directories

## Package Management Commands

### Useful Commands

```bash
# List all packages in workspace
colcon list

# Find a specific package
ros2 pkg list | grep <package_name>

# Show package information
ros2 pkg info <package_name>

# Find package path
ros2 pkg prefix <package_name>
```

## Learning Objectives for This Section

After completing this section, you will be able to:
- Create and structure ROS2 packages according to conventions
- Set up and manage ROS2 workspaces
- Understand the build and installation process
- Manage package dependencies effectively
- Apply best practices for package organization

## Summary

ROS2 packages and workspaces provide a standardized way to organize, build, and distribute robotic software. Understanding this structure is essential for developing maintainable and reusable robot applications. Packages encapsulate specific functionality, while workspaces allow you to work with multiple packages together. The build system (colcon) handles dependencies and compilation, making it easy to develop complex robotic systems with multiple components.