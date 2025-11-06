# JeTank ROS2 Integration

<div align="center">
  <img src="https://www.waveshare.com/media/catalog/product/cache/1/image/800x800/9df78eab33525d08d6e5fb8d27136e95/j/e/jetank-ai-kit-1.jpg" alt="JeTank Robot with ROS Integration" />
  <p><i>ROS integration for the Waveshare JeTank AI Kit with autonomous sock collection functionality</i></p>
</div>

## 🤖 Project Overview

This repository provides a complete Robot Operating System (ROS) integration for the [Waveshare JeTank AI Kit](https://www.waveshare.com/wiki/JETANK_AI_Kit), transforming it into a fully autonomous robot platform. Building upon the [original Waveshare JeTank repository](https://github.com/waveshare/JETANK), this project adds ROS2 compatibility and extends the robot's capabilities to include autonomous sock detection and collection in indoor environments.

**This is all work in progress!**

For recent development logs and notes, see the [Updates](#updates) section.

## 🛠️ Hardware updates

- **Camera module**: Updated the stock IMX219-160 Camera module to a IMX219-83 Stereo Camera module to allow for depth perception
- **Computation module**: The basic jetank is made for a Jetson nano developer B01, But I want to upgrade it to the new Orin Jetson super developer kit

## ✨ Key Features

- **Complete ROS Integration**: Full conversion of the JeTank platform to ROS architecture
- **ROS Control Package**: Comprehensive drivers for all JeTank hardware:
  - Motor control interface
  - Camera integration
  - Servo controllers
  - Sensor data publishing
- **Autonomous Navigation**: SLAM-based mapping and path planning
- **Computer Vision Pipeline**: Object detection system for sock identification
- **Custom Gripper Mechanism**: Design files for sock collection attachment

## 🛠️ Technologies & Skills Demonstrated

- **ROS2 Humble**: Full node architecture with publishers/subscribers
- **NVIDIA Jetson Orin Nano Super**: Optimized for the Developer Kit
- **Python**: Clean, modular codebase for hardware interfacing and integration
- **C++**: Performance-critical components (motor control, perception pipeline)
- **Computer Vision**: OpenCV with GPU acceleration for stereo vision
- **Point Cloud Processing**: PCL for 3D data processing and filtering
- **SLAM**: PointCloud2 to LaserScan conversion for navigation stack
- **Gazebo Simulation**: Complete simulation environment for testing and development
- **URDF/Xacro**: Modular robot description following ROS2 best practices
- **Hardware Integration**: Bridging Jetson Orin Nano with ROS2 ecosystem

## 🏗️ System Architecture

```
├── jetank_motor_control/          (C++/ament_cmake)
│   ├── include/motor.hpp          # Motor control library header
│   └── src/motor/
│       ├── motor.cpp              # GPIO-based motor control implementation
│       └── robot_controller.cpp   # ROS2 motor controller node
│
├── jetank_perception/             (C++/ament_cmake)
│   ├── include/jetank_perception/ # Header-only stereo processing strategies
│   │   ├── camera_interface.hpp  # Camera abstraction (CSI/USB/Virtual)
│   │   ├── stereo_processing_strategy.hpp  # Stereo matching algorithms
│   │   └── quality_monitor.hpp   # Quality monitoring and filtering
│   ├── src/
│   │   ├── stereo_camera_node.cpp  # Main stereo perception node
│   │   └── single_camera.cpp       # Single camera testing node
│   ├── config/
│   │   ├── stereo_camera_config.yaml  # Complete stereo configuration
│   │   └── calibration/           # Camera calibration files
│   └── launch/
│       ├── stereo_camera.launch.py  # Stereo perception system
│       └── single_camera.launch.py  # Single camera testing
│
├── jetank_navigation/             (C++/ament_cmake) [In Development - Phase 2]
│   ├── include/jetank_navigation/
│   │   └── utils.hpp              # Navigation utility functions
│   ├── config/
│   │   └── laser_data.yaml        # LaserScan conversion configuration
│   ├── LEARNING_PLAN.md           # Phased learning approach
│   └── PROGRESS.md                # Current development status
│   # Purpose: Convert PointCloud2 to LaserScan for SLAM navigation
│
├── jetank_description/            (C++/ament_cmake)
│   ├── urdf/                      # Robot description files
│   │   ├── jetank.xacro           # Main robot description
│   │   ├── jetank_arm.xacro       # Arm configuration
│   │   ├── jetank_camera.xacro    # Camera mounts
│   │   ├── jetank_wheel.xacro     # Wheel definitions
│   │   └── ...                    # Additional URDF components
│   ├── meshes/                    # 3D mesh files (STL/DAE)
│   ├── launch/                    # Description launch files
│   └── config/                    # Robot configuration parameters
│   # Purpose: Centralized robot description following ROS2 best practices
│
├── jetank_simulation/             (C++/ament_cmake)
│   ├── worlds/                    # Gazebo world files
│   │   └── empty.world            # Empty testing environment
│   ├── launch/
│   │   └── gazebo.launch.py       # Gazebo simulation launcher
│   ├── config/                    # Simulation configuration
│   └── models/                    # Custom Gazebo models
│   # Purpose: Gazebo simulation environment for testing and development
│
└── jetank_ros_main/               (Python/ament_python)
    ├── launch/
    │   ├── main.launch.py         # Full system integration
    │   ├── motor_controller.launch.py  # Motor control
    │   ├── stereo_camera.launch.py     # Stereo perception
    │   └── urdf.launch.py         # Robot model visualization
    ├── config/
    │   └── motor_params.yaml      # Motor control parameters
    └── jetank_ros2_main/          # Python package for integration scripts
```

## 📋 Project Roadmap & TODO

### Core Infrastructure
- [x] Set up ROS2 workspace structure
- [x] Update hardware (Jetson Orin Nano Super + IMX219-83 Stereo Camera)
- [x] Complete URDF model for visualization in RViz
- [x] Split functionality into separate packages

### Motor Control (jetank_motor_control)
- [x] Implement GPIO-based motor control library
- [x] Create ROS2 motor controller node
- [x] Integrate with ROS2 control framework

### Perception (jetank_perception)
- [x] Implement stereo camera node with CSI/USB support
- [x] Create camera calibration system
- [x] Develop point cloud generation pipeline
- [x] Add GPU-accelerated stereo matching
- [x] Implement quality monitoring and filtering
- [x] Optimize for real-time performance on Jetson

### Robot Description (jetank_description)
- [x] Create jetank_description package following ROS2 best practices
- [x] Move URDF/xacro files from jetank_ros_main
- [x] Configure package with proper dependencies and install rules
- [x] Set up directory structure (urdf, meshes, launch, config)
- [ ] Add Gazebo-specific tags and plugins to URDF
- [ ] Create or source 3D mesh files for visualization
- [ ] Add display.launch.py for RViz visualization
- [ ] Document robot kinematic structure

### Simulation (jetank_simulation)
- [x] Create jetank_simulation package for Gazebo
- [x] Set up package structure (worlds, launch, config, models)
- [x] Create empty.world for basic testing
- [x] Implement gazebo.launch.py for robot spawning
- [ ] Add differential drive plugin to URDF
- [ ] Add camera plugins for stereo vision simulation
- [ ] Create additional test worlds (obstacles, sock collection)
- [ ] Configure ros2_control for simulated motors
- [ ] Test sensor data publishing in simulation
- [ ] Integrate with navigation stack in simulation

### Navigation (jetank_navigation) [In Progress - Phase 2]
- [x] Design node architecture for PointCloud2 to LaserScan conversion
- [x] Create utility functions (FOV calculations, angle conversions)
- [x] Research camera specifications (IMX219-83: 73° horizontal FOV)
- [ ] Complete configuration parameters (laser angles, QoS settings)
- [ ] Implement PointCloud2 to LaserScan conversion node
- [ ] Integrate with ROS2 Navigation Stack (Nav2)
- [ ] Implement SLAM for mapping and localization
- [ ] Test autonomous navigation capabilities

### Manipulation (Future)
- [ ] Develop servo control interface for robot arm
- [ ] Design and implement gripper attachment
- [ ] Create MoveIt2 configuration for arm and gripper
- [ ] Integrate manipulation with navigation

### Vision & AI (Future)
- [ ] Add object segmentation model
- [ ] Implement sock detection algorithm
- [ ] Integrate vision pipeline with navigation
- [ ] Develop autonomous sock collection behavior

### Testing & Documentation
- [ ] Create comprehensive documentation
- [ ] Add unit tests for core components
- [ ] Test system integration in various environments
- [ ] Create usage tutorials and examples

## 🚀 Installation & Setup

### Prerequisites
- NVIDIA Jetson Orin Nano Super Developer Kit
- Ubuntu 22.04 (JetPack 6.0+)
- ROS2 Humble

### ROS2 Installation

```bash
# Install ROS2 Humble on Ubuntu 22.04
sudo apt update && sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y ros-humble-desktop

# Install development tools
sudo apt install -y ros-dev-tools python3-colcon-common-extensions

# Initialize rosdep
sudo rosdep init
rosdep update
```

### Workspace Setup

```bash
# Create ROS2 workspace
mkdir -p ~/workspaces/ros2_ws/src
cd ~/workspaces/ros2_ws

# Clone this repository
cd src
git clone <repository-url> jetank_ros_main
# Note: Adjust repository URL as needed

# Install dependencies
cd ~/workspaces/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
colcon build

# Source the workspace
source install/setup.bash
echo "source ~/workspaces/ros2_ws/install/setup.bash" >> ~/.bashrc
```

### Hardware Dependencies

```bash
# Install GPIO library for motor control
sudo apt install -y libgpiod-dev

# Install camera and vision dependencies
sudo apt install -y \
    libopencv-dev \
    libpcl-dev \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-camera-info-manager \
    ros-humble-pcl-conversions

# Install GStreamer for Jetson camera support
sudo apt install -y \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad

# Install navigation dependencies
sudo apt install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup

# Install Gazebo and simulation dependencies
sudo apt install -y \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-plugins \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-controller-manager
```

## 📁 Usage

### Build and Source

```bash
# Build all packages
cd ~/workspaces/ros2_ws
colcon build

# Source the workspace
source install/setup.bash
```

### Launch Individual Components

```bash
# Launch stereo camera perception system
ros2 launch jetank_perception stereo_camera.launch.py

# Launch single camera for testing
ros2 launch jetank_perception single_camera.launch.py

# Launch motor controller
ros2 launch jetank_ros_main motor_controller.launch.py

# Launch URDF visualization in RViz
ros2 launch jetank_ros_main urdf.launch.py

# Launch Gazebo simulation
ros2 launch jetank_simulation gazebo.launch.py

# Launch Gazebo without GUI (headless)
ros2 launch jetank_simulation gazebo.launch.py gui:=false
```

### Launch Full System

```bash
# Launch complete JeTank system (real hardware)
ros2 launch jetank_ros_main main.launch.py

# Launch complete system in simulation (when integrated)
ros2 launch jetank_simulation gazebo.launch.py
```

### Run Individual Nodes

```bash
# Run stereo camera node
ros2 run jetank_perception stereo_camera_node

# Run robot controller
ros2 run jetank_motor_control robot_controller
```

### Monitor Topics

```bash
# List all active topics
ros2 topic list

# View stereo camera point cloud
ros2 topic echo /stereo_camera/points

# Monitor motor commands
ros2 topic echo /cmd_vel

# Check LaserScan data (when navigation is ready)
ros2 topic echo /scan
```

## 🎯 Technical Challenges & Solutions

### Challenge 1: Stereo Vision on Embedded Hardware
**Problem**: Real-time stereo processing at 30fps with limited computational resources.
**Solution**:
- Implemented header-only strategy pattern for compile-time optimization
- Leveraged GPU acceleration when available (CUDA/OpenCV)
- Created efficient point cloud filtering and quality monitoring
- Configurable processing modes (CPU fallback for testing)

### Challenge 2: Hardware Abstraction for Camera Interfaces
**Problem**: Supporting multiple camera types (CSI native, USB fallback, virtual for testing).
**Solution**:
- Created flexible camera interface abstraction layer
- GStreamer pipelines optimized for Jetson hardware acceleration
- Automatic fallback mechanisms for development environments
- Unified configuration system across camera types

### Challenge 3: PointCloud2 to LaserScan Conversion for SLAM
**Problem**: Nav2 SLAM algorithms require LaserScan data, but stereo cameras produce PointCloud2.
**Solution** (In Progress):
- Designing efficient 3D-to-2D projection algorithm
- Calculating proper coordinate frame transformations
- Implementing noise filtering and quality validation
- Matching QoS settings across the perception pipeline

### Challenge 4: GPIO Motor Control in ROS2
**Problem**: Integrating low-level GPIO hardware control with ROS2 control framework.
**Solution**:
- Created libgpiod-based motor control library
- Integrated with ROS2 control interfaces
- Proper hardware abstraction for different GPIO controllers
- Safe shutdown and error handling mechanisms

## 👨‍💻 About the Developer

This project aims to gain experience in robotics software integration, ROS2 architecture, computer vision, and practical autonomous systems. By transforming a consumer robot kit into a fully ROS2-compatible platform with advanced capabilities, this work demonstrates the ability to bridge hardware and software in complex robotic systems, particularly on embedded platforms like the NVIDIA Jetson Orin Nano Super.

## 📝 License

This project is licensed under the GNU GPLv3 License - see the [LICENSE](LICENSE) file for details.

## 🙏 Acknowledgments

- [Waveshare](https://github.com/waveshare/JETANK) for creating and open-sourcing the JeTank AI Kit
- The ROS community for their extensive documentation and support
- NVIDIA for the Jetson Nano platform
- The Construct for great ROS guides

## Updates

- [06/11/2025 - PM]: Created jetank_description and jetank_simulation packages following ROS2 best practices. Refactored URDF files into dedicated description package. Implemented Gazebo simulation environment with basic world and launch files. Updated all dependencies and launch files for proper package integration.
- [06/11/2025 - AM]: Documentation updated. All packages reviewed and readme reflects current state.
- [30/10/2025]: Navigation package (jetank_navigation) in Phase 2 development - designing PointCloud2 to LaserScan conversion for SLAM integration.
- [14/10/2025]: Core packages complete and functional. Stereo perception system operational with point cloud generation. Navigation package created.
- [30/06/2025]: Initial package structure created, development environment configured.
- [29/06/2025]: Hardware upgrade to Jetson Orin Nano Super Developer Kit. Migration from ROS1 to ROS2 Humble initiated. 
