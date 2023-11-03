# Slam Repository for TWSR FYP

## 1.Build

### 1.1 Install Opencv and Ros2 humble

detailed guidelines can be found in our RV2024 repo - [RV2024](https://github.com/hkustenterprize/RM2023-RV)

### 1.2 Install ORB-SLAM3

follow this websit:
[https://github.com/UZ-SLAMLab/ORB_SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3)

a. install [Pangolin](https://github.com/UZ-SLAMLab/ORB_SLAM3#pangolin)

b. install [Eigen3](https://github.com/UZ-SLAMLab/ORB_SLAM3#eigen3)

c. build as instructions:

```
git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git 
ORB_SLAM3cd ORB_SLAM3
chmod +x build.sh
./build.sh
```

notes: change the cmake file to c++14 and 11 accordingly and change all monotonic_clock to steady_clock in the ORB_SLAM3 repo

### 1.3 Install Livox-SDK2

```bash
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd ./Livox-SDK2/
mkdir build
cd build
cmake .. && make -j
sudo make install
```

The official viewer download: (choose the Livox Viewer 2 - Ubuntu version)

https://www.livoxtech.com/mid-360/downloads

do sudo ifconfig enx207bd28d25c4 192.168.1.50 to set the correct ip

### 1.4 Clone this repo and build

```bash
git clone --recursive git@github.com:FYP-TWSR/fyp-slam.git
cd ./fly-slam/
```

```
## add a file called package.xml to the livox_driver folder ##

<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>livox_ros_driver2</name>
  <version>1.0.0</version>
  <description>The ROS device driver for Livox 3D LiDARs, for ROS2</description>
  <maintainer email="dev@livoxtech.com">feng</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_cmake_auto</buildtool_depend>
  <build_depend>rosidl_default_generators</build_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>

  <depend>rclcpp</depend>
  <depend>rclcpp_components</depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>rcutils</depend>
  <depend>pcl_conversions</depend>
  <depend>rcl_interfaces</depend>
  <depend>libpcl-all-dev</depend>

  <exec_depend>rosbag2</exec_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <depend>git</depend>
  <depend>apr</depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

```
## install the pcl package for fast-lio package ##
sudo apt install ros-humble-pcl-ros

## then it is ok to build ##
colcon build --symlink-install
```
