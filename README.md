# Slam Repository for TWSR FYP
## 1.Build
### 1.1 Install ORB-SLAM3
follow this websit:
<https://github.com/UZ-SLAMLab/ORB_SLAM3>

use the C++14 branch or the version I modified:
<https://github.com/HUANG-Haolun/ORB_SLAM3.git>
### 1.2 Install Livox-SDK2
```bash
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd ./Livox-SDK2/
mkdir build
cd build
cmake .. && make -j
sudo make install
```
### 1.3 Clone this repo
```bash
git clone --recursive git@github.com:FYP-TWSR/fyp-slam.git
colcon build --symlink-install
```


sudo ifconfig enxc84d442988e6 192.168.1.50
