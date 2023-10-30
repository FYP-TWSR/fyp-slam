# Slam Repository for TWSR FYP
## 1.Build
### 1.1 Install ORB-SLAM3
follow this websit:
<https://github.com/UZ-SLAMLab/ORB_SLAM3>
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