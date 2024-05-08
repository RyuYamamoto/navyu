# navyu
[![colcon-build](https://github.com/RyuYamamoto/navyu/actions/workflows/colcon-build.yml/badge.svg)](https://github.com/RyuYamamoto/navyu/actions/workflows/colcon-build.yml)

Original 2D Navigation Packages.  
**under development**

## Quick Run
### Docker
```bash
docker pull hazehk/navyu

xhost +
docker run -it --rm --net=host --env="DISPLAY=$DISPLAY" hazehk/navyu:latest
```

## How to Build
```
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/RyuYamamoto/navyu
git clone https://github.com/CIT-Autonomous-Robot-Lab/emcl2_ros2 # if use emcl2 package
cd ../
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Launch
Launch Gazebo Simulator
```bash
ros2 launch navyu_simulator navyu_simulator_bringup.launch.py
ros2 launch navyu_navigation navyu_bringup.launch.py localization:=true
```

https://github.com/RyuYamamoto/navyu/assets/6177252/22a092f1-14fc-4514-85c5-f63540dd98c3

Launch Simple Simulator
```bash
ros2 launch navyu_simulator navyu_simple_simulator_bringup.launch.py use_rviz:=false
ros2 launch navyu_navigation navyu_bringup.launch.py localization:=false
```

## Feature
- [x] Path Tracking
- [ ] Velocity Planning
- [ ] Velocity Smoothing
- [ ] Model Predictive Control Path Tracking
- [ ] 2D/3D Localization
- [ ] Local Path Planning
- [x] Collision Monitor
- [x] Add Obstacle Layer to Costmap
- [ ] 3D Costmap
