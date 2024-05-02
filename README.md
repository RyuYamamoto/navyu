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
```bash
ros2 launch navyu_simulator navyu_simulator_bringup.launch.py
ros2 launch navyu_navigation navyu_bringup.launch.py localization:=true
```

https://github.com/RyuYamamoto/navyu/assets/6177252/a620db9e-a79a-47b1-a69f-bd70c8a2c020

```bash
ros2 launch navyu_simulator navyu_simple_simulator_bringup.launch.py use_rviz:=false
ros2 launch navyu_navigation navyu_bringup.launch.py localization:=false
```

### Navigation 2 Demo
```bash
ros2 launch navyu_navigation navigation2_bringup.launch.py
```
[![](https://img.youtube.com/vi/V2hUBr7PJto/0.jpg)](https://www.youtube.com/watch?v=V2hUBr7PJto)

### Simple Simulator Demo
```
ros2 launch navyu_simulator navyu_simple_simulator_bringup.launch.py
```
![image](https://github.com/RyuYamamoto/navyu/assets/6177252/9d6984f6-edd8-4c10-a049-9d9f2b11834b)

## Feature
- [ ] Path Tracking
- [ ] 2D/3D Localization
- [ ] Local Path Planning
- [ ] Collision Monitor
- [ ] Add Obstacle Layer to Costmap
