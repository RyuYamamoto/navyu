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

### Launch 
```bash
ros2 launch navyu_simulator navyu_simple_simulator_bringup.launch.py
ros2 launch navyu_costmap_2d navyu_costmap_2d.launch.py
ros2 launch navyu_path_planner navyu_global_planner.launch.py
```

![rviz_screenshot_2024_04_29-12_34_14](https://github.com/RyuYamamoto/navyu/assets/6177252/6aea86e5-2e57-44f4-98f6-69d3431346db)

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
