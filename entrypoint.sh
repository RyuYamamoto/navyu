#!/bin/bash

tmux new-session -d;

tmux send-keys 'source install/setup.bash && ros2 launch navyu_simulator navyu_simulator_bringup.launch.py use_rviz:=false' ENTER;

tmux split-window -h;

tmux send-keys 'source install/setup.bash && ros2 launch navyu_navigation navyu_bringup.launch.py localization:=true' ENTER;
tmux attach-session
