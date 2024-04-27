#!/bin/bash

tmux new-session -d;

tmux send-keys 'source install/setup.bash && ros2 launch navyu_simulator navyu_simulator_bringup.launch.py use_rviz:=false' ENTER;

tmux split-window -h;

tmux send-keys 'source install/setup.bash' ENTER;
tmux send-keys ' ros2 launch navyu_navigation navigation2_bringup.launch.py'
tmux attach-session
