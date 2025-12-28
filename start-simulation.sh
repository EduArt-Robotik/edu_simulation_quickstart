if [ -z "$TMUX" ]; then
  tmux new-session -d -s default
  tmux split-window -h
  tmux split-window -v
  tmux select-pane -L
  tmux split-window -v

  # Pane 0 (oben links)
  tmux select-pane -t 0
  tmux select-pane -T "1. Start Gazebo Sim"
  tmux send-keys "ros2 launch edu_simulation gazebo.launch.py world:=maze.world" C-m

  # Pane 1 (oben rechts)
  tmux select-pane -t 1
  tmux select-pane -T "2. Start Virtual Joy"
  tmux send-keys "ros2 run edu_virtual_joy virtual_joy --ros-args -r __ns:=/eduard/blue" C-m

  # Pane 2 (unten links)
  tmux select-pane -t 2
  tmux select-pane -T "3. Add Eduard"
  tmux send-keys "COLOR=red
sleep 5 && ros2 launch edu_simulation eduard.launch.py \
  edu_robot_namespace:=eduard/${COLOR} \
  wheel_type:=mecanum \
  eduard_color:=${COLOR} \
  pos_x:=0.0 pos_y:=0.0 pos_z:=0.04 yaw:=0.0" C-m

  # Pane 3 (unten rechts)
  tmux select-pane -t 3
  tmux select-pane -T "4. Start RViz"
  tmux send-keys "ros2 launch edu_simulation eduard_monitor.launch.py edu_robot_namespace:=eduard/blue" C-m

  tmux select-pane -t 0
  tmux attach-session -t default
fi
