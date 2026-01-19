if [ -z "$TMUX" ]; then
  tmux new-session -d -s simulation
  tmux split-window -h
  tmux split-window -v
  tmux select-pane -L
  tmux split-window -v

  # Pane 0 (top left)
  tmux select-pane -t 0
  tmux select-pane -T "1. Start Gazebo Sim"
  tmux send-keys "ros2 launch edu_simulation gazebo.launch.py world:=maze.world" C-m

  # Pane 1 (bottom left)
  tmux select-pane -t 1
  tmux select-pane -T "2. Start Virtual Joy"
  tmux send-keys "PYTHONPATH=/home/user/python_env/.flet/lib/python3.12/site-packages:$PYTHONPATH ros2 run edu_virtual_joy virtual_joy --ros-args -r __ns:=/eduard/green" C-m

  # Pane 2 (top right)
  tmux select-pane -t 2
  tmux select-pane -T "3. Add Eduard"
  tmux send-keys "sleep 5 && \
  ros2 launch edu_simulation eduard.launch.py \
  edu_robot_namespace:=eduard/green \
  wheel_type:=mecanum \
  eduard_color:=green \
  pos_x:=0.0 pos_y:=0.0 pos_z:=0.04 yaw:=0.0" C-m

  # Pane 3 (bottom right)
  tmux select-pane -t 3
  tmux select-pane -T "4. Start RViz"
  tmux send-keys "ros2 launch /home/user/workspace/ros2_ws/src/simulation/launch/robot_monitor.launch.py edu_robot_namespace:=eduard/green" C-m

  tmux select-pane -t 0
  tmux attach-session -t simulation
fi
