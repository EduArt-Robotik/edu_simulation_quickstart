if [ -z "$TMUX" ]; then
  tmux new-session -d -s navigation
  tmux split-window -h
  tmux split-window -v
  tmux select-pane -L
  tmux split-window -v

  # Pane 0 (top left)
  tmux select-pane -t 0
  tmux select-pane -T "1. Start mapping"
  tmux send-keys "EDU_ROBOT_NAMESPACE=eduard/blue ros2 launch /home/user/workspace/ros2_ws/src/navigation/launch/localization.launch.py"

  # Pane 1 (bottom left)
  tmux select-pane -t 1
  tmux select-pane -T "3. Start localization"
  tmux send-keys "EDU_ROBOT_NAMESPACE=eduard/blue MAP_FILE=/home/user/workspace/maps/simulation_maze/map.yaml ros2 launch /home/user/workspace/ros2_ws/src/navigation/launch/localization.launch.py use_sim_time:=True"

  # Pane 2 (top right)
  tmux select-pane -t 2
  tmux select-pane -T "2. Save map"
  tmux send-keys "export MAPFILE=test && mkdir /home/user/workspace/maps/\${MAPFILE} && ros2 service call /eduard/blue/slam_toolbox/save_map slam_toolbox/srv/SaveMap \"{name: {data: '/home/user/workspace/maps/\${MAPFILE}/map'}}\""
 
  # Pane 3 (bottom right)
  tmux select-pane -t 3
  tmux select-pane -T "3. Start localization"
  tmux send-keys "EDU_ROBOT_NAMESPACE=eduard/blue ros2 launch /home/user/workspace/ros2_ws/src/navigation/launch/navigation.launch.py use_sim_time:=True"

  tmux select-pane -t 0
  tmux attach-session -t navigation
fi
