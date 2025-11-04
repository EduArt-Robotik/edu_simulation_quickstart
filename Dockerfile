FROM osrf/ros:jazzy-desktop

ENV DEBIAN_FRONTEND=noninteractive

RUN apt update && apt install -y \
    xfce4 xfce4-terminal x11vnc xvfb novnc websockify supervisor dbus-x11 \
    sudo net-tools curl wget && \
    rm -rf /var/lib/apt/lists/*

# -------------------------------------------------------------------
# Custom Desktop Background for XFCE
# -------------------------------------------------------------------
# Eigenes Wallpaper als Standard-Hintergrund verwenden indem das Standard-Wallpaper ersetzt wird
COPY Docker-Background.svg /usr/share/backgrounds/xfce/xfce-shapes.svg


# User anlegen
RUN useradd -m ros && echo "ros:ros" | chpasswd && adduser ros sudo
USER ros
WORKDIR /home/ros
ENV HOME=/home/ros
ENV USER=ros

USER root
COPY supervisord.conf /etc/supervisor/conf.d/supervisord.conf



EXPOSE 8080
CMD ["/usr/bin/supervisord"]

RUN apt update \
    &&  apt install -y \
    tmux \
    git \
    openssh-client \
    gdb \
    build-essential

RUN apt-get update \
    && apt-get install -y \
        python3-pip \
        python3.12-venv

RUN bash -c "\
    mkdir /home/ros/python_env \
    && cd /home/ros/python_env \
    && python3 -m venv .flet \
    && source .flet/bin/activate \
    && pip3 install flet setuptools pyyaml \
    && pip install 'flet[all]==0.25.1' --upgrade"

# Configuration
RUN touch ~/.tmux.conf
RUN echo "set -g default-terminal \"screen-256color\"" >> ~/.tmux.conf
RUN echo "set -g mouse on" >> ~/.tmux.conf

# Source ROS files
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
RUN echo "source /home/ros/ros2_ws/install/setup.bash" >> ~/.bashrc

# Install EduRobot dependencies
RUN apt update \
    && apt install -y \
    ros-jazzy-hardware-interface \
    ros-jazzy-diagnostic-updater \
    ros-jazzy-hardware-interface \
    ros-jazzy-laser-geometry \
    ros-jazzy-gz-sim-vendor \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz \
    ros-jazzy-xacro \
    ros-jazzy-rviz2

RUN mkdir /home/ros/ros2_ws/src -p
WORKDIR /home/ros/ros2_ws

# Get EduArt repos
RUN bash -c "\
    source /opt/ros/jazzy/setup.bash \
    && git clone https://github.com/EduArt-Robotik/edu_robot.git src/edu_robot\
    && colcon build --symlink-install --packages-select edu_robot --event-handlers console_direct+"
    
# Get EduArt repos
RUN bash -c "\
    source /opt/ros/jazzy/setup.bash \
    && git clone https://github.com/EduArt-Robotik/edu_robot_control.git src/edu_robot_control\
    && colcon build --symlink-install --packages-select edu_robot_control --event-handlers console_direct+"

# Get EduArt repos
RUN bash -c "\
    source /opt/ros/jazzy/setup.bash \
    && git clone -b 0.3.0 https://github.com/EduArt-Robotik/edu_simulation.git src/edu_simulation\
    && colcon build --symlink-install --packages-select edu_simulation --event-handlers console_direct+"

# Get EduArt repos
RUN bash -c "\
    source /opt/ros/jazzy/setup.bash \
    && git clone -b develop https://github.com/EduArt-Robotik/edu_virtual_joy.git src/edu_virtual_joy\
    && colcon build --symlink-install --packages-select edu_virtual_joy --event-handlers console_direct+"

# Open virtual joystick in a window, not in the browser (doesn't work in dev container)
RUN sed -i 's\ft.app(target=main, view=ft.AppView.WEB_BROWSER, port=8888, assets_dir="assets")\ft.app(target=main, assets_dir="assets")\g' /home/ros/ros2_ws/src/edu_virtual_joy/edu_virtual_joy/edu_virtual_joy.py

# Set ROS varables
ENV ROS_DOMAIN_ID=0
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
#ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Set Python environment
ENV PYTHONPATH='/home/ros/python_env/.flet/lib/python3.12/site-packages'

# Enable color on command prompt
ENV TERM=xterm-256color
ENV color_prompt=yes

# Automatisch tmux starten und in 4 Fenster teilen
# Automatisch tmux starten und in 4 Fenster teilen
RUN cat << 'EOF' >> /home/ros/.bashrc
# Automatisch tmux starten und in 4 Fenster teilen
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
  tmux send-keys "ros2 launch edu_simulation eduard.launch.py wheel_type:=mecanum pos_x:=0.0 pos_y:=0.0 pos_z:=0.04 yaw:=0.0 edu_robot_namespace:=eduard/blue"

  # Pane 3 (unten rechts)
  tmux select-pane -t 3
  tmux select-pane -T "4. Start RViz"
  tmux send-keys "ros2 launch edu_simulation eduard_monitor.launch.py edu_robot_namespace:=eduard/blue"

  tmux select-pane -t 0
  tmux attach-session -t default
fi
EOF




