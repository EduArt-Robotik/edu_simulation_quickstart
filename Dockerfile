FROM osrf/ros:jazzy-desktop

ENV DEBIAN_FRONTEND=noninteractive

# -------------------------------------------------------------------
# Create custom user and configure the user settings
# -------------------------------------------------------------------
RUN useradd -m user -s /bin/bash && echo "user:user" | chpasswd && adduser user sudo

# -------------------------------------------------------------------
# Install dependencies (as root)
# -------------------------------------------------------------------
USER root

RUN apt update && apt install -y \
    sudo tmux git nano gdb build-essential net-tools openssh-client software-properties-common swig 

RUN apt-get update && apt-get install -y \
    python3-pip python3.12-venv python3-pip

RUN apt update && apt install -y \
    xfce4 xfce4-terminal x11vnc xvfb novnc websockify supervisor dbus-x11 curl wget mousepad

# Install GPIO MRAA lib for edu_robot_control_template
# Build and install MRAA from source
RUN git clone https://github.com/eclipse/mraa.git /opt/mraa \
    && cd /opt/mraa && mkdir build && cd build \
    && cmake .. -DBUILDSWIGPYTHON=ON \
    && make -j$(nproc) && make install \
    && ldconfig \
    && rm -rf /opt/mraa

# Install edu_robot dependencies
RUN apt update \
    && apt install -y \
    ros-jazzy-rmw-cyclonedds-cpp \
    ros-jazzy-hardware-interface \
    ros-jazzy-diagnostic-updater \
    ros-jazzy-hardware-interface \
    ros-jazzy-laser-geometry \
    ros-jazzy-gz-sim-vendor \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz \
    ros-jazzy-xacro \
    ros-jazzy-rviz2 \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup

# -------------------------------------------------------------------
# Switch to user to set up the workspace
# -------------------------------------------------------------------
USER user

# -------------------------------------------------------------------
# Install packages for simulation
# -------------------------------------------------------------------
RUN mkdir -p /home/user/ros2_ws/src
WORKDIR /home/user/ros2_ws

# Get edu_robot package
RUN bash -c "\
    source /opt/ros/jazzy/setup.bash \
    && git clone https://github.com/EduArt-Robotik/edu_robot.git src/edu_robot \
    && colcon build --symlink-install --packages-select edu_robot --event-handlers console_direct+"

# Get edu_robot_control package
RUN bash -c "\
    source /opt/ros/jazzy/setup.bash \
    && git clone https://github.com/EduArt-Robotik/edu_robot_control.git src/edu_robot_control \
    && colcon build --symlink-install --packages-select edu_robot_control --event-handlers console_direct+"

# Get edu_robot_control_template package
RUN bash -c "\
    source /opt/ros/jazzy/setup.bash \
    && git clone -b master https://github.com/EduArt-Robotik/edu_robot_control_template.git src/edu_robot_control_template \
    && colcon build --symlink-install --packages-select edu_robot_control_template --event-handlers console_direct+"

# Get edu_simulation package
RUN bash -c "\
    source /opt/ros/jazzy/setup.bash \
    && git clone -b atWork-sim https://github.com/EduArt-Robotik/edu_simulation.git src/edu_simulation \
    && colcon build --symlink-install --packages-select edu_simulation --event-handlers console_direct+"

# Get edu_virtual_joy package
RUN bash -c "\
    source /opt/ros/jazzy/setup.bash \
    && git clone -b develop https://github.com/EduArt-Robotik/edu_virtual_joy.git src/edu_virtual_joy \
    && colcon build --symlink-install --packages-select edu_virtual_joy --event-handlers console_direct+"

# Open virtual joystick in a window, not in the browser
RUN sed -i 's\ft.app(target=main, view=ft.AppView.WEB_BROWSER, port=8888, assets_dir="assets")\ft.app(target=main, assets_dir="assets")\g' \
    /home/user/ros2_ws/src/edu_virtual_joy/edu_virtual_joy/edu_virtual_joy.py

# -------------------------------------------------------------------
# Create virtual environments
# -------------------------------------------------------------------
RUN bash -c "\
    mkdir -p /home/user/python_env \
    && cd /home/user/python_env \
    && python3 -m venv .flet \
    && source .flet/bin/activate \
    && pip3 install flet setuptools pyyaml \
    && pip install 'flet[all]==0.25.1' --upgrade"

# -------------------------------------------------------------------
# Copy scripts in the container
# -------------------------------------------------------------------
# Using own background image for XFCE by replacing the default background image.
COPY docker_setup/Docker-Background.svg /usr/share/backgrounds/xfce/xfce-shapes.svg

# VNC setup
RUN mkdir -p /home/user/supervisor/logs /home/user/supervisor/run
COPY --chown=user:user docker_setup/supervisord.conf /home/user/supervisor/supervisord.conf

# Copy start scripts for the simulation
COPY --chmod=755 docker_setup/start-simulation.sh /usr/local/bin/start-simulation.sh
COPY --chmod=755 docker_setup/start-navigation.sh /usr/local/bin/start-navigation.sh

# -------------------------------------------------------------------
# Configure the user space
# -------------------------------------------------------------------
WORKDIR /home/user

# Configure tmux
RUN touch ~/.tmux.conf
RUN echo "set -g default-terminal \"screen-256color\"" >> ~/.tmux.conf
RUN echo "set -g mouse on" >> ~/.tmux.conf

# Source ROS files
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
RUN echo "source /home/user/ros2_ws/install/setup.bash" >> ~/.bashrc

# -------------------------------------------------------------------
# Environment variables
# -------------------------------------------------------------------
# Configure user-space
ENV HOME=/home/user
ENV USER=user

# Set Python environment
# ENV PYTHONPATH='/home/user/python_env/.flet/lib/python3.12/site-packages'

# Enable color on command prompt
ENV TERM=xterm-256color
ENV color_prompt=yes



# -------------------------------------------------------------------
# Create useful terminal commands cheat sheet on Desktop
# -------------------------------------------------------------------
RUN mkdir -p /home/user/Desktop && cat > /home/user/Desktop/useful_terminal_commands.txt <<'EOF'
# START SIMULATION

start-simulation.sh

# START NAVIGATION

start-navigation.sh

# UPDATE EDU SIMULATION REPO

cd ~/ros2_ws/src/edu_simulation && git pull

# BUILD EDU SIMULATION REPO AGAIN

cd ~/ros2_ws && source /opt/ros/jazzy/setup.bash && colcon build --symlink-install --packages-select edu_simulation --event-handlers console_direct+

# START GAZEBO SIM WITH MAZE WORLD

ros2 launch edu_simulation gazebo.launch.py world:=maze.world

# START VIRTUAL JOY

PYTHONPATH=/home/user/python_env/.flet/lib/python3.12/site-packages:$PYTHONPATH ros2 run edu_virtual_joy virtual_joy --ros-args -r __ns:=/eduard_atwork

# LAUNCH EDUARD ATWORK IN SIMULATION

ros2 launch edu_simulation eduard_atwork.launch.py pos_x:=0.0 pos_y:=0.0 pos_z:=0.04 yaw:=0.0

# START RVIZ / MONITOR

ros2 launch edu_simulation eduard_monitor.launch.py
EOF



