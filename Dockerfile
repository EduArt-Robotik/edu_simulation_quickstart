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
    tmux git openssh-client gdb build-essential software-properties-common swig

RUN apt-get update && apt-get install -y \
    python3-pip python3.12-venv python3-pip

RUN apt update && apt install -y \
    xfce4 xfce4-terminal x11vnc xvfb novnc websockify supervisor dbus-x11 \
    sudo net-tools curl wget

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
    ros-jazzy-rviz2

# -------------------------------------------------------------------
# Set up user workspace
# -------------------------------------------------------------------
USER user
WORKDIR /home/user

# Set up directories
RUN mkdir -p /home/user/python_env \
    &&  \
    && 

# Create virtual environment with python modules for edu_virtual_joy
RUN bash -c "\
    cd /home/user/python_env \
    && python3 -m venv .flet \
    && source .flet/bin/activate \
    && pip3 install flet setuptools pyyaml \
    && pip install 'flet[all]==0.25.1' --upgrade"

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
# Copy scripts in the container
# -------------------------------------------------------------------

# Using own background image for XFCE by replacing the default background image.
COPY Docker-Background.svg /usr/share/backgrounds/xfce/xfce-shapes.svg

# VNC setup
RUN mkdir -p /home/user/supervisor/logs /home/user/supervisor/run
COPY --chown=user:user supervisord.conf /home/user/supervisor/supervisord.conf

# Copy script to start the simulation
COPY start-simulation.sh /usr/local/bin/start-simulation.sh
RUN chmod +x /usr/local/bin/start-simulation.sh

# Make sure home content stays owned by user after root copies
# RUN chown -R user:user /home/user

# -------------------------------------------------------------------
# Configure the user space
# -------------------------------------------------------------------
WORKDIR /home/user
ENV HOME=/home/user
ENV USER=user

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
# Set Python environment
ENV PYTHONPATH='/home/user/python_env/.flet/lib/python3.12/site-packages'

# Enable color on command prompt
ENV TERM=xterm-256color
ENV color_prompt=yes
