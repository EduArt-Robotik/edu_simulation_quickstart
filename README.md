# Installation
## Building Container
- depending on used host system 3-20min

```
docker build --platform=linux/amd64 -t ros2-vnc .
```

oder ohne cache:

```
docker build --no-cache --platform=linux/amd64 -t ros2-vnc .
```

## Running with Quickstart Skript
```
docker compose -f docker-compose.yml up
```

if container already exists:
```
docker compose -f docker-compose.yml down
docker compose -f docker-compose.yml up
```

### Running in Browser without VNC Viewer without quickstart script
- copy + paste from and to host system does not work

```
docker run --platform=linux/amd64 -p 8080:8080 --shm-size=2g --name ros2-vnc-test ros2-vnc
```

### Running in Browser and / or VNC Viewer without quickstart script

```
docker run -it \
  -p 8080:8080 \
  -p 5900:5900 \
  --shm-size=2g \
  --name ros2-vnc-test \
  ros2-vnc
```

# Troubleshooting
- ros2-vnc-test already exists

```
docker rm -f ros2-vnc-test
```

Oder
```
docker rm -f ros2-vnc
```

# Create new nodes
```
cd /home/user/workspace/ros2_ws/src
ros2 pkg create --build-type ament_python edu_template --license MIT --node-name dont_hit_the_wall_node
cd ..
colcon build --symlink-install --packages-select edu_template --event-handlers console_direct+
```

# Navigation

Run the localization without a known map -> Mapping (SLAM)
```
EDU_ROBOT_NAMESPACE=eduard/blue ros2 launch ~/workspace/ros2_ws/src/navigation/launch/localization.launch.py
```

Save the current map to a file
```
ros2 service call /eduard/blue/slam_toolbox/save_map slam_toolbox/srv/SaveMap "name: data: '/home/user/workspace/maps/simulation_maze/map'"
```

Run the localization with a known map
```
EDU_ROBOT_NAMESPACE=eduard/blue MAP_FILE=/home/user/workspace/maps/simulation_maze/map.yaml ros2 launch ~/workspace/ros2_ws/src/navigation/launch/localization.launch.py use_sim_time:=True
```

Run the navigation. Works best with a known map.
```
EDU_ROBOT_NAMESPACE=eduard/blue ros2 launch ~/workspace/ros2_ws/src/navigation/launch/navigation.launch.py use_sim_time:=True
```