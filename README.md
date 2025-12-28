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