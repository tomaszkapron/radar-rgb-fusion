# radar-rgb-fusion
Data fusion between radar data and detected features on rgb image built in ROS2.

## Dependencies
- rocker

## Install

build docker image end run container
```
cd .devcontainer
cd ..
./run.sh
```

Inside the container update repositories clone dependencies and build workspace
```
sudo apt-get update
rosdep update
vcs import . < default.repos
rosdep install --from-paths src --ignore-src -yr
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

### Radar network setup
```
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up
```
