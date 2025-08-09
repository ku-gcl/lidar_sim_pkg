# LiDAR simulation
## Preliminaries
- ROS 2 Humble
- Gazebo Fortress (ignition)

## How to use
### Clone and build

```bash
mkdir -p ~/ws_lidar/src
cd ~/ws_lidar/src

git clone https://github.com/ku-gcl/lidar_sim_pkg.git

cd ~/ws_lidar
colcon build --symlink-install
```


### simulation

```bash
source /opt/ros/humble/setup.bash
source ~/ws_lidar/install/setup.bash

ros2 launch lidar_sim_pkg simulation.launch.py
```

### Check topic

```bash
ign topic -l                # see topic list
ign topic -e -t /lidar      # echo /lidar topic data
```