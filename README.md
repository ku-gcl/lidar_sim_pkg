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


## Reference
- [Gazebo sensors tutorial](https://gazebosim.org/docs/fortress/sensors/)
- [Gazebo sensors tutorial code](https://github.com/gazebosim/docs/tree/master/fortress/tutorials/sensors)
- [ros_gz](https://github.com/gazebosim/ros_gz?tab=readme-ov-file)
