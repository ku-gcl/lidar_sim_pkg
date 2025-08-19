# LiDAR simulation
## Preliminaries
- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Harmonic


## YouTube Video
[![YouTube](doc/img/thumbnail.png)](https://youtu.be/BaE7Gy0TvX8)


## How to use
### Clone and build

```bash
# Install dependencies
sudo apt install -y ros-humble-actuator-msgs ros-humble-gps-msgs ros-humble-vision-msgs

export GZ_VERSION=harmonic

mkdir -p ~/ws_lidar/src
cd ~/ws_lidar/src

# Download needed software
git clone https://github.com/gazebosim/ros_gz.git -b humble
cd ~/ws_lidar
rosdep install -r --from-paths src -i -y --rosdistro humble

# Build and install into workspace
source /opt/ros/humble/setup.bash
cd ~/ws_lidar
colcon build --symlink-install
```

### Bridge gz gpu_lidar to ROS 2

```bash
cd ~/ws_lidar/src
git clone https://github.com/ku-gcl/lidar_sim_pkg
cd ~/ws_lidar
export GZ_VERSION=harmonic && colcon build --symlink-install
```

### Run

```bash
echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
echo "source ~/ws_lidar/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Then, run your gazebo simulation. 
After run gazebo simulation, execute command below

```bash
ros2 launch lidar_sim_pkg pc2_bridge.launch.py
```

```bash
gz topic -t /world/helipad_lidar/model/wrscopter_lidar_0/link/mid360_link/sensor/lidar/scan/points --info
ros2 run ros_gz_bridge parameter_bridge /world/helipad_lidar/model/wrscopter_lidar_0/link/mid360_link/sensor/lidar/scan/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked
```




## Livox Mid-360 Specs
|  # | Item                        | Value                                                       |
| -: | --------------------------- | ----------------------------------------------------------- |
|  1 | Model                       | MID-360                                                     |
|  2 | Laser Wavelength            | 905 nm                                                      |
|  3 | Laser Safety                | Class 1 (IEC 60825-1:2014)                                  |
|  4 | Detection Range (@100 klx)  | **40 m @ 10% reflectivity** / 70 m @ 80% reflectivity           |
|  5 | Close Proximity Blind Zone  | 0.1 m                                                       |
|  6 | Field of View (FOV)         | **Horizontal 360° / Vertical −7° to +52° (59° total)**          |
|  7 | Range Precision (1σ)        | ≤ 2 cm (@10 m) / ≤ 3 cm (@0.2 m)                            |
|  8 | Angular Precision (1σ)      | < 0.15°                                                     |
|  9 | Point Rate                  | **200,000 points/s (first return)**                             |
| 10 | Frame Rate                  | **10 Hz (typical)**                                             |
| 11 | Data Port                   | 100BASE-TX Ethernet                                         |
| 12 | Time Sync                   | IEEE 1588-2008 (PTPv2), GPS                                 |
| 13 | Anti-Interference Function  | Available                                                   |
| 14 | False Alarm Rate (@100 klx) | < 0.01%                                                     |
| 15 | IMU                         | Built-in ICM40609                                           |
| 16 | Operating Temperature       | −20°C to 55°C (−4°F to 131°F)                               |
| 17 | IP Rating                   | IP67                                                        |
| 18 | Power (avg.)                | 6.5 W (may peak to \~14 W in self-heating mode at low temp) |
| 19 | Power Supply Voltage        | 9–27 V DC                                                   |
| 20 | Dimensions                  | 65 × 65 × 60 mm                                             |
| 21 | Weight                      | 265 g                                                       |


You can configure the LiDAR by editing [worlds/sensor_tutorial.sdf](worlds/sensor_tutorial.sdf).
Below are the key LiDAR settings. Adjust to match your device.

- update_rate
- horizontal.samples
- horizontal.min_angle
- horizontal.max_angle
- vertical.samples
- vertical.min_angle
- vertical.max_angle
- range.min
- range.max

> [!NOTE]
> The real Livox Mid-360 uses a **non-reptitive scanning pattern** that fills its FOV over time rather than a fixed grid per frame.
> For simplify in this simulation, we approximate it with a uniform grid: 
> **2000 horizontal samples x 10 vertical samples at `update_rate` = 10 Hz**, which yields **≈200,000 points/s**.


```sdf
<sensor name='gpu_lidar' type='gpu_lidar'>
    <pose relative_to='lidar_frame'>0 0 0.2 0 0 0</pose>
    <topic>lidar</topic>
    <update_rate>10</update_rate>
    <ray>
        <scan>
            <horizontal>
                <samples>2000</samples>
                <resolution>1</resolution>
                <min_angle>-3.1415926536</min_angle>
                <max_angle> 3.1415926536</max_angle>
            </horizontal>

            <vertical>
                <samples>10</samples>
                <resolution>1</resolution>
                <min_angle>-0.122173048</min_angle>
                <max_angle> 0.907571211</max_angle>
            </vertical>
        </scan>

        <range>
            <min>0.10</min>
            <max>10.0</max>
            <resolution>0.01</resolution>
        </range>
        <noise>
            <type>gaussian</type>
            <mean>0.0</mean>
            <stddev>0.02</stddev>
        </noise>
    </ray>
    <always_on>1</always_on>
    <visualize>true</visualize>
</sensor>
```


## Reference
### Gazebo
- [Gazebo sensors tutorial](https://gazebosim.org/docs/fortress/sensors/)
- [Gazebo sensors tutorial code](https://github.com/gazebosim/docs/tree/master/fortress/tutorials/sensors)
- [ros_gz](https://github.com/gazebosim/ros_gz?tab=readme-ov-file)

### Livox Mid-360
- [Livox Mid-360 Specs](https://www.livoxtech.com/jp/mid-360/specs)

