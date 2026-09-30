# Lightweight 2D LiDAR odometry

`lidar_2d_odometry` is a standalone ROS 2 scan-to-local-map odometry node. It
does not link Cartographer, Ceres, PCL, or RF2O. Its only non-ROS numerical
dependency is Eigen.

The matcher uses a constant-velocity initial estimate, point-to-line ICP with a
Huber loss, and a bounded keyframe map. Laser points are transformed into the
configured tracking frame through TF, so the laser mounting pose is not baked
into the implementation.

## Build and run

```bash
cd ~/jdbot_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select lidar_2d_odometry
source install/setup.bash
ros2 launch lidar_2d_odometry lidar_2d_odometry.launch.py
```

可在 launch 命令中覆盖坐标系，例如：

```bash
ros2 launch lidar_2d_odometry lidar_2d_odometry.launch.py \
  tracking_frame:=base_link odom_frame:=laser_odom
```

Input is `/scan` (`sensor_msgs/msg/LaserScan`). Output is `/odom`
(`nav_msgs/msg/Odometry`) and, unless disabled, `odom -> base_footprint` TF.

When replaying the supplied acceptance bag, avoid colliding with its recorded
odometry and TF:

```bash
ros2 launch lidar_2d_odometry lidar_2d_odometry.launch.py \
  publish_tf:=false odom_topic:=/lidar_odom
ros2 bag play ~/carto_odom_test_data
```

All matcher limits are parameters in `config/lidar_2d_odometry.yaml`. The local
map is deliberately bounded by `max_keyframes`; memory and nearest-neighbour
query cost therefore remain bounded during long runs.

## Acceptance result

The complete `~/carto_odom_test_data` bag was replayed at 4x speed on the same
host for both packages. Each scan pose was timestamp-interpolated against the
recorded `/odom` trajectory and the trajectories were rigidly aligned in 2D.
Distances below are position errors.

| implementation | mean | RMSE | P95 | maximum |
| --- | ---: | ---: | ---: | ---: |
| `cartographer_lidar_odometry` | 0.77 cm | 0.89 cm | 1.54 cm | 3.05 cm |
| `lidar_2d_odometry` | 2.22 cm | 2.41 cm | 3.82 cm | 5.51 cm |
| increase | **1.45 cm** | **1.52 cm** | **2.28 cm** | **2.45 cm** |

All measured error increases are below 3 cm. Both runs produced one pose per
accepted scan (896 Cartographer poses after its initializer, 897 lightweight
poses). GNU `time -v` measured 2.86 seconds of CPU time and 24.4 MiB peak RSS
for this package, versus 5.65 seconds and 43.0 MiB for Cartographer. The test
machine, ROS installation, bag, playback rate, and process wrapper were kept
the same. These figures are acceptance-bag results, not a general accuracy
guarantee for different sensors or environments.
