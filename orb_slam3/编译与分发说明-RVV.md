# ORB-SLAM3 RGB-D 编译与分发说明（ROS 2 Humble / RISC-V RVV）

本文对应已在 SpacemiT RISC-V 主机验证过的组合：ROS 2 Humble、
`opencv-spacemit=4.14.0-2bb4`、Eigen 3.4、Pangolin v0.9.4，以及
ORB-SLAM3 `4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4`。

构建全程在独立目录中进行，最终运行与开发文件统一安装到
`/opt/orbslam3`，不依赖构建用户的 home 目录。本文使用的 RVV 选项是：

```bash
-march=rv64gcv_zvl256b -mrvv-vector-bits=zvl
-DEIGEN_RISCV64_USE_RVV10
```

其中 `-mrvv-vector-bits=zvl` 不能省略：系统 Eigen 开启
`EIGEN_RISCV64_USE_RVV10` 后会检查该编译选项。

## 1. 准备依赖和独立构建目录

```bash
sudo apt update
sudo apt install -y git cmake ninja-build build-essential patchelf \
  libeigen3-dev opencv-spacemit=4.14.0-2bb4 \
  libboost-serialization-dev libssl-dev libsuitesparse-dev \
  libglew-dev libgl1-mesa-dev libepoxy-dev libx11-dev libwayland-dev \
  libjpeg-dev libpng-dev libtiff-dev

sudo apt install -y ros-humble-cv-bridge ros-humble-message-filters \
  ros-humble-tf2-ros

export BUILD_ROOT="$HOME/orbslam3-rvv-$(date +%Y%m%d)"
export STAGE="$BUILD_ROOT/stage"
mkdir -p "$BUILD_ROOT" "$STAGE"

export RVV_C_FLAGS="-O3 -DNDEBUG -march=rv64gcv_zvl256b -mrvv-vector-bits=zvl"
export RVV_CXX_FLAGS="$RVV_C_FLAGS -DEIGEN_RISCV64_USE_RVV10"
export OPENCV_PREFIX=/opt/opencv-spacemit
export OPENCV_DIR="$OPENCV_PREFIX/lib/cmake/opencv4"
```

不要把 ORB-SLAM3 配置回发行版的 `/usr` OpenCV。某些 ROS deb 包会使
`libopencv-dev` 间接保留在系统中，这不影响构建，但所有 OpenCV 消费方都应
显式指定上面的 `OpenCV_DIR`。内存为 8 GiB 的机器建议准备交换空间并使用
`-j4`，主库并行度过高可能触发 OOM。

## 2. 编译并暂存 Pangolin

```bash
cd "$BUILD_ROOT"
git clone --branch v0.9.4 --depth 1 \
  https://github.com/stevenlovegrove/Pangolin.git

cmake -S Pangolin -B Pangolin/build -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$STAGE" \
  -DCMAKE_C_FLAGS_RELEASE="$RVV_C_FLAGS" \
  -DCMAKE_CXX_FLAGS_RELEASE="$RVV_CXX_FLAGS" \
  -DOpenCV_DIR="$OPENCV_DIR" \
  -DBUILD_EXAMPLES=OFF -DBUILD_TOOLS=OFF -DBUILD_PANGOLIN_PYTHON=OFF
cmake --build Pangolin/build -j4
cmake --install Pangolin/build
```

## 3. 编译 ORB-SLAM3

```bash
cd "$BUILD_ROOT"
git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git
cd ORB_SLAM3
git checkout 4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4
git apply /path/to/orbslam3_ros2/patches/orbslam3-riscv64.patch
```

补丁完成四项兼容处理：在 RISC-V 上移除 `-march=native`；主工程改用
C++17；DBoW2/g2o 同样移除 `-march=native`；把 `mnFullBAIdx` 修正为其实际
使用所需的整数类型。

依次编译第三方库和主库。不要使用源码自带的 `build.sh`，否则容易遗漏 RVV
和 SpacemiT OpenCV 的显式配置。

```bash
cmake -S Thirdparty/DBoW2 -B Thirdparty/DBoW2/build -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_C_FLAGS_RELEASE="$RVV_C_FLAGS" \
  -DCMAKE_CXX_FLAGS_RELEASE="$RVV_CXX_FLAGS" \
  -DCMAKE_POLICY_VERSION_MINIMUM=3.5
cmake --build Thirdparty/DBoW2/build -j4

cmake -S Thirdparty/g2o -B Thirdparty/g2o/build -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_C_FLAGS_RELEASE="$RVV_C_FLAGS" \
  -DCMAKE_CXX_FLAGS_RELEASE="$RVV_CXX_FLAGS" \
  -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
  -DG2O_EIGEN3_INCLUDE=/usr/include/eigen3
cmake --build Thirdparty/g2o/build -j4

cmake -S . -B build -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_C_FLAGS_RELEASE="$RVV_C_FLAGS" \
  -DCMAKE_CXX_FLAGS_RELEASE="$RVV_CXX_FLAGS" \
  -DCMAKE_PREFIX_PATH="$STAGE;$OPENCV_PREFIX" \
  -DPangolin_DIR="$STAGE/lib/cmake/Pangolin" \
  -DOpenCV_DIR="$OPENCV_DIR" \
  -DCMAKE_POLICY_VERSION_MINIMUM=3.5
cmake --build build -j4
```

配置结束时应显示 `Found OpenCV: /opt/opencv-spacemit (found version
"4.14.0")`。也可以检查缓存：

```bash
grep -E '^(OpenCV_DIR|Pangolin_DIR):' build/CMakeCache.txt
```

## 4. 整理 `/opt/orbslam3` 分发目录

先将 ORB-SLAM3 文件放入暂存目录：

```bash
cd "$BUILD_ROOT/ORB_SLAM3"
install -d "$STAGE"/{lib,include/orbslam3,share/orbslam3}
cp -a lib/libORB_SLAM3.so Thirdparty/DBoW2/lib/libDBoW2.so \
  Thirdparty/g2o/lib/libg2o.so "$STAGE/lib/"
cp -a include/. "$STAGE/include/orbslam3/"

install -d "$STAGE/include/orbslam3/Thirdparty"/{Sophus,DBoW2,g2o}
cp -a Thirdparty/Sophus/sophus \
  "$STAGE/include/orbslam3/Thirdparty/Sophus/"
cp -a Thirdparty/DBoW2/DBoW2 Thirdparty/DBoW2/DUtils \
  "$STAGE/include/orbslam3/Thirdparty/DBoW2/"
cp -a Thirdparty/g2o/g2o "$STAGE/include/orbslam3/Thirdparty/g2o/"
install -m 0644 Thirdparty/g2o/config.h \
  "$STAGE/include/orbslam3/Thirdparty/g2o/config.h"

tar -xOf Vocabulary/ORBvoc.txt.tar.gz ORBvoc.txt \
  > "$STAGE/share/orbslam3/ORBvoc.txt"
patchelf --set-rpath '$ORIGIN:/opt/opencv-spacemit/lib' \
  "$STAGE/lib/libORB_SLAM3.so"
```

把仓库中的 RVV 专用 CMake 包配置安装到暂存目录。源文件使用 `-RVV`
后缀以区别于普通构建模板；安装后必须改为 CMake 约定的标准包配置文件名
`ORB_SLAM3Config.cmake`，这样 `find_package(ORB_SLAM3 CONFIG REQUIRED)`
才能找到它：

```bash
install -d "$STAGE/lib/cmake/ORB_SLAM3"
install -m 0644 \
  /path/to/orbslam3_ros2/vendor/ORB_SLAM3Config-RVV.cmake \
  "$STAGE/lib/cmake/ORB_SLAM3/ORB_SLAM3Config.cmake"
```

RVV 配置锁定 OpenCV 4.14.0，并提供 `ORB_SLAM3::ORB_SLAM3`、
`ORB_SLAM3::DBoW2` 和 `ORB_SLAM3::g2o` 目标。

审核暂存目录后再一次性安装：

```bash
sudo install -d /opt/orbslam3
sudo cp -a "$STAGE"/. /opt/orbslam3/

echo /opt/orbslam3/lib | sudo tee /etc/ld.so.conf.d/orbslam3.conf
sudo ldconfig
```

下游 CMake 可直接使用：

```cmake
find_package(Pangolin CONFIG REQUIRED)
find_package(ORB_SLAM3 CONFIG REQUIRED)
target_link_libraries(your_target PRIVATE ORB_SLAM3::ORB_SLAM3)
```

## 5. 处理 ROS 2 `cv_bridge` 的 OpenCV ABI

Humble 的二进制 `ros-humble-cv-bridge` 可能链接发行版 OpenCV 4.10，而本次
ORB-SLAM3 链接 OpenCV 4.14。一个进程同时加载这两个 ABI 不安全，因此需用
与已安装 deb 相同版本的 `vision_opencv` 源码重编 `cv_bridge`。本机 Humble
包版本对应 `3.2.1`：

```bash
source /opt/ros/humble/setup.bash
cd "$BUILD_ROOT"
git clone --branch 3.2.1 --depth 1 \
  https://github.com/ros-perception/vision_opencv.git

colcon --log-base cv-bridge-log build --base-paths vision_opencv \
  --packages-select cv_bridge \
  --build-base cv-bridge-build --install-base cv-bridge-stage \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_C_FLAGS_RELEASE="$RVV_C_FLAGS" \
    -DCMAKE_CXX_FLAGS_RELEASE="$RVV_CXX_FLAGS" \
    -DOpenCV_DIR="$OPENCV_DIR"
```

为避免覆盖或冒充系统 `libcv_bridge.so`，给专用库设置独立 SONAME 后将其纳入
`/opt/orbslam3`：

```bash
patchelf --set-soname libcv_bridge_orbslam3.so \
  "$BUILD_ROOT/cv-bridge-stage/lib/libcv_bridge.so"
patchelf --set-rpath '$ORIGIN:/opt/opencv-spacemit/lib' \
  "$BUILD_ROOT/cv-bridge-stage/lib/libcv_bridge.so"
ln -s libcv_bridge.so \
  "$BUILD_ROOT/cv-bridge-stage/lib/libcv_bridge_orbslam3.so"

sudo cp -a "$BUILD_ROOT/cv-bridge-stage/lib/libcv_bridge"* \
  /opt/orbslam3/lib/
sudo cp -a "$BUILD_ROOT/cv-bridge-stage/include/cv_bridge" \
  /opt/orbslam3/include/
sudo cp -a "$BUILD_ROOT/cv-bridge-stage/share/cv_bridge" \
  /opt/orbslam3/share/
sudo ldconfig
```

## 6. 编译和运行 ROS 2 包

```bash
source /opt/ros/humble/setup.bash
cd ~/orbslam3_ws
export CMAKE_PREFIX_PATH="/opt/orbslam3:/opt/opencv-spacemit:$CMAKE_PREFIX_PATH"

colcon build --symlink-install --cmake-clean-cache \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_C_FLAGS_RELEASE="$RVV_C_FLAGS" \
    -DCMAKE_CXX_FLAGS_RELEASE="$RVV_CXX_FLAGS" \
    -DOpenCV_DIR="$OPENCV_DIR"
source install/setup.bash
```

缓存中应分别指向 `/opt/orbslam3` 和 `/opt/opencv-spacemit`：

```bash
grep -R -E '^(ORB_SLAM3_DIR|Pangolin_DIR|cv_bridge_DIR|OpenCV_DIR):' \
  build/*/CMakeCache.txt
```

运行时无需设置指向 home 目录的 `LD_LIBRARY_PATH`：

```bash
ros2 launch orbslam3_ros2 rgbd_odometry.launch.py
```

默认词典来自 `${ORB_SLAM3_PREFIX:-/opt/orbslam3}/share/orbslam3/ORBvoc.txt`，
相机参数通过 ROS 2 package share 动态定位。也可覆盖词典、配置和话题：

```bash
ros2 launch orbslam3_ros2 rgbd_odometry.launch.py \
  vocabulary:=/another/location/ORBvoc.txt \
  settings:=/another/location/camera.yaml \
  color_topic:=/camera/camera/color/image_raw \
  depth_topic:=/camera/camera/aligned_depth_to_color/image_raw
```

## 7. 验证 RVV、依赖和运行链路

```bash
# 只应看到 OpenCV 4.14（DT_NEEDED 后缀为 .414），不能混入 .410。
readelf -d /opt/orbslam3/lib/libORB_SLAM3.so | \
  grep -E 'NEEDED|RPATH|RUNPATH'
readelf -d /opt/orbslam3/lib/libcv_bridge.so | \
  grep -E 'SONAME|NEEDED|RPATH|RUNPATH'
ldd /opt/orbslam3/lib/libORB_SLAM3.so | grep 'not found' && exit 1 || true

# ELF 属性至少应包含 v1p0 和 zvl256b；反汇编应能找到 RVV 指令。
readelf -A /opt/orbslam3/lib/libORB_SLAM3.so
objdump -d /opt/orbslam3/lib/libORB_SLAM3.so | \
  grep -m5 -E 'vsetvli|vsetivli|vle[0-9]+\.v|vse[0-9]+\.v'
```

真实 RGB-D 联调时，先启动深度对齐的相机节点，再启动 ORB-SLAM3，并确认
`tracking_state` 有输出：

```bash
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true \
  rgb_camera.color_profile:=640x480x15 \
  depth_module.depth_profile:=640x480x15

ros2 topic echo /orbslam3/tracking_state --once
```

## 8. 打包和部署

在构建机上打包完整前缀：

```bash
cd /opt
sudo tar czf "$HOME/orbslam3-riscv64-rvv.tar.gz" orbslam3
```

在目标机解压并注册动态库：

```bash
sudo tar xzf orbslam3-riscv64-rvv.tar.gz -C /opt
echo /opt/orbslam3/lib | sudo tee /etc/ld.so.conf.d/orbslam3.conf
sudo ldconfig
```

目标机还必须安装同版 `opencv-spacemit=4.14.0-2bb4`、ROS 2 运行依赖及
其他系统动态库。该包使用 `rv64gcv_zvl256b`，只适用于支持 RVV 1.0 且最小
向量长度不少于 256 bit 的 RISC-V CPU；不能用于 x86_64、ARM 或不满足该
ISA 条件的 RISC-V 主机。跨发行版部署时建议在目标系统按本文重新编译。

## 9. 本次验证记录

2026-09-14 在 `bianbu@10.0.91.131` 完成验证：独立构建目录为
`~/orbslam3-rvv-20260914`，安装前缀 `/opt/orbslam3` 约 151 MiB。ELF 属性
包含 RVV 1.0 和 `zvl256b`，反汇编可见 `vsetivli`、`vle64.v`、`vse64.v`
等指令；两个 ROS 2 包完成干净重编，运行文件仅加载 OpenCV 4.14。

使用 D455 的 640×480@15 彩色与对齐深度流做实际测试时，基础 RGB-D 节点
返回 `tracking_state=2` 并创建 1089 个地图点；集成建图节点同样返回状态
`2`、发布 `orb_map` 位姿并创建 1068 个地图点。测试结束后相机与 SLAM 进程
均已退出。本次未驱动底盘，也未改变激光雷达配置。
